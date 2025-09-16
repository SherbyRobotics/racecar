
//=========================HEADER=============================================================
// Firmware for the Arduino managing the propulsion of the slash platform (UdeS Racecar)
//============================================================================================

/////////////////////////////////////////////////////////////////
// Includes
///////////////////////////////////////////////////////////////////

#include <Arduino.h>
#include <SPI.h>
#include <Servo.h>

#include "PBUtils.h"
#include "floatarray.pb.h"
#define USB_USBCON
#define IMU

// IMU
#ifdef IMU
#include "MPU9250.h"
MPU9250 imu(Wire, 0x68);
#endif

///////////////////////////////////////////////////////////////////
// Init I/O
///////////////////////////////////////////////////////////////////

// Servo objects for PWM control of
// Sterring servo-motor
Servo steeringServo;

// Slave Select pins for the encoder
const int slaveSelectEnc = 45;

// Pins for outputs PWM
const int str_pin = 9; // Servo

// Custom drive
const int dri_pwm_pin = 6;  // H bridge drive pwm
const int dri_dir_pin = 42; //

// debug
long timer_debug      = 0;
long time_micros      = 0;
long time_micros_last = 0;

// Prototype
void cmdCallback();
void sensorsCallback(unsigned long dt);

///////////////////////////////////////////////////////////////////
// Parameters
///////////////////////////////////////////////////////////////////

// Controller

// TODO: VOUS DEVEZ DETERMINEZ DES BONS PARAMETRES SUIVANTS
const float filter_rc  = 0.1;
const float vel_kp     = 10.0;
const float vel_ki     = 0.0;
const float vel_kd     = 0.0;
const float pos_kp     = 1.0;
const float pos_kd     = 0.0;
const float pos_ki     = 0.0;
const float pos_ei_sat = 10000.0;

// Loop period
const unsigned long time_period_low  = 2;    // 500 Hz for internal PID loop
const unsigned long time_period_high = 25;   // 50 Hz  for ROS communication
const unsigned long time_period_com  = 1000; // 1000 ms = max com delay (watchdog)

// Hardware min-zero-max range for the steering servo and the drive
const int min_str_angle = 45;
const int zer_str_angle = 90;
const int max_str_angle = 135;
const int pwm_min_dri   = -511;
const int pwm_zer_dri   = 0;
const int pwm_max_dri   = 511;

const int dri_wakeup_time = 20; // micro second

// Units Conversion
const double batteryV             = 8;
const double maxAngle             = 30;                             // max steering angle in deg
const double steeringAngleStep    = (zer_str_angle - min_str_angle) / maxAngle;
const double steeringCmd2SerAngle = steeringAngleStep * RAD_TO_DEG; // cmd is in rad but servo are control in deg
const double volt2pwm             = (pwm_zer_dri - pwm_min_dri) / batteryV;
const double tick2m               = 0.000002752;                    // To confirm

///////////////////////////////////////////////////////////////////
// Memory
///////////////////////////////////////////////////////////////////

// Inputs
float str_ref   = 0; // rad
float dri_ref   = 0; // volt
int ctl_mode    = 0; // discrete control mode
int dri_standby = 0;

// Ouputs
int str_angle = 0;
int dri_pwm   = 0;
float dri_cmd = 0;

// Controller memory (differentiation, filters and integral actions)
signed long enc_now = 0;
signed long enc_old = 0;

float pos_now = 0;
float vel_now = 0;
float vel_old = 0;

float vel_error_int = 0;
float pos_error_int = 0;

// Loop timing
unsigned long time_now       = 0;
unsigned long time_last_low  = 0;
unsigned long time_last_high = 0;
unsigned long time_last_com  = 0; // com watchdog

// For odometry
signed long enc_last_high = 0;

///////////////////////////////////////////////////////////////////
// Encoder init/read/reset functions
///////////////////////////////////////////////////////////////////

//////////////////////////////////////
void initEncoder()
{
    // Set slave selects as outputs
    pinMode(slaveSelectEnc, OUTPUT);

    // Raise select pin
    // Communication begins when you drop the individual select signsl
    digitalWrite(slaveSelectEnc, HIGH);

    SPI.begin();

    // Initialize encoder
    //    Clock division factor: 0
    //    Negative index input
    //    free-running count mode
    //    x4 quatrature count mode (four counts per quadrature cycle)
    // NOTE: For more information on commands, see datasheet
    digitalWrite(slaveSelectEnc, LOW);  // Begin SPI conversation
    SPI.transfer(0x88);                 // Write to MDR0
    SPI.transfer(0x03);                 // Configure to 4 byte mode
    digitalWrite(slaveSelectEnc, HIGH); // Terminate SPI conversation
}

//////////////////////////////////////
long readEncoder()
{
    // Initialize temporary variables for SPI read
    unsigned int count_1, count_2, count_3, count_4;
    long count_value;

    // Read encoder
    digitalWrite(slaveSelectEnc, LOW);  // Begin SPI conversation
    SPI.transfer(0x60);                 // Request count
    count_1 = SPI.transfer(0x00);       // Read highest order byte
    count_2 = SPI.transfer(0x00);
    count_3 = SPI.transfer(0x00);
    count_4 = SPI.transfer(0x00);       // Read lowest order byte
    digitalWrite(slaveSelectEnc, HIGH); // Terminate SPI conversation

    // Calculate encoder count
    count_value = (count_1 << 8) + count_2;
    count_value = (count_value << 8) + count_3;
    count_value = (count_value << 8) + count_4;

    return count_value;
}

//////////////////////////////////////
void clearEncoderCount()
{
    // Set encoder1's data register to 0
    digitalWrite(slaveSelectEnc, LOW); // Begin SPI conversation
    // Write to DTR
    SPI.transfer(0x98);
    // Load data
    SPI.transfer(0x00);                 // Highest order byte
    SPI.transfer(0x00);
    SPI.transfer(0x00);
    SPI.transfer(0x00);                 // lowest order byte
    digitalWrite(slaveSelectEnc, HIGH); // Terminate SPI conversation

    delayMicroseconds(100);             // provides some breathing room between SPI conversations

    // Set encoder1's current data register to center
    digitalWrite(slaveSelectEnc, LOW);  // Begin SPI conversation
    SPI.transfer(0xE0);
    digitalWrite(slaveSelectEnc, HIGH); // Terminate SPI conversation
}

///////////////////////////////////////////////////////////////////
// Convertion functions
///////////////////////////////////////////////////////////////////

// Convertion function : steering Angle --> Servo Angle
// Note steering angle is assume to be in rad and to represent
// the angle of the interior wheel during the rotation
double steering2ServoAngle(double steeringCmd)
{
    // Scale and offset
    int servo_angle = (int)(steeringCmd * steeringCmd2SerAngle + (double)zer_str_angle);

    // Saturations
    if (servo_angle > max_str_angle)
    {
        servo_angle = max_str_angle;
    }
    if (servo_angle < min_str_angle)
    {
        servo_angle = min_str_angle;
    }

    return servo_angle;
}

// Convertion function : Volt Command --> PWM
double cmd2pwm(double cmd)
{
    int pwm = (int)(cmd / batteryV * pwm_max_dri + 0.5);

    // Saturations
    if (pwm < pwm_min_dri)
    {
        pwm = pwm_min_dri;
    }
    if (pwm > pwm_max_dri)
    {
        pwm = pwm_max_dri;
    }

    return pwm;
}

///////////////////////////////////////////////////////////////////
// Set PWM value
///////////////////////////////////////////////////////////////////
void set_pwm(int pwm)
{
    // Zero cmd
    if (pwm == 0)
    {
        digitalWrite(dri_pwm_pin, LOW);
        dri_standby = 1;
    }

    // Non-zero PWM
    else
    {
        // Wake-up PWM if if needed
        if (dri_standby == 1)
        {
            digitalWrite(dri_pwm_pin, HIGH);
            delayMicroseconds(dri_wakeup_time);
            dri_standby = 0;
        }

        // PWM direction
        if (pwm < 0)
        {
            digitalWrite(dri_dir_pin, HIGH);
        }
        else
        {
            digitalWrite(dri_dir_pin, LOW);
        }

        // Registery-based pwm duty cycle adjustement

        // Fast PWM, 9-bit, prescaler divider = 1
        TCCR4A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM41);
        TCCR4B = _BV(CS20) | _BV(WGM42);

        OCR4A = abs(pwm) - 1; // set the duty cycle of pin 6
    }
}

// ==================== TOPICS ====================
// Out
FloatArray sensorsMsg = FloatArray_init_zero;

// In
FloatArray cmdMsg = FloatArray_init_zero;

const Topic topics[] = {
    // Out
    {SENSORS, FloatArray_fields, &sensorsMsg},
    // In
    {CMD, FloatArray_fields, &cmdMsg},
};

// ==================== SERIAL COMMUNICATION ====================
const String START_DELIMITER = "<{";
const String END_DELIMITER   = ">}";

bool recvInProgress     = false;
int inCmdIndex          = 0;
char inCmd[MAX_MSG_LEN] = {"\0"};
bool inCmdComplete      = false;
int inCmdType           = -1;
int nbsNewMsgs          = 0;
int newMsgsIds[MAX_NBS_MSG];
bool msgDiscardedLength = false;

PBUtils pbUtils(topics);

const unsigned long baud_rate = 115200;

///////////////////////////////////////////////////////////////////
// Controller One tick
///////////////////////////////////////////////////////////////////
void ctl(int dt_low)
{
    ///////////////////////////////////////////////
    // STEERING CONTROL
    ///////////////////////////////////////////////

    // Servo Open-Loop fonction
    str_angle = steering2ServoAngle(str_ref);
    steeringServo.write(str_angle);

    ///////////////////////////////////////////////
    // PROPULSION CONTROL
    ///////////////////////////////////////////////

    // Retrieve current encoder counters
    enc_now = readEncoder();

    // Position computation
    pos_now = (float)enc_now * tick2m;

    // Velocity computation

    // TODO: VOUS DEVEZ COMPLETEZ LA DERIVEE FILTRE ICI
    float vel_raw = (enc_now - enc_old) * tick2m / dt_low * 1000;
    float alpha   = 0;       // TODO
    float vel_fil = vel_raw; // Filter TODO

    // Propulsion Controllers

    //////////////////////////////////////////////////////
    if (ctl_mode == 0)
    {
        // Zero output
        dri_pwm = pwm_zer_dri;

        // reset integral actions
        vel_error_int = 0;
        pos_error_int = 0;
    }
    //////////////////////////////////////////////////////
    else if (ctl_mode == 1)
    {
        // Fully Open-Loop
        // Commands received in [Volts] directly
        dri_cmd = dri_ref;
        dri_pwm = cmd2pwm(dri_cmd);

        // reset integral actions
        vel_error_int = 0;
        pos_error_int = 0;
    }
    //////////////////////////////////////////////////////
    else if (ctl_mode == 2)
    {
        // Low-level Velocity control
        // Commands received in [m/sec] setpoints

        float vel_ref, vel_error;

        // TODO: VOUS DEVEZ COMPLETEZ LE CONTROLLEUR SUIVANT
        vel_ref       = dri_ref;
        vel_error     = vel_ref - vel_fil;
        vel_error_int = 0;                  // TODO
        dri_cmd       = vel_kp * vel_error; // proportionnal only

        dri_pwm = cmd2pwm(dri_cmd);
    }
    ///////////////////////////////////////////////////////
    else if (ctl_mode == 3)
    {
        // Low-level Position control
        // Commands received in [m] setpoints

        float pos_ref, pos_error, pos_error_ddt;

        // TODO: VOUS DEVEZ COMPLETEZ LE CONTROLLEUR SUIVANT
        pos_ref       = dri_ref;
        pos_error     = 0; // TODO
        pos_error_ddt = 0; // TODO
        pos_error_int = 0; // TODO

        // Anti wind-up
        if (pos_error_int > pos_ei_sat)
        {
            pos_error_int = pos_ei_sat;
        }

        dri_cmd = 0; // TODO

        dri_pwm = cmd2pwm(dri_cmd);
    }
    ///////////////////////////////////////////////////////
    else if (ctl_mode == 4)
    {
        // Reset encoder counts

        clearEncoderCount();

        // reset integral actions
        vel_error_int = 0;
        pos_error_int = 0;

        dri_pwm = pwm_zer_dri;
    }
    ////////////////////////////////////////////////////////
    else
    {
        // reset integral actions
        vel_error_int = 0;
        pos_error_int = 0;

        dri_pwm = pwm_zer_dri;
    }
    ///////////////////////////////////////////////////////

    // H-bridge pwm update
    set_pwm(dri_pwm);

    // Update memory variable
    enc_old = enc_now;
    vel_old = vel_fil;
}

///////////////////////////////////////////////////////////////////
// Arduino Initialization
///////////////////////////////////////////////////////////////////
void setup()
{
    Serial.begin(baud_rate);
    delay(10);

    // Init PWM output Pins
    steeringServo.attach(str_pin);
    pinMode(dri_dir_pin, OUTPUT);
    pinMode(dri_pwm_pin, OUTPUT);

    // Init and Clear Encoders
    initEncoder();
    clearEncoderCount();

    // Initialize Steering and drive cmd to neutral
    steeringServo.write(max_str_angle);
    set_pwm(0);

// Initialize
#ifdef IMU
    imu.begin();
    imu.setAccelRange(MPU9250::ACCEL_RANGE_2G);
    imu.setGyroRange(MPU9250::GYRO_RANGE_250DPS);
    imu.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_41HZ);
    imu.setSrd(9); // 100 Hz update rate
#endif
    //
    delay(3000);
    steeringServo.write(zer_str_angle);
}

void loop()
{
    time_now = millis();

    /////////////////////////////////////////////////////////////
    // Watchdog: stop the car if no recent communication from ROS
    //////////////////////////////////////////////////////////////

    if ((time_now - time_last_com) > time_period_com)
    {
        // All-stop
        dri_ref  = 0; // velocity set-point
        ctl_mode = 2; // closed-loop velocity mode
    }

    ////////////////////////////////////////
    // Low-level controller
    ///////////////////////////////////////

    if ((time_now - time_last_low) > time_period_low)
    {
        ctl(time_now - time_last_low);                // one control tick
        timer_debug = time_micros - time_micros_last; ///

        time_last_low    = time_now;
        time_micros_last = time_micros;
    }

    if (inCmdComplete)
    {
        inCmdComplete = false;
        bool status   = pbUtils.decodePb(inCmd, newMsgsIds, nbsNewMsgs);

        if (status && nbsNewMsgs > 0)
        {
            for (int i = 0; i < nbsNewMsgs; ++i)
            {
                switch (newMsgsIds[i])
                {
                case CMD:
                    cmdCallback();
                    break;
                default:
                    break;
                }
            }
        }
        else
        {
            inCmdType = -1;
        }
    }

    unsigned long dt = time_now - time_last_high;
    if (dt > time_period_high)
    {
        sensorsCallback(dt);
        time_last_high = time_now;
        enc_last_high  = enc_now;
    }
}

// ======================================== CALLBACKS ========================================

void cmdCallback()
{
    str_ref  = -cmdMsg.data[0]; // rad
    dri_ref  = cmdMsg.data[1];  // volt or m/s or m
    ctl_mode = cmdMsg.data[2];  // 1    or 2   or 3*/

    time_last_com = millis();
}

void sensorsCallback(unsigned long dt)
{
    sensorsMsg.data_count = 19;
    sensorsMsg.data[0]    = pos_now; // wheel position in m
    sensorsMsg.data[1]    = vel_old; // wheel velocity in m/sec

    // For DEBUG
    sensorsMsg.data[2] = (float)dri_ref; // set point received by arduino
    sensorsMsg.data[3] = (float)dri_cmd; // drive set point in volts
    // sensorsMsg.data[3] = (float)Serial.available(); // futile: we SHOULD NOT receive anything if Serial is not available.
    sensorsMsg.data[4] = (float)dri_pwm;            // drive set point in pwm
    sensorsMsg.data[5] = (float)enc_now;            // raw encoder counts
    sensorsMsg.data[6] = (float)str_ref;            // steering angle (don't remove/change, used for GRO830)
    sensorsMsg.data[7] = (float)(ctl_mode);         // for com debug
    sensorsMsg.data[8] = (float)dt;                 // time elapsed since last publish (don't remove/change, used for GRO830)
    sensorsMsg.data[9] =
        (enc_now - enc_last_high) * tick2m; // distance travelled since last publish (don't remove/change, used for GRO830)

// Read IMU (don't remove/change, used for GRO830)
#ifdef IMU
    imu.readSensor();
    sensorsMsg.data[10] = imu.getAccelX_mss();
    sensorsMsg.data[11] = imu.getAccelY_mss();
    sensorsMsg.data[12] = imu.getAccelZ_mss();
    sensorsMsg.data[13] = imu.getGyroX_rads();
    sensorsMsg.data[14] = imu.getGyroY_rads();
    sensorsMsg.data[15] = imu.getGyroZ_rads();
    sensorsMsg.data[16] = imu.getMagX_uT();
    sensorsMsg.data[17] = imu.getMagY_uT();
    sensorsMsg.data[18] = imu.getMagZ_uT();
#endif

    pbUtils.pbSend(1, SENSORS);
    Serial.flush();
}

// ======================================== SERIAL ========================================
void serialEvent()
{
    while (Serial.available() > 0)
    {
        char inChar         = Serial.read();
        int startDelimIndex = START_DELIMITER.indexOf(inChar);

        if (recvInProgress)
        {
            int endDelimIndex = END_DELIMITER.indexOf(inChar); // If start and end index is diff, will always return end
            if (endDelimIndex == -1)
            {
                inCmd[inCmdIndex++] = inChar;

                if (inCmdIndex >= MAX_MSG_LEN - 1) // If the last char isn't the end delimiter the message is not going to fit
                {
                    msgDiscardedLength = true;
                    inCmd[inCmdIndex]  = '\0';
                    recvInProgress     = false;
                    inCmdIndex         = 0;
                }
            }
            else
            {
                inCmd[inCmdIndex] = '\0';
                recvInProgress    = false;
                inCmdIndex        = 0;
                inCmdType         = endDelimIndex;
                inCmdComplete     = true;
            }
        }
        else if (startDelimIndex != -1)
            recvInProgress = true;
    }
}
