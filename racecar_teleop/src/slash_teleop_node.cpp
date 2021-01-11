
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

class Teleop
{

public:
	Teleop() :
		maxVel_(6.0),
		maxVolt_(8.0),
		maxStAng_(40.0),
		ps4_(false),
		joystickCompatibilityWarned_(false)
	{        
		ros::NodeHandle pnh("~");
		pnh.param("max_vel", maxVel_, maxVel_);       // Max linear velocity (m/s)
		pnh.param("max_volt", maxVolt_, maxVolt_);    // Max voltage is set at 6 volts   
		pnh.param("max_angle", maxStAng_, maxStAng_); // Supposing +/- 40 degrees max for the steering angle
		pnh.param("ps4", ps4_, ps4_);                 // PlayStation4 controller: speed axis is 4
		cmd2rad_ = maxStAng_*2.0*3.1416/360.0;
		
		ros::NodeHandle nh;
		
		cmdVelPub_ = nh.advertise<geometry_msgs::Twist>("ctl_ref", 1);	
		joySub_ = nh.subscribe("joy", 1, &Teleop::joyCb, this);
	}

	virtual ~Teleop(){}

	void joyCb(const sensor_msgs::JoyConstPtr & joy_msg)
	{
        size_t min_axes = ps4_?5:4;
        if(joy_msg->axes.size() < min_axes || joy_msg->buttons.size() < 7)
        {
            if(!joystickCompatibilityWarned_) {
                ROS_WARN("slash_teleop: Received \"%s\" topic doesn't have enough axes (has %d, min=%d) and/ro buttons (has %d, min=7). If a Logitech gamepad is used, make sure also it is in D mode. This warning is only shown once. Until then, this node won't publish any \"%s\".", joySub_.getTopic().c_str(), (int)joy_msg->axes.size(), (int)min_axes, (int)joy_msg->buttons.size(), cmdVelPub_.getTopic().c_str());
                joystickCompatibilityWarned_ = true;
            }
            return;
        }

        joystickCompatibilityWarned_ = false;   // reset in case we switch mode on the gamepad

        float propulsion_user_input = joy_msg->axes[ps4_?4:3]; // Up-down Right joystick 
        float steering_user_input   = joy_msg->axes[0];        // Left-right left joystick
        
        geometry_msgs::Twist cmd_msg;             
                
        // Software deadman switch
        // If left button is active 
        if (joy_msg->buttons[4])
        {
            // If right button is active       
            if (joy_msg->buttons[5]) { 
                
                // Fully Open-Loop
                cmd_msg.linear.x  = propulsion_user_input * maxVolt_; // [volts]
                cmd_msg.angular.z = steering_user_input * cmd2rad_;
                cmd_msg.linear.z  = 1;  // Control mode
            }
                
            // If button A is active 
            else if(joy_msg->buttons[1]) {
                
                // Closed-loop position, Open-loop steering
                cmd_msg.linear.x  = propulsion_user_input; // [m]
                cmd_msg.angular.z = steering_user_input * cmd2rad_;
                cmd_msg.linear.z  = 3;  // Control mode
            }
                
            // If button B is active 
            else if(joy_msg->buttons[2]) {
                
                // Closed-loop velocity, Closed-loop steering 
                cmd_msg.linear.x  = propulsion_user_input * maxVel_; // [m/s]
                cmd_msg.angular.z = steering_user_input; // [m]
                cmd_msg.linear.z  = 5;  // Control mode
            }
                
            // If button x is active 
            else if(joy_msg->buttons[0]) {   
                
                // Closed-loop velocity, Closed-loop steering
                cmd_msg.linear.x  = propulsion_user_input * maxVel_; // [m/s]
                cmd_msg.angular.z = steering_user_input; // [m]
                cmd_msg.linear.z  = 6; // Control mode
            }
                
            // If button y is active 
            else if(joy_msg->buttons[3]) {
                
                // Reset Encoder
                cmd_msg.linear.x  = 0;
                cmd_msg.angular.z = 0;
                cmd_msg.linear.z  = 4;  // Control mode
            }
                
            // If left trigger is active 
            else if (joy_msg->buttons[6]) {
                
                // Joystick disabled!
                return;
            }
                
            // If right joy pushed
            else if(joy_msg->buttons[11]) {
                
                // Template
                cmd_msg.linear.x  = 0;
                cmd_msg.angular.z = 0;
                cmd_msg.linear.z  = 0; // Control mode
            }     
                       
            // If bottom arrow is active
            else if(joy_msg->axes[5]) {
                
                // Template
                cmd_msg.linear.x  = 0;
                cmd_msg.angular.z = 0;
                cmd_msg.linear.z  = 0; // Control mode
            }
            
            // Defaults operation
            // No active button
            else {
                
                // Closed-loop velocity, Open-loop steering
                cmd_msg.linear.x  = propulsion_user_input * maxVel_; // [m/s]
                cmd_msg.angular.z = steering_user_input * cmd2rad_;
                cmd_msg.linear.z  = 0; // Control mode
            }
        }
        
        // Deadman is un-pressed
        else {
            
            cmd_msg.linear.x = 0; 
            cmd_msg.linear.y = 0;
            cmd_msg.linear.z = -1;            
            cmd_msg.angular.x = 0;
            cmd_msg.angular.y = 0;
            cmd_msg.angular.z = 0;
		}
		
        // Publish cmd msg
        cmdVelPub_.publish( cmd_msg );
    }

private:
	double maxVel_;
	double maxVolt_;
	double maxStAng_;
	double cmd2rad_;
	bool ps4_;
	bool joystickCompatibilityWarned_;

	ros::Subscriber joySub_;
	ros::Publisher cmdVelPub_;
};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "teleop");
	Teleop teleop;
	ros::spin();
	return 0;
}

