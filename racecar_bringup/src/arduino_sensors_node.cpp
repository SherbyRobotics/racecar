
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/JointState.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

class ArduinoSensors
{

public:
	ArduinoSensors() :
		l_(0.34),
		w_(0.20),
		radius_(0.05),
		childFrameId_("base_footprint"),
		publishTf_(true),
		tfPrefix_("racecar"),
		x_(0.0),
        y_(0.0),
        theta_(0.0),
        totalDistance_(0.0)
	{      
		ros::NodeHandle pnh("~");
		pnh.param("wheelbase", l_, l_);
		pnh.param("axle_track", w_, w_);
		pnh.param("wheel_radius", radius_, radius_);
		pnh.param("child_frame_id", childFrameId_, childFrameId_);
		pnh.param("publish_tf", publishTf_, publishTf_);
		pnh.param("tf_prefix", tfPrefix_, tfPrefix_);
		
		ros::NodeHandle nh;
		
		odomPub_ = nh.advertise<nav_msgs::Odometry>("odom", 1);
		imuPub_ = nh.advertise<sensor_msgs::Imu>("imu/data_raw", 1);
		magPub_ = nh.advertise<sensor_msgs::MagneticField>("imu/mag", 1);
		jointsPub_ = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
		
		rawOdomSub_ = nh.subscribe("raw_odom", 1, &ArduinoSensors::rawOdomCb, this);
	}

	virtual ~ArduinoSensors(){}

	void rawOdomCb(const std_msgs::Float32MultiArrayConstPtr & msg)
	{
        if(msg->data.size() != 19) {
            ROS_ERROR("Received data from arduino should have a length of 19! current length=%d, make sure you have the latest arduino firmware installed.", (int)msg->data.size());
            return;
        }
        
        double elapsed_seconds = msg->data[8]/1000.0;
        double totalEncDistance = msg->data[0];
        double speed = msg->data[9]/elapsed_seconds;
        double distance = speed*elapsed_seconds;
        double steering_angle = -msg->data[6];
         
        //IMU
        double linear_acceleration_x = msg->data[10];
        double linear_acceleration_y = msg->data[11];
        double linear_acceleration_z = msg->data[12];
        double angular_velocity_x = msg->data[13];
        double angular_velocity_y = msg->data[14];
        double angular_velocity_z = msg->data[15];
        double magnetic_x = msg->data[16];
        double magnetic_y = msg->data[17];
        double magnetic_z = msg->data[18];

        if(elapsed_seconds <= 0.0) {
            ROS_WARN("elapsed_seconds is 0.");
            return;
        }
        
        double v_x = speed * std::cos(theta_);
        double v_y = speed * std::sin(theta_);
        double v_theta = speed * std::tan(steering_angle) / l_;

        x_ = x_ + v_x * elapsed_seconds;
        y_ = y_ + v_y * elapsed_seconds;
        theta_ = theta_ + v_theta * elapsed_seconds;

        ros::Time now = ros::Time::now();
        sendOdometry(now, x_, y_, theta_, speed, 0, v_theta);

        sendImu(now, 
        	linear_acceleration_x, linear_acceleration_y, linear_acceleration_z,
            angular_velocity_x, angular_velocity_y, angular_velocity_z);
         
        sendMag(now, magnetic_x, magnetic_y, magnetic_z);

        sendWheelJoints(now, steering_angle, distance);   
    }
    
    void sendOdometry(const ros::Time & stamp, double x, double y, double theta, double v_x, double v_y, double v_theta)
    {
        nav_msgs::Odometry msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = tfPrefix_+"/odom";
        msg.child_frame_id = tfPrefix_+"/"+childFrameId_;

        msg.pose.pose.position.x = x;
        msg.pose.pose.position.y = y;

        tf2::Quaternion q;
        q.setRPY( 0, 0, theta );
        msg.pose.pose.orientation.x = q.x();
        msg.pose.pose.orientation.y = q.y();
        msg.pose.pose.orientation.z = q.z();
        msg.pose.pose.orientation.w = q.w();

        msg.twist.twist.linear.x = v_x;
        msg.twist.twist.linear.y = v_y;

        msg.twist.twist.angular.z = v_theta;

        odomPub_.publish(msg);

        if(publishTf_) {
        	geometry_msgs::TransformStamped transformStamped;
        	transformStamped.transform.translation.x = x;
        	transformStamped.transform.translation.y = y;
			transformStamped.transform.rotation.x = q.x();
			transformStamped.transform.rotation.y = q.y();
			transformStamped.transform.rotation.z = q.z();
			transformStamped.transform.rotation.w = q.w();
        	transformStamped.header.frame_id = msg.header.frame_id;
        	transformStamped.child_frame_id = msg.child_frame_id;
			transformStamped.header.stamp = stamp;
            tfBroadcaster_.sendTransform(transformStamped);
        }
    }

    void sendImu(const ros::Time & stamp,
            double linear_acceleration_x, double linear_acceleration_y, double linear_acceleration_z,
            double angular_velocity_x, double angular_velocity_y, double angular_velocity_z)
    {
        sensor_msgs::Imu msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = tfPrefix_+"/imu_link";

        msg.linear_acceleration.x = linear_acceleration_x;
        msg.linear_acceleration.y = linear_acceleration_y;
        msg.linear_acceleration.z = linear_acceleration_z;

        msg.angular_velocity.x = angular_velocity_x;
        msg.angular_velocity.y = angular_velocity_y;
        msg.angular_velocity.z = angular_velocity_z;

        imuPub_.publish(msg);
    }
        
    void sendMag(const ros::Time & stamp, double x, double y, double z)
    {
        sensor_msgs::MagneticField msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = tfPrefix_+"/imu_link";

        msg.magnetic_field.x = x;
        msg.magnetic_field.y = y;
        msg.magnetic_field.z = z;
        
        magPub_.publish(msg);
    }

    void sendWheelJoints(const ros::Time & stamp, double angle, double distance)
    {
        totalDistance_ += distance;
        double rotation = totalDistance_ / radius_;
        sensor_msgs::JointState joint_state;
        joint_state.header.stamp = stamp;
        joint_state.name.push_back("left_front_wheel_joint");
        joint_state.name.push_back("left_rear_wheel_joint");
        joint_state.name.push_back("right_rear_wheel_joint");
        joint_state.name.push_back("right_front_wheel_joint");
        joint_state.name.push_back("left_steering_hinge_joint");
        joint_state.name.push_back("right_steering_hinge_joint");

        double lsinphy = 2*l_*std::sin(angle);
        double lcosphy = 2*l_*std::cos(angle);
        double wsinphy = w_*std::sin(angle);
        double steering_angle_left = std::atan(lsinphy / (lcosphy-wsinphy));
        double steering_angle_right = std::atan(lsinphy / (lcosphy+wsinphy));
        joint_state.position.push_back(rotation);
        joint_state.position.push_back(rotation);
        joint_state.position.push_back(rotation);
        joint_state.position.push_back(rotation);
        joint_state.position.push_back(steering_angle_left);
        joint_state.position.push_back(steering_angle_right);
        jointsPub_.publish(joint_state);
    }

private:
	// Parameters
	double l_;
	double w_;
	double radius_;
	std::string childFrameId_;
	bool publishTf_;
	std::string tfPrefix_;
	
	//State space variables
	double x_ = 0.0;
	double y_ = 0.0;
	double theta_ = 0.0;
	double totalDistance_ = 0.0;

	ros::Subscriber rawOdomSub_;

	ros::Publisher odomPub_;
	ros::Publisher imuPub_;
	ros::Publisher magPub_;
	ros::Publisher jointsPub_;
	
	tf2_ros::TransformBroadcaster tfBroadcaster_;
};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "arduino_sensors");
	ArduinoSensors arduinoSensors;
	ros::spin();
	return 0;
}

