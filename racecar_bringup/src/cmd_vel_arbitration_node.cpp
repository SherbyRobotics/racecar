
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>

class Arbitration
{

public:
	Arbitration() :
		delaySec_(0.5)
	{
		ros::NodeHandle pnh("~");
		pnh.param("delay_sec", delaySec_, delaySec_);
		
		timeCalled_.resize(8, ros::Time::now().toSec());

		ros::NodeHandle nh;
		
		cmdVelPub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel_output", 1);
		statusPub_ = nh.advertise<std_msgs::Int32>("status", 1);
		
		cmdVelSub0_ = nh.subscribe("cmd_vel_abtr_0", 1, &Arbitration::cmdVelCallback0, this);
		cmdVelSub1_ = nh.subscribe("cmd_vel_abtr_1", 1, &Arbitration::cmdVelCallback1, this);
		cmdVelSub2_ = nh.subscribe("cmd_vel_abtr_2", 1, &Arbitration::cmdVelCallback2, this);
		cmdVelSub3_ = nh.subscribe("cmd_vel_abtr_3", 1, &Arbitration::cmdVelCallback3, this);
		cmdVelSub4_ = nh.subscribe("cmd_vel_abtr_4", 1, &Arbitration::cmdVelCallback4, this);
		cmdVelSub5_ = nh.subscribe("cmd_vel_abtr_5", 1, &Arbitration::cmdVelCallback5, this);
		cmdVelSub6_ = nh.subscribe("cmd_vel_abtr_6", 1, &Arbitration::cmdVelCallback6, this);
		cmdVelSub7_ = nh.subscribe("cmd_vel_abtr_7", 1, &Arbitration::cmdVelCallback7, this);
		
		
	}

	virtual ~Arbitration(){}

	void cmdVelCallback(const geometry_msgs::TwistConstPtr & msg, int priority)
	{
        timeCalled_[priority] = ros::Time::now().toSec();
        bool pub = true;
        for(int i=1; i<priority+1; ++i)
        {
            if(timeCalled_[priority] - timeCalled_[i-1] < delaySec_) {
                pub=false;
                break;
            }
        }
        if(pub) {
            cmdVelPub_.publish(msg);
            std_msgs::Int32 status;
            status.data = priority;
            statusPub_.publish(status);
        }
    }

	void cmdVelCallback0(const geometry_msgs::TwistConstPtr & msg) {
		cmdVelCallback(msg, 0);
	}
	void cmdVelCallback1(const geometry_msgs::TwistConstPtr & msg) {
		cmdVelCallback(msg, 1);
	}
	void cmdVelCallback2(const geometry_msgs::TwistConstPtr & msg) {
		cmdVelCallback(msg, 2);
	}
	void cmdVelCallback3(const geometry_msgs::TwistConstPtr & msg) {
		cmdVelCallback(msg, 3);
	}
	void cmdVelCallback4(const geometry_msgs::TwistConstPtr & msg) {
		cmdVelCallback(msg, 4);
	}
	void cmdVelCallback5(const geometry_msgs::TwistConstPtr & msg) {
		cmdVelCallback(msg, 5);
	}
	void cmdVelCallback6(const geometry_msgs::TwistConstPtr & msg) {
		cmdVelCallback(msg, 6);
	}
	void cmdVelCallback7(const geometry_msgs::TwistConstPtr & msg) {
		cmdVelCallback(msg, 7);
	}

private:
	double delaySec_;

	ros::Subscriber cmdVelSub0_;
	ros::Subscriber cmdVelSub1_;
	ros::Subscriber cmdVelSub2_;
	ros::Subscriber cmdVelSub3_;
	ros::Subscriber cmdVelSub4_;
	ros::Subscriber cmdVelSub5_;
	ros::Subscriber cmdVelSub6_;
	ros::Subscriber cmdVelSub7_;
	ros::Publisher cmdVelPub_;
	ros::Publisher statusPub_;
	std::vector<double> timeCalled_;
};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "cmd_vel_arbitration");
	Arbitration arbitration;
	ros::spin();
	return 0;
}

