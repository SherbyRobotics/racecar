
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>

ros::Publisher cloudPub;
laser_geometry::LaserProjection projector;

void scanCb(const sensor_msgs::LaserScanConstPtr& scan)
{
    // convert the message of type LaserScan to a PointCloud2
    sensor_msgs::PointCloud2 cloud;
    projector.projectLaser(*scan, cloud);

    // publish it
    cloud.header = scan->header;
    cloudPub.publish(cloud);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "laserscan_to_pointcloud");
	ros::NodeHandle nh;
	cloudPub = nh.advertise<sensor_msgs::PointCloud2>("cloud", 1);	
	ros::Subscriber scanSub = nh.subscribe("scan", 1, &scanCb);
	ros::spin();
	return 0;
}

