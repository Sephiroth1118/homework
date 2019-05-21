#include <iostream>
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
using namespace std;
void chatterCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
ROS_INFO("%.f", (msg->header).stamp.toSec());
}
int main(int argc, char **argv)
{
ros::init(argc,argv,"homework_test");
ros::NodeHandle n;
ros::Publisher chatter_pub=n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
ros::Subscriber sub=n.subscribe("/original/scan",1000,chatterCallback);
geometry_msgs::Twist msg;
msg.angular.z=0.1;
ros::Rate loop_rate(2);
while (ros::ok())
{
ROS_INFO("%.3f",msg.angular.z);
chatter_pub.publish(msg);
ros::spinOnce();
loop_rate.sleep();
}
return 0;
}