//标准头文件
#include<ros/ros.h>
#include<iostream>
#include<std_msgs/String.h>
#include<string.h>
#include<cmath>
#include<vector>

//navigation中需要使用的位姿信息头文件
#include<geometry_msgs/Pose.h>
#include<geometry_msgs/Point.h>
#include<geometry_msgs/PoseWithCovariance.h>
#include<geometry_msgs/PoseWithCovarianceStamped.h>
#include<geometry_msgs/Twist.h>
#include<geometry_msgs/Quaternion.h>
#include<nav_msgs/Odometry.h>

//自定义消息类型
#include<homework/location.h>

using namespace std;

//定义全局变量
//订阅器和发布器
ros::Subscriber sub_ctr_data;               //订阅控制输入
ros::Subscriber sub_odom_data;              //订阅里程计消息
ros::Publisher pub_twist_data;              //发布速度

double cur_x;
double cur_y;
double cur_theta;
double target_x;
double target_y;
double target_theta;

double dx;
double dy;
double dtheta;

double v;
double w;

double k1;
double k2;

double e1;
double e2;
double e3;

void InputCallback(const homework::location::ConstPtr& msg)
{
    //@TODO
    target_x = msg->x;
    target_y = msg->y;
    target_theta = msg->theta;
}

void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    //@TODO
    geometry_msgs::Pose tmp;
    tmp.position.x = 0.0;
    tmp.position.y = 0.0;
    tmp.position.z = 0.0;
    tmp.orientation.x = msg->pose.pose.orientation.x;
    tmp.orientation.y = msg->pose.pose.orientation.y;
    tmp.orientation.z = msg->pose.pose.orientation.z;
    tmp.orientation.w = msg->pose.pose.orientation.w;

    cur_x = msg->pose.pose.position.x;
    cur_y = msg->pose.pose.position.y;
    cur_theta = Trans(tmp);
}

//将四元组转化为欧拉角
double Trans(geometry_msgs::Pose a)
{
    double x = a.orientation.x;
    double y = a.orientation.y;
    double z = a.orientation.z;
    double w = a.orientation.w;

    double theta = atan2(2*(w*z+x*y),1-2*(y*y+z*z));
    return theta;
}

//计算位姿误差
void cal_error()
{
    dx = cur_x - target_x;
    dy = cur_y - target_y;
    dtheta = cur_theta - target_theta;

    e1 = cos(cur_theta)*dx + sin(cur_theta)*dy;
    e2 = -sin(cur_theta)*dx + cos(cur_theta)*dy;
    e3 = dtheta;
}

//计算速度发布
void cal_vel()
{

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller_curve");
    ros::NodeHandle nh;

    ros::Rate loop_rate(5);  // 自循环频率

    pub_twist_data = nh.advertise<geometry_msgs::Twist>("/ctr_cmd_vel", 1);
    sub_ctr_data = nh.subscribe("/besier_to_controller", 1, InputCallback);
    sub_odom_data = nh.subscribe("sensor_fusions/odom", 1 , OdomCallback);

    ROS_INFO("Start controlling!");

    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}