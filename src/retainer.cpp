//标准头文件
#include<ros/ros.h>
#include<iostream>
#include<std_msgs/String.h>
#include<string.h>
//navigation中的位姿信息坐标
#include<geometry_msgs/Twist.h>

using namespace std;


//定义全局变量
geometry_msgs::Twist twist;

//订阅器和发布器
ros::Subscriber sub_twist;
ros::Publisher pub_twist;

void init()
{
    //线速度
    twist.linear.x = 0;
    twist.linear.y = 0;
    twist.linear.z = 0;

    //角速度
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = 0;
}

void TwistCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    //线速度
    twist.linear.x = msg->linear.x/1.2;
    twist.linear.y = msg->linear.y/1.2;
    twist.linear.z = msg->linear.z/1.2;

    //角速度
    twist.angular.x = msg->angular.x/1.2;
    twist.angular.y = msg->angular.y/1.2;
    twist.angular.z = msg->angular.z/1.2;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "retainer");
    ros::NodeHandle nh;
    sub_twist = nh.subscribe("/Middle_twist", 1, TwistCallback);
    pub_twist = nh.advertise<geometry_msgs::Twist>("/ctrl_cmd_vel", 10);

    init();
    ros::Rate loop_rate(3);
    while(ros::ok())
    {
         //发布速度
        pub_twist.publish(twist);
        //cout<<"------------------";
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;   
}