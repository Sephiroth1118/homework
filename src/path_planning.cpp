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

//坐标变换中用到的头文件
#include<tf/transform_broadcaster.h>
#include<tf/transform_listener.h>
#include<tf/message_filter.h>
#include<geometry_msgs/PointStamped.h>

//自定义消息类型
#include<homework/path.h>
#include<homework/location.h>

using namespace std;

#define MAXNUM 999999;


vector<geometry_msgs::Pose> path_pose; //地图数据坐标
geometry_msgs::Pose cur_pose;   //机器人当前坐标

double cur_x;
double cur_y;
double cur_theta;

double des_x;
double des_y;
double des_theta;

bool flag = false;

double th = 0.1;        //阈值

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

void pathCallback(const homework::path::ConstPtr& msg)
{
    path_pose.clear();
    for(int i = 0; i <msg->path.size(); i++)
    {   
        path_pose.push_back(msg->path[i]);
    }
    cout<<"received"<<endl;

    path_pose.push_back(msg->path[msg->path.size()-1]);
    flag = true;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    cur_pose.position.x = msg->pose.pose.position.x;
    cur_pose.position.y = msg->pose.pose.position.y;
    cur_pose.position.z = msg->pose.pose.position.z;

    cur_pose.orientation.x = msg->pose.pose.orientation.x;
    cur_pose.orientation.y = msg->pose.pose.orientation.y;
    cur_pose.orientation.z = msg->pose.pose.orientation.z;
    cur_pose.orientation.w = msg->pose.pose.orientation.w;

    cur_x = cur_pose.position.x;
    cur_y = cur_pose.position.y;
    cur_theta = Trans(cur_pose);
}


//计算两点间的欧氏距离
double EucDistance(geometry_msgs::Pose a, geometry_msgs::Pose b)
{
    //@TODO
    double distance = 0;

    double a_x = a.position.x;
    double a_y = a.position.y;

    double b_x = b.position.x;
    double b_y = b.position.y;

    distance = fabs(sqrt(pow(a_x,2)+pow(a_y,2)) - sqrt(pow(b_x,2)+pow(b_y,2)));
    return distance;
}

//返回一个vector中最小的元素的编号
int SmlNumber(vector<double> a)
{
    int smallest_number = 0;
    double smallest_data = MAXNUM;
    for(int i = 0; i<a.size(); i++)
    {
        if(a[i]<=smallest_data)
        {
            smallest_data = a[i];
            smallest_number = i;
        }
    }
    return smallest_number;
}

void judge()
{
    int number = 0;
    vector<double> Distance;

    for(int i = 0; i < path_pose.size(); i++)
    {
        double d = EucDistance(cur_pose,path_pose[i]);
        Distance.push_back(d);
    }

    number = SmlNumber(Distance);

    if(Distance[number] <= th)
    {
        if(number == path_pose.size()-1)
        {
            des_x = path_pose[number].position.x;
            des_y = path_pose[number].position.y;
            des_theta = 0;
        }
        else if(number == path_pose.size()-2)
        {
            des_x = path_pose[number+1].position.x;
            des_y = path_pose[number+1].position.y;
            des_theta = 0;
        }
        else
        {
            des_x = path_pose[number+1].position.x;
            des_y = path_pose[number+1].position.y;
            des_theta = atan2(path_pose[number+2].position.y - path_pose[number+1].position.y,path_pose[number+2].position.x-path_pose[number+1].position.x);
        }        
    }
    else
    {
        if(number == path_pose.size()-1)
        {
            des_x = path_pose[number].position.x;
            des_y = path_pose[number].position.y;
            des_theta = 0;
        }
        else if(number == path_pose.size()-2)
        {
            des_x = path_pose[number].position.x;
            des_y = path_pose[number].position.y;
            des_theta = atan2(path_pose[number+1].position.y - path_pose[number].position.y,path_pose[number+1].position.x-path_pose[number].position.x);
        }
        else
        {
            des_x = path_pose[number].position.x;
            des_y = path_pose[number].position.y;
            des_theta = atan2(path_pose[number+1].position.y - path_pose[number].position.y,path_pose[number+1].position.x-path_pose[number].position.x);
        } 
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "besier");
    ros::NodeHandle nh;

    ros::Rate loop_rate(5);  // 自循环频率

    //声明发布器和订阅器
    ros::Publisher pub_data = nh.advertise<homework::location>("/besier_to_controller",1);
    ros::Subscriber sub_data = nh.subscribe("/odom_plan",1,pathCallback);
    ros::Subscriber sub_odm_data = nh.subscribe("/sensors_fusion/odom",1,odomCallback);

    ROS_INFO("Start Planning!");

    while(ros::ok())
    {   
        ros::spinOnce();
        if(flag == true)
        {
            judge();
            homework::location mess;
            mess.x = des_x;
            mess.y = des_y;
            mess.theta = des_theta;
            mess.curvature = 0.0;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
