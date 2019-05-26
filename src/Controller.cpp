#include<ros/ros.h>
#include<iostream>
#include "homework/path.h"
#include<geometry_msgs/Pose.h>
#include<geometry_msgs/PoseWithCovarianceStamped.h>
#include<cmath>
#include<geometry_msgs/Twist.h>
#include<tf/transform_listener.h>
#include<geometry_msgs/PointStamped.h>
#include<nav_msgs/Odometry.h>

using namespace std;

void PathCallback(const homework::path::ConstPtr& msg);
void MapPathCallback(const homework::path::ConstPtr& msg);
void LocationCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
void OdomLocationCallback(const nav_msgs::Odometry::ConstPtr& msg);
void transformPoint(const tf::TransformListener& listener);
vector<geometry_msgs::Pose> path_pose;
geometry_msgs::Pose self_pose;
geometry_msgs::Pose odom_pose;
bool flag=0;
float step=3,dis=0,alph=0,beta=0,dir=0,angle_dif=0;
geometry_msgs::Twist vel;
float kp=0.06;
float ka=0.18;
float kb=-0.03;
int total_points=0;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Controller");
	ros::NodeHandle myNode;
	ros::Rate loop_rate(100);  // 自循环频率
	cout<<"welcome to Controller!"<<endl;
    ros::Subscriber path=myNode.subscribe("/odom_plan",10,PathCallback);
    ros::Subscriber map_path=myNode.subscribe("/path_plan",10,MapPathCallback);
    ros::Subscriber location=myNode.subscribe("/amcl_pose",10,LocationCallback);
    ros::Subscriber odom_location=myNode.subscribe("/sensors_fusion/odom",10,OdomLocationCallback);
    ros::Publisher vel_control=myNode.advertise<geometry_msgs::Twist>("/ctrl_cmd_vel",10);
    while(!flag)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    cout<<"path in odom frame:"<<endl;
    for(int i=0;i<path_pose.size();i++)
    {
        cout<<"("<<path_pose[i].position.x<<", "<<path_pose[i].position.y<<")"<<endl;
        cout<<"("<<path_pose[i].orientation.z<<", "<<path_pose[i].orientation.w<<")"<<endl;
    }
    cout<<"totally "<<path_pose.size()<<" points"<<endl;
    total_points=path_pose.size();
    cout<<"total_points= "<<total_points<<endl;
    for(int i=1;i<total_points;i++)
    {
        do
        {
            ros::spinOnce();
            float theta=asin(odom_pose.orientation.z);
            dis=sqrt(pow(path_pose[i].position.x-odom_pose.position.x,2)+pow(path_pose[i].position.y-odom_pose.position.y,2));
            dir=atan2(path_pose[i].position.y-odom_pose.position.y,path_pose[i].position.x-odom_pose.position.x);
            alph=-theta-dir;
            beta=-theta-alph+asin(path_pose[i].orientation.z);
            angle_dif=abs(theta-dir);
            if(angle_dif>2.36)
            {
                vel.linear.x=-kp*dis;
                vel.angular.z=ka*alph+kb*beta;
            }
            else
            {
                vel.linear.x=kp*dis;
                vel.angular.z=-(ka*alph+kb*beta);
            }
            vel.linear.y=0;
            vel.linear.z=0;
            vel.angular.x=0;
            vel.angular.y=0;
        
            vel_control.publish(vel);
            // cout<<"************************************************"<<endl;
            // cout<<"self_pose: "<<endl;
            // cout<<"  position:"<<endl;
            // cout<<"   x= "<<self_pose.position.x<<endl;
            // cout<<"   y= "<<self_pose.position.y<<endl;
            // cout<<"  orientation: "<<endl;
            // cout<<"   z= "<<self_pose.orientation.z<<endl;
            // cout<<"   w= "<<self_pose.orientation.w<<endl;
            // cout<<endl;
            // cout<<"path_pose"<<"["<<i<<"]: "<<endl;
            // cout<<"  position:"<<endl;
            // cout<<"   x= "<<path_pose[i].position.x<<endl;
            // cout<<"   y= "<<path_pose[i].position.y<<endl;
            // cout<<"  orientation: "<<endl;
            // cout<<"   z= "<<path_pose[i].orientation.z<<endl;
            // cout<<"   w= "<<path_pose[i].orientation.w<<endl;
            // cout<<endl;
            // cout<<"vel_twist: "<<endl;
            // cout<<"  linear:"<<endl;
            // cout<<"   x= "<<vel.linear.x<<endl;
            // cout<<"  angular: "<<endl;
            // cout<<"   z= "<<vel.angular.z<<endl;
            // cout<<endl;
            // cout<<"dis: "<<dis<<"; "<<"alph: "<<alph<<"; "<<"beta: "<<beta<<endl;
            // cout<<"************************************************"<<endl;
        } while (dis>0.4);
        cout<<"sucessfully reach "<<i<<" point!!!"<<endl;
        // if(i==path_pose.size()-1)
        // {
        //     cout<<"into the break loop!!!!!"<<endl;
        //     break;
        // }
    }
    cout<<"sucessfully reach the navigation goal!!!!!!!!"<<endl;
    return 0;
}

void PathCallback(const homework::path::ConstPtr& msg)
{
    for(int i=0;i<msg->path.size();i+=step)
    {
        path_pose.push_back(msg->path[i]);
    }
    flag=1;
}

void MapPathCallback(const homework::path::ConstPtr& msg)
{
    cout<<"path in map frame:"<<endl;
    for(int i=0;i<msg->path.size();i+=step)
    {
        cout<<"("<<msg->path[i].position.x<<", "<<msg->path[i].position.y<<")"<<endl;
    }
}

void LocationCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    self_pose=msg->pose.pose;
}

void OdomLocationCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    odom_pose=msg->pose.pose;
}
