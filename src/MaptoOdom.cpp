#include<iostream>
#include<ros/ros.h>
#include<tf/transform_broadcaster.h>
#include<tf/transform_listener.h>
#include<tf/message_filter.h>
#include<geometry_msgs/PointStamped.h>
#include "homework/path.h"

using namespace std;

vector<geometry_msgs::Pose> path_pose;
vector<geometry_msgs::Pose> odom_pose;
vector<geometry_msgs::PointStamped> odom_point;
ros::Publisher odom_path_pub;
homework::path odom_path;
float step=3;
bool flag=0;

void PathCallback(const homework::path::ConstPtr& msg)
{
    for(int i=0;i<msg->path.size();i+=step)
    {
        path_pose.push_back(msg->path[i]);
        odom_pose.push_back(msg->path[i]);
    }
    flag=1;
}


void transformPoint(const tf::TransformListener& listener){
  //if(odom_pose.size()!=0) return;
  geometry_msgs::PointStamped map_point;
    map_point.header.frame_id = "map";
    odom_point.clear();
    for(int i=0;i<path_pose.size();i++)
    {
        //we'll just use the most recent transform available for our simple example
        map_point.header.stamp = ros::Time();

        //just an arbitrary point in space
        map_point.point.x = path_pose[i].position.x;
        map_point.point.y = path_pose[i].position.y;
        map_point.point.z = path_pose[i].position.z;

        try{
            geometry_msgs::PointStamped temp;
            listener.transformPoint("odom", map_point, temp);

            printf("map: (%.2f, %.2f. %.2f) -----> odom: (%.2f, %.2f, %.2f) at time %.2f",
                    map_point.point.x, map_point.point.y, map_point.point.z,
                    temp.point.x, temp.point.y, temp.point.z, temp.header.stamp.toSec());
            odom_point.push_back(temp);
            
        }
        catch(tf::TransformException& ex)
        {
            ROS_ERROR("Received an exception trying to transform a point from \"map\" to \"odom\": %s", ex.what());
        }
    }
    for(int i=0;i<odom_pose.size();i++)
    {
      odom_pose[i].position.x=odom_point[i].point.x;
      odom_pose[i].position.y=odom_point[i].point.y;
      odom_pose[i].position.z=odom_point[i].point.z;
    }
    odom_path.path=odom_pose;
    odom_path_pub.publish(odom_path);
    //return;
}

int main(int argc, char** argv)
{
  ros::init(argc,argv,"MaptoOdom");
  ros::NodeHandle myNode;
  ros::Rate r(10);
  tf::TransformBroadcaster broadcaster;
  ros::Rate loop_rate(0.1);  // 自循环频率
  ros::Subscriber path=myNode.subscribe("/path_plan",10,PathCallback);
  odom_path_pub=myNode.advertise<homework::path>("/odom_plan",10);
  while(!flag)
    {
      ros::spinOnce();
      loop_rate.sleep();
    }
  
  while(ros::ok())
  {
    tf::TransformListener listener(ros::Duration(1));
    //we'll transform a point once every second
    ros::Timer timer = myNode.createTimer(ros::Duration(1.0), boost::bind(&transformPoint, boost::ref(listener)));
    ros::spin();
    
    r.sleep();
  }
    return 0;
}