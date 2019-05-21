#include<iostream>
#include<ros/ros.h>
#include<tf/transform_broadcaster.h>
#include<tf/transform_listener.h>
#include<tf/message_filter.h>
#include<geometry_msgs/PointStamped.h>

using namespace std;

void transformPoint(const tf::TransformListener& listener){
  //we'll create a point in the base_laser frame that we'd like to transform to the base_link frame
  geometry_msgs::PointStamped map_point;
  map_point.header.frame_id = "map";

  //we'll just use the most recent transform available for our simple example
  map_point.header.stamp = ros::Time();

  //just an arbitrary point in space
  map_point.point.x = 12.0;
  map_point.point.y = 34.2;
  map_point.point.z = 0.0;

  try{
    geometry_msgs::PointStamped base_point;
    listener.transformPoint("odom", map_point, base_point);

    printf("map: (%.2f, %.2f. %.2f) -----> odom: (%.2f, %.2f, %.2f) at time %.2f",
        map_point.point.x, map_point.point.y, map_point.point.z,
        base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
  }
  catch(tf::TransformException& ex){
    ROS_ERROR("Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());
  }
}

int main(int argc, char** argv)
{
  ros::init(argc,argv,"MaptoOdom");
  ros::NodeHandle myNode;
  ros::Rate r(100);
  tf::TransformBroadcaster broadcaster;

  while(ros::ok())
  {
    //send transform
    // broadcaster.sendTransform(
    // tf::StampedTransform(
    // tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.1, 0.0, 0.2)),
    // ros::Time::now(),"base_link", "map"));

    //subscribe transform
    tf::TransformListener listener(ros::Duration(10));

    //we'll transform a point once every second
    ros::Timer timer = myNode.createTimer(ros::Duration(1.0), boost::bind(&transformPoint, boost::ref(listener)));

    ros::spin();
    r.sleep();
  }
    return 0;
}