//标准头文件
#include<ros/ros.h>
#include<iostream>
#include<std_msgs/String.h>
#include<string.h>
//navigation中需要使用的位姿信息头文件
#include<geometry_msgs/Pose.h>
#include<geometry_msgs/Point.h>
#include<geometry_msgs/PoseWithCovariance.h>
#include<geometry_msgs/PoseWithCovarianceStamped.h>
#include<geometry_msgs/Twist.h>
#include<geometry_msgs/Quaternion.h>
//move_base头文件
#include<move_base_msgs/MoveBaseGoal.h>
#include<move_base_msgs/MoveBaseAction.h>
//actionlib头文件
#include<actionlib/client/simple_action_client.h>
#include<stdlib.h>
#include<cstdlib>
//播放声音
#include<sound_play/sound_play.h>

using namespace std;

//定义全局变量
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient; //简化类型书写为MoveBaseClient

bool is_go = false;

geometry_msgs::Pose initial_pose;
geometry_msgs::Pose goal_pose;

ros::Subscriber sub_sound;

void initpose()
{
    //初始位置
    initial_pose.position.x = 0.0;
    initial_pose.position.y = 0.0;
    initial_pose.position.z = 0.0;

    initial_pose.orientation.x = 0.0;
    initial_pose.orientation.y = 0.0;
    initial_pose.orientation.z = 0.0;
    initial_pose.orientation.w = 0.0;
    //目标位置
    goal_pose.position.x = 0.0;
    goal_pose.position.y = 0.0;
    goal_pose.position.z = 0.0;

    goal_pose.orientation.x = 0.0;
    goal_pose.orientation.y = 0.0;
    goal_pose.orientation.z = 0.0;
    goal_pose.orientation.w = 0.0;
}

void speakCallback(const std_msgs::String::ConstPtr& msg)
{
    if(msg->data == "destination")
    {
        is_go = true;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "navigation_demo");
    ros::NodeHandle nh;

    sub_sound = nh.subscribe("kws_data", 10, speakCallback);

    initpose();

    sound_play::SoundClient sound_client;//语音客户端

    MoveBaseClient  mc_("move_base", true); //建立导航客户端
    move_base_msgs::MoveBaseGoal naviGoal; //导航目标点

    while(ros::ok())
    {
        if(is_go == true)
        {
            naviGoal.target_pose.header.frame_id = "map"; 
			naviGoal.target_pose.header.stamp = ros::Time::now();
			naviGoal.target_pose.pose = geometry_msgs::Pose(goal_pose);

            while(!mc_.waitForServer(ros::Duration(5.0)))
			{
				//等待服务初始化
				cout<<"Waiting for the server..."<<endl;
			}
			mc_.sendGoal(naviGoal);
			mc_.waitForResult(ros::Duration(40.0));

            sound_client.say("I wiil go to the destination!");

            if(mc_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			{
				cout<<"Yes! The robot has moved to the goal_pos1"<<endl;
                sound_client.say("I have reached the destination!");
			}
        }
       ros::spinOnce(); 
    }
}