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

//定义全局变量
int total_number = 0;   //总共的点的数量
const int step = 4;   //贝塞尔曲线阶数
int start = 0;          //曲线开始点的位置
int ending = start + step;//曲线结束点的位置
vector<geometry_msgs::Pose> path_pose; //地图数据坐标
geometry_msgs::Pose final_pose; //地图上终点的坐标
geometry_msgs::Pose cur_pose;   //机器人当前坐标

//预定点的位置及角度
double x_des = 0;
double y_des = 0;
double theta_des = 0;

//定义发布器和订阅器
ros::Publisher pub_data;    //发布生成的贝塞尔曲线中的数据
ros::Subscriber sub_data;   //订阅地图中的原始数据
ros::Subscriber sub_odm_data;    //订阅里程计消息

bool flag = false;              //只有收到地图消息后才进入ros循环

//订阅tf变换
tf::TransformListener listener;

//初始化坐标点位置
void init()
{
    final_pose.position.x = 0.0;
    final_pose.position.y = 0.0;
    final_pose.position.z = 0.0;
    final_pose.orientation.x = 0.0;
    final_pose.orientation.y = 0.0;
    final_pose.orientation.z = 0.0;
    final_pose.orientation.w = 0.0;

    cur_pose.position.x = 0.0;
    cur_pose.position.y = 0.0;
    cur_pose.position.z = 0.0;
    cur_pose.orientation.x = 0.0;
    cur_pose.orientation.y = 0.0;
    cur_pose.orientation.z = 0.0;
    cur_pose.orientation.w = 0.0;
}

//坐标变换函数
void transformPoint(const tf::TransformListener& listener, geometry_msgs::Pose a, geometry_msgs::Pose b)
{
    geometry_msgs::PointStamped map_point;
    //父坐标系
    map_point.header.frame_id = "map";
    //时间戳
    map_point.header.stamp = ros::Time();

    //地图中的坐标点
    map_point.point.x = a.position.x;
    map_point.point.y = a.position.y;
    map_point.point.z = a.position.z;

    //里程计坐标系下的点
    geometry_msgs::PointStamped base_point;

    try
    {
        listener.transformPoint("odom", map_point, base_point);
        printf("map: (%.2f, %.2f. %.2f) -----> odom: (%.2f, %.2f, %.2f) at time %.2f",
        map_point.point.x, map_point.point.y, map_point.point.z,
        base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());  
    }
    catch(tf::TransformException& ex)
    {
        ROS_ERROR("Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());
    }
    
    b.position.x = base_point.point.x;
    b.position.y = base_point.point.y;
    b.position.z = base_point.point.z;
}

//存储地图数据
//@TODO map->odom坐标变换
void mapCallback(const homework::path::ConstPtr& msg)
{
    for(int i = 0; i <msg->path.size(); i += 10)
    {   
        geometry_msgs::Pose tmp;
        transformPoint(listener, msg->path[i], tmp);
        path_pose.push_back(tmp);
    }

    geometry_msgs::Pose tmp;
    transformPoint(listener, msg->path[msg->path.size()-1], tmp);
    path_pose.push_back(tmp);

    flag = true;
}

//更新机器人当前位置坐标
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    //@TODO
    cur_pose.position.x = msg->pose.pose.position.x;
    cur_pose.position.y = msg->pose.pose.position.y;
    cur_pose.position.z = msg->pose.pose.position.z;

    cur_pose.orientation.x = msg->pose.pose.orientation.x;
    cur_pose.orientation.y = msg->pose.pose.orientation.y;
    cur_pose.orientation.z = msg->pose.pose.orientation.z;
    cur_pose.orientation.w = msg->pose.pose.orientation.w;
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

//先判断所在位置处在哪段曲线上
void CurveJudge()
{
    //@TODO
    int number = 0;
    vector<double> Distance;

    for(int i = 0; i < path_pose.size(); i++)
    {
        double d = EucDistance(cur_pose,path_pose[i]);
        Distance.push_back(d);
    }

    number = SmlNumber(Distance) + 1;
    int mul = number/(step+1);
    int rem = number%(step+1);
    
    start = mul * (step + 1);
    if(start + step + 1 >path_pose.size())
    {
        ending = path_pose.size() - 1;
    }
    else
    {
        ending = start + step;
    }
}

//将四元组转化为笛卡尔坐标系下的角度
double Trans(geometry_msgs::Pose a)
{
    double x = a.orientation.x;
    double y = a.orientation.y;
    double z = a.orientation.z;
    double w = a.orientation.w;

    double theta = atan2(2*(w*z+x*y),1-2*(y*y+z*z));
    return theta;
}

//贝塞尔曲线拟合
double BasierCurve()
{
    //@TODO
    if(ending - start == 0)                 //只有一个点，即为目标点
    {
        x_des = final_pose.position.x;
        y_des = final_pose.position.y;
        theta_des = Trans(final_pose);
    }
    else if(ending - start == 1)            //用一阶贝塞尔曲线
    {
        double x = 0;
        double y = 0;
        double s_d = MAXNUM;
        double s_t = 0;                     //距离最近的点的t值
        //分别从0-1等间隔取10个数作为t
        for(int t = 0; t <= 1; t+=0.1)
        {
            x = (1 - t) * path_pose[start].position.x + t * path_pose[ending].position.x;
            y = (1 - t) * path_pose[start].position.y + t * path_pose[ending].position.y;
            double distance = fabs(sqrt(pow(cur_pose.position.x,2)+pow(cur_pose.position.y,2)) - sqrt(pow(x,2)+pow(y,2)));
            if(distance < s_d)
            {
                x_des = x;
                y_des = y;
                s_t = t;
                s_d = distance;
            }
        }

        theta_des = atan2(path_pose[ending].position.y - path_pose[start].position.y, path_pose[ending].position.x - path_pose[start].position.x);
    }

    else if(ending - start == 2)            //用二阶贝塞尔曲线
    {
        double x = 0;
        double y = 0;
        double s_d = MAXNUM;
        double s_t = 0;
        for(int t = 0; t <= 1; t += 0.1)
        {
            x = (1 - t)*(1 - t)*path_pose[start].position.x + 2*(1-t)*t*path_pose[start+1].position.x + t*t*path_pose[ending].position.x;
            y = (1 - t)*(1 - t)*path_pose[start].position.y + 2*(1-t)*t*path_pose[start+1].position.y + t*t*path_pose[ending].position.y;
            double distance = fabs(sqrt(pow(cur_pose.position.x,2)+pow(cur_pose.position.y,2)) - sqrt(pow(x,2)+pow(y,2)));
            if(distance < s_d)
            {
                x_des = x;
                y_des = y;
                s_t = t;
                s_d = distance;
            }
        }
        if(s_t == 1)
        {
            theta_des = atan2(path_pose[s_t].position.y - path_pose[s_t-0.1].position.y, path_pose[s_t].position.x - path_pose[s_t - 0.1].position.x);
        }
        else
        {
            theta_des = atan2(path_pose[s_t+0.1].position.y - path_pose[s_t].position.y, path_pose[s_t+0.1].position.x - path_pose[s_t].position.x);
                        
        }
        
    }
    else if( ending - start == 3)            //用三阶贝塞尔曲线
    {
        double x = 0;
        double y = 0;
        double s_d = MAXNUM;
        double s_t = 0;
        for(int t = 0; t <= 1; t += 0.1)
        {
            x = pow(1-t,3)*path_pose[start].position.x + 3*(1-t)*(1-t)*t*path_pose[start+1].position.x + 3*(1-t)*t*t*path_pose[start+2].position.x +t*t*t*path_pose[ending].position.x;
            y = pow(1-t,3)*path_pose[start].position.y + 3*(1-t)*(1-t)*t*path_pose[start+1].position.y + 3*(1-t)*t*t*path_pose[start+2].position.y +t*t*t*path_pose[ending].position.y;
            double distance = fabs(sqrt(pow(cur_pose.position.x,2)+pow(cur_pose.position.y,2)) - sqrt(pow(x,2)+pow(y,2)));
            if(distance < s_d)
            {
                x_des = x;
                y_des = y;
                s_t = t;
                s_d = distance;
            }
        }
        if(s_t == 1)
        {
            theta_des = atan2(path_pose[s_t].position.y - path_pose[s_t-0.1].position.y, path_pose[s_t].position.x - path_pose[s_t - 0.1].position.x);
        }
        else
        {
            theta_des = atan2(path_pose[s_t+0.1].position.y - path_pose[s_t].position.y, path_pose[s_t+0.1].position.x - path_pose[s_t].position.x);                       
        }
    }

    else if(ending - start == 4)            //使用四阶贝塞尔曲线
    {
        double x = 0;
        double y = 0;
        double s_d = MAXNUM;
        double s_t = 0;
        for(int t =0; t<= 1;t+=0.1)
        {
            x = pow(1-t,4)*path_pose[start].position.x + 4*pow(1-t,3)*t*path_pose[start+1].position.x + 6*(1-t)*(1-t)*t*t*path_pose[start+2].position.x + 4*(1-t)*pow(t,3)*path_pose[start+3].position.x + pow(t,4)*path_pose[ending].position.x;
            y = pow(1-t,4)*path_pose[start].position.y + 4*pow(1-t,3)*t*path_pose[start+1].position.y + 6*(1-t)*(1-t)*t*t*path_pose[start+2].position.y + 4*(1-t)*pow(t,3)*path_pose[start+3].position.y + pow(t,4)*path_pose[ending].position.y;
            double distance = fabs(sqrt(pow(cur_pose.position.x,2)+pow(cur_pose.position.y,2)) - sqrt(pow(x,2)+pow(y,2)));
            if(distance < s_d)
            {
                x_des = x;
                y_des = y;
                s_t = t;
                s_d = distance;
            }
        }
        if(s_t == 1)
        {
            theta_des = atan2(path_pose[s_t].position.y - path_pose[s_t-0.1].position.y, path_pose[s_t].position.x - path_pose[s_t - 0.1].position.x);
        }
        else
        {
            theta_des = atan2(path_pose[s_t+0.1].position.y - path_pose[s_t].position.y, path_pose[s_t+0.1].position.x - path_pose[s_t].position.x);                       
        }
    }  
}

//主函数
int main(int argc, char **argv)
{
    ros::init(argc, argv, "navigation_demo");
    ros::NodeHandle nh;

    ros::Rate loop_rate(5);  // 自循环频率

    //声明发布器和订阅器
    pub_data = nh.advertise<homework::location>("/besier_to_controller",1);
    sub_data = nh.subscribe("/raw_map_data", 1, mapCallback);
    sub_odm_data = nh.subscribe("/sensor_fusions/odom", 1 , odomCallback);

    init();

    ROS_INFO("Start Besier!");

    while(ros::ok() && flag == true)
    {
        CurveJudge();
        BasierCurve();
        homework::location location_data;
        location_data.x = x_des;
        location_data.y = y_des;
        location_data.theta = theta_des;

        pub_data.publish(location_data);

        ros::spinOnce();
    }
    return 0;
}