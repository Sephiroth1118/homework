//copyright: Author Huziqi 2019.5.18//
#include<iostream>
#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<cmath>
#include<ros/ros.h>
#include<nav_msgs/MapMetaData.h>
#include<nav_msgs/OccupancyGrid.h>
#include<geometry_msgs/PoseWithCovarianceStamped.h>
#include<geometry_msgs/PoseStamped.h>
#include "homework/path.h"


using namespace std;
using namespace cv;


class point
{
public:
	point * father;
	int x,y,G,H,F,mark;
	point()
	{
		x = 0;
		y = 0;
		G = 0;
		H = 0;
		F = G + H;
		mark = 0;
		father = NULL;
	}
};



int height, width;
bool stop_flag=false;
int *map_data;
float resolution,map_origin_x,map_origin_y;
float start_pose_x=0,start_pose_y=0,NaviGoal_pose_x=0,NaviGoal_pose_y=0,start_ori_z,start_ori_w,NaviGoal_ori_z,NaviGoal_ori_w;
homework::path Path;
std::vector<geometry_msgs::Pose> path_pose;


void spawn(vector<vector<point*> > Map, point* center, point* end,vector<point*> &open,vector<point*> &close_list);
bool in_list(vector<point*> open,point* a);
point* add_close_list(vector<point*> &open,vector<point*> &close_list);
void MapCallback(const nav_msgs::MapMetaData::ConstPtr& msg);
void MapDataCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
void MapInfoCallback(const nav_msgs::MapMetaData::ConstPtr& msg);
void transform(vector<vector<point*> > Map,int* path_x,int* path_y);
void InitialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
void NaviGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);


int main(int argc, char** argv)
{
	ros::init(argc, argv, "A_Star");
	ros::NodeHandle myNode;
	ros::Rate loop_rate(2);  // 自循环频率
	cout<<"welcome to A_star planner!"<<endl;
	ros::Subscriber Map_info=myNode.subscribe("/map_metadata",10,MapCallback);
	ros::Subscriber Map_data=myNode.subscribe("/map",10,MapDataCallback);
	ros::Subscriber Init_pose=myNode.subscribe("/initialpose",10,InitialPoseCallback);
	ros::Subscriber NaviGoal=myNode.subscribe("/move_base_simple/goal",10,NaviGoalCallback);
	ros::Publisher Path_plan=myNode.advertise<homework::path>("/path_plan",10);
	int **map;
	for(int k=0;k<5;k++)
	{
		map=new int*[height];
		for(int i=0;i<height;i++)
		{
			map[i]=new int[width];
		}
		int count=0;
		for(int i=height-1;i>-1;i--)
		{
			for(int j=0;j<width;j++)
			{	
				if(map_data[count]==0)
				{
					map[i][j]=255;
				}
				else
				{
					map[i][j]=0;
				}
				count++;
			}
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	while(NaviGoal_pose_x==0)
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	///////////////////////
	int step=10;
	int **tem;
    tem=new int*[height];
    for(int i=0;i<height;i++)
    {
        tem[i]=new int[width];
    }
    for(int i=0;i<height;i++)
    {
        for(int j=0;j<width;j++)
        {
            tem[i][j]=map[i][j];
        }
    }
    for(int i=step;i<height-step;i++)
    {
        for(int j=step;j<width-step;j+=step)
        {
            if(int(map[i][j])==0)
            {
                for(int r=i-step+1;r<i+step-1;r++)
                {
                    for(int c=j-step+1;c<j+step-1;c++)
                    {
                       tem[r][c]=0;
                    }
                }
            }
        }
    }


	/////////////////////////

	cout<<height<<" "<<width<<endl;

	vector<point*> open;
	vector<point*> close_list;
	

	point* start=new point;
	//cin >> start->x >> start->y;
	start->x=-(start_pose_y-map_origin_y)/resolution+height;
	start->y=(start_pose_x-map_origin_x)/resolution;
	start->mark = 100;
	point* end=new point;
	//cin >> end->x >> end->y;
	end->x=-(NaviGoal_pose_y-map_origin_y)/resolution+height;
	end->y=(NaviGoal_pose_x-map_origin_x)/resolution;
	end->mark = 150;
	cout<<"start_pose: "<<start->x<<", "<<start->y<<endl;
	cout<<"NaviGoal: "<<end->x<<", "<<end->y<<endl;

	vector<vector<point*> > Map;
	Map.resize(height); // row
	for (int i = 0; i < height; ++i) {
		Map[i].resize(width); // column
	}
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			point* poi=new point;
			poi->mark = tem[i][j];
			poi->x = i;
			poi->y = j;
			poi->H = abs(i - end->x) + abs(j - end->y);
			Map[i][j] = poi;
		}
	}

	point *temp = start;
	vector<float> path_x;
	vector<float> path_y;
	close_list.push_back(start);
	while (!stop_flag)
	{
		spawn(Map, temp, end,open,close_list);
		if (stop_flag) break;
		temp = add_close_list(open,close_list);
		cout << "path.x: " << temp->x << " " << "path.y: " << temp->y << endl;
	}
	temp = end;
	while (!(temp->x == start->x&&temp->y==start->y))
	{
		Map[temp->father->x][temp->father->y]->mark = 6;
		temp = temp->father;
		path_x.push_back(temp->y*resolution+map_origin_x);
		path_y.push_back(map_origin_y-(temp->x-height)*resolution);
	}
	// for (int i = 0; i < height; i++)
	// {
	// 	for (int j = 0; j < width; j++)
	// 	{
	// 		cout << Map[i][j]->mark << " ";
	// 	}
	// 	cout << endl;
	// }
	cout<<"start_pose: "<<start_pose_x<<", "<<start_pose_y<<endl;
	cout<<"NaviGoal: "<<NaviGoal_pose_x<<", "<<NaviGoal_pose_y<<endl;
	geometry_msgs::Pose temp_pose;
	for(int i=path_x.size()-1;i>0;i--)
	{	
		cout<<"("<<path_x[i]<<", "<<path_y[i]<<")"<<endl;
		temp_pose.position.x=path_x[i];
		temp_pose.position.y=path_y[i];
		temp_pose.position.z=0;
		temp_pose.orientation.x=0;
		temp_pose.orientation.y=0;
		temp_pose.orientation.z=sin((atan((path_y[i-1]-path_y[i])/(path_x[i-1]-path_x[i])))/2);
		temp_pose.orientation.w=cos((atan((path_y[i-1]-path_y[i])/(path_x[i-1]-path_x[i])))/2);
		path_pose.push_back(temp_pose);
	}
	Path.path=path_pose;
	Path_plan.publish(Path);
	Mat img=imread("/home/kamerider/catkin_ws/src/homework/maps/my_map.pgm",1);
	Mat map_path=img.clone();
	cvtColor(map_path,map_path,cv::COLOR_RGB2GRAY);
	for(int i=0;i<height;i++)
	{
		for(int j=0;j<width;j++)
		{
			if(Map[i][j]->mark==6)
			{
				map_path.at<uchar>(i,j)=Map[i][j]->mark;
			}
		}
	}
	imshow("path",map_path);
	waitKey(0);
	return 0;
}

void MapCallback(const nav_msgs::MapMetaData::ConstPtr& msg)
{
	height=int(msg->height);
	width=int(msg->width);
	resolution=msg->resolution;
	map_origin_x=msg->origin.position.x;
	map_origin_y=msg->origin.position.y;
}

void MapDataCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	map_data=new int[msg->info.height*msg->info.width];
	for(int i=0;i<msg->info.height*msg->info.width;i++)
	{
		map_data[i]=msg->data[i];
	}
}

void InitialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	start_pose_x=msg->pose.pose.position.x;
	start_pose_y=msg->pose.pose.position.y;
	start_ori_z=msg->pose.pose.orientation.z;
	start_ori_w=msg->pose.pose.orientation.w;
}

void NaviGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	NaviGoal_pose_x=msg->pose.position.x;
	NaviGoal_pose_y=msg->pose.position.y;
	NaviGoal_ori_z=msg->pose.orientation.z;
	NaviGoal_ori_w=msg->pose.orientation.w;
}


void spawn(vector<vector<point*> > Map,point* center,point* end,vector<point*> &open,vector<point*> &close_list)
{
	for (int i = center->x - 1; i <= center->x + 1; i++)
	{
		for (int j = center->y - 1; j <= center->y + 1; j++)
		{
			if (i >= 0 && j >= 0 && i < height && j < width)
			{
				if (Map[i][j]->x == end->x&&Map[i][j]->y == end->y)
				{
					end->father = center;
					stop_flag = true;
					return;
				}
				if (i == center->x&&j == center->y)
				{
					continue;
				}
				if (Map[i][j]->mark == 255)
				{
					if (in_list(open, Map[i][j]))
					{
						if ((((i - center->x)*(j - center->y) != 0) && center->G + 14 < Map[i][j]->G) || (((i - center->x)*(j - center->y) == 0) && center->G + 10 < Map[i][j]->G))
						{
							Map[i][j]->father = center;
							Map[i][j]->G = ((i - center->x)*(j - center->y) != 0) ? center->G + 14 : center->G + 10;
							Map[i][j]->F = Map[i][j]->G + Map[i][j]->H;
						}
					}
					else
					{
						if (!in_list(close_list, Map[i][j]))
						{
							Map[i][j]->father = center;
							Map[i][j]->G = ((i - center->x)*(j - center->y) != 0) ? Map[i][j]->G + 14 : Map[i][j]->G + 10;
							open.push_back(Map[i][j]);
							Map[i][j]->F = Map[i][j]->G + Map[i][j]->H;
						}
					}
				}
			}
		}
	}
}

bool in_list(vector<point*> list,point* a)
{
	for (int i = 0; i < list.size(); i++)
	{
		if (a->x == list[i]->x&&a->y == list[i]->y)
		{
			return true;
		}
	}
	return false;
}

point* add_close_list(vector<point*> &open,vector<point*> &close_list)
{
	point* temp = open[0];
	for (int i = 1; i < open.size(); i++)
	{
		if (open[i]->H < temp->H)
		{
			temp = open[i];
		}
	}
	close_list.push_back(temp);
	vector<point*>::iterator itr = open.begin();
	while (itr != open.end())
	{
		if (*itr == temp)
		{
			itr = open.erase(itr);
		}
		else
		{
			++itr;
		}
	}
	return temp;
}