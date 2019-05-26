#! /usr/bin/env python
# -*- encoding: UTF-8 -*-
#Author:Yu Shizhuo
import rospy
import os
import sys
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
import math
import numpy as np
import cv2
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PointStamped

class map_update:

    def __init__(self):
        rospy.init_node("map_update")
        self.map_update_pub=rospy.Publisher("/map",OccupancyGrid,queue_size=15)
        self.order_pub=rospy.Publisher("/order_to_stop",String,queue_size=10)
        #地图相关
        self.stop_order="Stop now."
        self.map=[]
        self.map_raw=[]
        #地图原点
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.height = 0
        self.width = 0
        self.resolution = 0.0
        #判断是否接收到地图
        self.is_map_received=False
        self.map_sub=rospy.Subscriber("/map",OccupancyGrid,self.mapCallback)
        #激光相关
        self.angle_min=0.0
        self.angle_max=0.0
        self.angle_increment=0.0
        self.scan_time=0.0
        self.range_min=-math.pi/12
        self.range_max=math.pi/12
        #判断是否检测到障碍物
        self.isob=False
        self.dis=0.5
        self.laser_sub=rospy.Subscriber("/scan",LaserScan,self.laserCallback)
        #amcl相关       
        self.x = 0.0
        self.y = 0.0
        self.wy= 0.0
        self.wx= 0.0
        self.xm= 0.0
        self.ym= 0.0
        self.isamcl=False
        self.amcl_sub=rospy.Subscriber("/amcl_pose",PoseWithCovarianceStamped,self.amclCallback)
        #判断amcl是否完成定位
        rospy.Rate(1)
        rospy.spin()

    def World2map(self,xw,yw):
        xm=float(xw-self.origin_x)/float(self.resolution)
        ym=float(yw-self.origin_y)/float(self.resolution)
        return xm,ym
    
    #更新地图函数:

    def update_map(self):
        if self.is_map_received and self.isob and self.isamcl:
            self.order_pub.publish(self.stop_order)
            print("update map")
            self.wx=self.x+self.dis*math.cos(self.theta)
            self.wy=self.y+self.dis*math.sin(self.theta)
         #   print(self.wx)
         #   print(self.wy)
            self.xm,self.ym=self.World2map(self.wx,self.wy)
            self.xm=int(self.xm)
            self.ym=int(self.ym)
         #   print(self.xm)
         #   print(self.ym)
            U_map=self.map_raw.reshape(self.height,self.width)
            for i in range(self.ym-3,self.ym+3):
                for j in range(self.xm-3,self.xm+3):
                    U_map[i][j]=100
            Up_map=np.array(U_map,dtype=np.int8)
            self.map.data=list(Up_map.flatten())
            self.map_update_pub.publish(self.map)

    #回调函数:
   
    def mapCallback(self, msg):
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y
        self.resolution = msg.info.resolution
        self.width = msg.info.width
        self.height = msg.info.height
        self.map=msg
        self.map_raw=np.array(msg.data,dtype=np.int8)
        self.is_map_received=True
        print("Map received")
        
    def laserCallback(self,msg):
        self.angle_min=msg.angle_min
        self.angle_max=msg.angle_max
        self.angle_increment=msg.angle_increment
        min_distance=10
        range_index_start = int((self.range_min - self.angle_min)/self.angle_increment)
        range_index_end = int ((self.range_max - self.angle_min)/self.angle_increment)
        #寻找范围内最小的距离
        for i in range(range_index_start,range_index_end):
            if msg.ranges[i]<min_distance and msg.ranges[i]>0.0:
                min_distance = msg.ranges[i]
            if min_distance <= 0.5:
                self.isob=True
            else:
                self.isob =False
        print("Laser is ready")
        self.update_map()

    def amclCallback(self,msg):
        print(msg.header.frame_id)
        print(msg.pose.pose.position.x,msg.pose.pose.position.y)
        self.x=msg.pose.pose.position.x
        self.y=msg.pose.pose.position.y
        self.theta=math.acos(1-2*msg.pose.pose.orientation.z*msg.pose.pose.orientation.z)
        self.isamcl=True
        print("amcl is ready")

if __name__ == '__main__':
    
    map_update()