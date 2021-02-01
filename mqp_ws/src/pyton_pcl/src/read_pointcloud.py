#!/usr/bin/env python2
# -*- coding: UTF-8 -*-
import rospy
import numpy as np
import math as math
import pcl 
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
import pptk

class read_pointcloud:

    def __init__(self):
        """
        Class constructor
        """
        rospy.init_node("pcl", anonymous=True)
        rospy.Subscriber("/camera/depth/points", PointCloud2, self.displaypc)
        rospy.sleep(2) 

    def displaypc(self,msg):
        points_list = []

        for data in pc2.read_points(msg, skip_nans=True):
            points_list.append([data[0], data[1], data[2], data[3]])

        pcl_data = pcl.PointCloud_PointXYZRGB()
        pcl_data.from_list(points_list)

        print pcl_data

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    read_pointcloud().run()

