#!/usr/bin/env python

from __future__ import print_function
import rospy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Pose
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import os
from math import sqrt, atan2, pi, cos, sin
from collections import namedtuple
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class cloud_filter():

    old_cloud = PointCloud2()

    pos_x = 0
    pos_y = 0
    angle = 0

    def __init__(self):

        self.pub_new_cloud = rospy.Publisher('/aai_cloud_world_frame', PointCloud2, queue_size=1)
        self.sub_pc2 = rospy.Subscriber('/sensor/velodyne', PointCloud2, self.callback)
        self.sub_pose = rospy.Subscriber('/aai_rosi_pose', Pose, self.callback_pose)

        node_sleep_rate = rospy.Rate(10)

        while not rospy.is_shutdown():

            NewCloud = self.FilterC()

            self.pub_new_cloud.publish(NewCloud)
            print('pub')
            node_sleep_rate.sleep()

    def world_frame(self,rp):
        rf = np.array([ [rp[0]], [rp[1]], [1] ])
        H = np.array([ [cos(self.angle), -sin(self.angle), self.pos_x], [sin(self.angle), cos(self.angle), self.pos_y], [0,0,1] ])
        wf = H.dot(rf)
        wp = (wf[0], wf[1])
        return wp

    def FilterC(self):
        pontos = pc2.read_points_list(self.old_cloud, field_names = ("x", "y", "z"), skip_nans=True)
        
        new_pontos = list()

        for p in pontos:
            rp = (p[0], p[1])
            wp = self.world_frame(rp)
            Point = namedtuple("Point", ("x", "y", "z"))
            l = (wp[0], wp[1], p[2])
            new_p = Point._make(l)
            new_pontos.append(new_p)

        return pc2.create_cloud_xyz32(self.old_cloud.header, new_pontos)

    def callback(self, data):
        self.old_cloud = data

    def callback_pose(self, data):

		q_x = data.orientation.x
		q_y = data.orientation.y
		q_z = data.orientation.z
		q_w = data.orientation.w
		# Orientacao de quaternios para angulos de Euler
		euler_angles = euler_from_quaternion([q_x, q_y, q_z, q_w])

		self.pos_x  = data.position.x
		self.pos_y = data.position.y
		self.angle = euler_angles[2] # Apenas o angulo de Euler no eixo z nos interessa

if __name__ == '__main__':

    rospy.init_node('Cloud_Filter', anonymous=True)

    try:
        node_obj = cloud_filter()
    except rospy.ROSInterruptException:
        pass