#!/usr/bin/env python

from __future__ import print_function
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import os
from math import sqrt, cos, sin, pi
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Pose


class cloud_filter():

    old_cloud = PointCloud2()

    pos_x = 0
    pos_y = 0
    angle = 0

    def __init__(self):
        
        self.pub_new_cloud = rospy.Publisher('/aai_cloud', PointCloud2, queue_size=1)
        self.sub_pc2 = rospy.Subscriber('/sensor/velodyne', PointCloud2, self.callback)
        self.sub_pose = rospy.Subscriber('/aai_rosi_pose', Pose, self.callback_pose)

        node_sleep_rate = rospy.Rate(10)

        while not rospy.is_shutdown():

            NewCloud = self.FilterC()

            self.pub_new_cloud.publish(NewCloud)
            node_sleep_rate.sleep()

    def FilterC(self):
        pontos = pc2.read_points_list(self.old_cloud, field_names = ("x", "y", "z"), skip_nans=True)
        new_pontos = list()
        for p in pontos:
            # print('p = (', p[0], p[1], p[2],')')
            if p[2] >= -0.1 and p[2] <= 0.1 and sqrt(p[0]**2 + p[1]**2 + p[2]**2) > 0.5:
                rp = (p[0], p[1])
                wp = self.world_frame(rp)
                if not self.inTC(wp): 
                    new_pontos.append(p)
        return pc2.create_cloud(self.old_cloud.header, self.old_cloud.fields, new_pontos)

    def world_frame(self,rp):
        rf = np.array([ [rp[0]], [rp[1]], [1] ]); H = np.array([ [cos(self.angle), -sin(self.angle), self.pos_x], [sin(self.angle), cos(self.angle), self.pos_y], [0,0,1] ]); 	wf = H.dot(rf); wp = (wf[0], wf[1])
        return wp
        
    def inTC(self, p):
		x = p[0]
		y = p[1]
		if self.pos_y >= 0:
			if y <= 2: # and y >= -2:# and x <= -1.7 and x >= -50 :
				return True
			else:
				return False
		elif self.pos_y < 0:
			if y >= -2: # and y >= -2:# and x <= -1.7 and x >= -50 :
				return True
			else:
				return False



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