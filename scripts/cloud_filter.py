#!/usr/bin/env python

from __future__ import print_function
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import os
from math import sqrt
# import ros_numpy
# import pcl


class cloud_filter():

    old_cloud = PointCloud2()

    def __init__(self):

        freq = 10

        self.pub_new_cloud = rospy.Publisher('/aai_cloud', PointCloud2, queue_size=1)
        self.sub_pc2 = rospy.Subscriber('/sensor/velodyne', PointCloud2, self.callback)

        node_sleep_rate = rospy.Rate(freq)

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
                new_pontos.append(p)


        return pc2.create_cloud(self.old_cloud.header, self.old_cloud.fields, new_pontos)



    def callback(self, data):
        self.old_cloud = data

if __name__ == '__main__':

    rospy.init_node('Cloud_Filter', anonymous=True)

    try:
        node_obj = cloud_filter()
    except rospy.ROSInterruptException:
        pass