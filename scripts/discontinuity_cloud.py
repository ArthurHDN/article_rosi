#!/usr/bin/env python

from __future__ import print_function
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import os
from math import sqrt, atan2, pi, cos, sin
from collections import namedtuple

class cloud_filter():

    old_cloud = PointCloud2()

    def __init__(self):

        freq = 10

        self.pub_disc_cloud = rospy.Publisher('/aai_disc_cloud', PointCloud2, queue_size=1)
        self.pub_true_cloud = rospy.Publisher('/aai_true_cloud', PointCloud2, queue_size=1)
        self.sub_pc2 = rospy.Subscriber('/aai_cloud', PointCloud2, self.callback)

        node_sleep_rate = rospy.Rate(freq)

        while not rospy.is_shutdown():

            (TrueCloud, DiscCloud) = self.FilterC()

            self.pub_disc_cloud.publish(DiscCloud)
            self.pub_true_cloud.publish(TrueCloud)
            node_sleep_rate.sleep()

    def FilterC(self):
        pontos = pc2.read_points_list(self.old_cloud, field_names = ("x", "y", "z"), skip_nans=True)
        new_pontos = list()

        d_max = 10
        d_trig = 1.5

        for k in range(270):
            encontrou = False
            for p in pontos:
                if ( (180*atan2(p[1],p[0])/pi) - (k-135) )**2 < 0.01:
                    Point = namedtuple("Point", ("x", "y", "z"))
                    l = (p[0], p[1], 0)
                    new_p = Point._make(l)
                    new_pontos.append(new_p)
                    encontrou = True
                    break
            if not encontrou:
                Point = namedtuple("Point", ("x", "y", "z"))
                l = (d_max*cos(pi*(k-135)/180), d_max*sin(pi*(k-135)/180), 0)
                new_p = Point._make(l)
                new_pontos.append(new_p)

        disc = list()

        # p = new_pontos[0]
        # if sqrt(p[0]**2 + p[1]**2) < d_max:
        #     disc.append(p)
        d_max = 9

        for k1 in range(1, 269):
            k2 = k1 + 1
            p1 = new_pontos[k1]
            p2 = new_pontos[k2]
            d_p1 = sqrt(p1[0]**2 + p1[1]**2)
            d_p2 = sqrt(p2[0]**2 + p2[1]**2)
            if d_p1 == d_max and d_p2 < d_max:
                disc.append(p2)
            elif abs(d_p1 - d_p2) >= d_trig:
                if d_p1 < d_p2:
                    disc.append(p1)
                else:
                    disc.append(p2)

        # p = new_pontos[269]
        # if sqrt(p[0]**2 + p[1]**2) < d_max:
        #     disc.append(p)

        print(len(disc))
                


        return (pc2.create_cloud_xyz32(self.old_cloud.header, new_pontos), pc2.create_cloud_xyz32(self.old_cloud.header, disc) )



    def callback(self, data):
        self.old_cloud = data

if __name__ == '__main__':

    rospy.init_node('Cloud_Filter', anonymous=True)

    try:
        node_obj = cloud_filter()
    except rospy.ROSInterruptException:
        pass