#!/usr/bin/env python

from __future__ import print_function
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import os
from math import sqrt

class cloud_filter():

    old_cloud = PointCloud2()

    def __init__(self):

        freq = 10

        self.pub_new_cloud = rospy.Publisher('/aai_disc_cloud', PointCloud2, queue_size=1)
        self.sub_pc2 = rospy.Subscriber('/aai_cloud', PointCloud2, self.callback)

        node_sleep_rate = rospy.Rate(freq)

        while not rospy.is_shutdown():

            NewCloud = self.FilterC()

            self.pub_new_cloud.publish(NewCloud)
            node_sleep_rate.sleep()

    def FilterC(self):
        pontos = pc2.read_points_list(self.old_cloud, field_names = ("x", "y", "z"), skip_nans=True)
        new_pontos = list()
        if len(pontos) > 0:
            p_ant = pontos[0]

        for p in pontos:
            d_p = sqrt(p[0]**2 + p[1]**2 + p[2]**2)
            d_ant = sqrt(p_ant[0]**2 + p_ant[1]**2 + p_ant[2]**2)
            if abs(d_ant - d_p) > 0.5: #0.2*d_p:
                if  p[1] <1 and p[0] >= 0 and p[0] <= 15:
                    flag = True
                    for p2 in new_pontos:
                        if sqrt((p[0]-p2[0])**2 + (p[1]-p2[1])**2 + (p[2]-p2[2])**2) < 0.3:
                            flag = False
                            break
                    if flag:
                        new_pontos.append(p)
                        # print('p =', p[0], p[1], p[2])
                        # print(d_p)
                if  p_ant[1] <1 and p_ant[0] >= 0 and p_ant[0] <= 15:
                    flag = True
                    for p2 in new_pontos:
                        if sqrt((p_ant[0]-p2[0])**2 + (p_ant[1]-p2[1])**2 + (p_ant[2]-p2[2])**2) < 0.3:
                            flag = False
                            break
                    if flag:
                        new_pontos.append(p_ant)
                        # print('p =', p_ant[0], p_ant[1], p_ant[2])
                        # print(d_ant)
            p_ant = p


        
        print('-----')
        d_ps = [sqrt(p[0]**2 + p[1]**2 + p[2]**2) for p in new_pontos]
        disc = list()
        if len(d_ps) > 1:
            i = d_ps.index(min(d_ps))
            d_ps.remove(min(d_ps))
            j = d_ps.index(min(d_ps))
            if j >= i:
                j += 1
            disc = [new_pontos[i], new_pontos[j]]

        print(disc)

        # if len(new_pontos) > 0:
        #     nearp = new_pontos[0]
        #     d_near = sqrt(nearp[0]**2 + nearp[1]**2 + nearp[2]**2)
        # for p in new_pontos:
        #     d_p = sqrt(p[0]**2 + p[1]**2 + p[2]**2)
        #     if d_p < d_near:
        #         nearp = p
        #         d_near = d_p
        

        return pc2.create_cloud(self.old_cloud.header, self.old_cloud.fields, disc)



    def callback(self, data):
        self.old_cloud = data

if __name__ == '__main__':

    rospy.init_node('Cloud_Filter', anonymous=True)

    try:
        node_obj = cloud_filter()
    except rospy.ROSInterruptException:
        pass