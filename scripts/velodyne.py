#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import os
# import ros_numpy
# import pcl

def callback(data):
    # THIS_FOLDER = os.path.dirname(os.path.abspath(__file__))
    # my_file_path = os.path.join(THIS_FOLDER, '../text/velodynedata.txt')
    # write_handle = open(my_file_path, 'w')
    
    for p in pc2.read_points(data, field_names = ("x", "y", "z"), skip_nans=True):
        print " x : %f  y: %f  z: %f" %(p[0],p[1],p[2])
        # msg = str(p[0]) + '\t' + str(p[1]) + '\t' + str(p[2]) + '\n'
        # write_handle.write(msg)

    # write_handle.close()

if __name__ == '__main__':

    rospy.init_node('listener', anonymous=True)

    try:
        rospy.Subscriber('/sensor/velodyne', PointCloud2, callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

