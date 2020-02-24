#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import os
# import ros_numpy
# import pcl

def callback(data):
    THIS_FOLDER = os.path.dirname(os.path.abspath(__file__))
    my_file_path = os.path.join(THIS_FOLDER, '../text/velodynedata.txt')
    write_handle = open(my_file_path, 'w')
    for p in pc2.read_points(data, field_names = ("x", "y", "z"), skip_nans=True):
        print " x : %f  y: %f  z: %f" %(p[0],p[1],p[2])
        msg = str(p[0]) + '\t' + str(p[1]) + '\t' + str(p[2]) + '\n'
        write_handle.write(msg)

    write_handle.close()
    # pc = ros_numpy.numpify(pointcloud)
    # height = pc.shape[0]
    # width = pc.shape[1]
    # np_points = np.zeros((height * width, 3), dtype=np.float32)
    # np_points[:, 0] = np.resize(pc['x'], height * width)
    # np_points[:, 1] = np.resize(pc['y'], height * width)
    # np_points[:, 2] = np.resize(pc['z'], height * width)


    # pc = ros_numpy.numpify(data)
    # points=np.zeros((pc.shape[0],3))
    # points[:,0]=pc['x']
    # points[:,1]=pc['y']
    # points[:,2]=pc['z']
    # p = pcl.PointCloud(np.array(points, dtype=np.float32))


if __name__ == '__main__':

    rospy.init_node('listener', anonymous=True)

    try:
        rospy.Subscriber('/sensor/velodyne', PointCloud2, callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

