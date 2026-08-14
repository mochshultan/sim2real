#!/usr/bin/env python

import rospy
from livox_ros_driver2.msg import CustomMsg
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pcl2
import std_msgs.msg
import numpy as np


def livox_callback(msg):
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "livox_back"

    points = []

    for point in msg.points:
        x = point.x
        y = point.y
        z = point.z
        intensity = point.reflectivity
        points.append([x, y, z, intensity])

    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    ]

    pc2_msg = pcl2.create_cloud(header, fields, points)
    pointcloud_pub.publish(pc2_msg)


if __name__ == '__main__':
    rospy.init_node('livox_to_pointcloud2')
    pointcloud_pub = rospy.Publisher('/cloud_registered_body_back', PointCloud2, queue_size=10)
    rospy.Subscriber('/livox/lidar_192_168_1_124', CustomMsg, livox_callback)
    print("start livox to pointcloud2 node")
    rospy.spin()

