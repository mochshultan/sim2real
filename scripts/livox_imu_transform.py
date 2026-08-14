#!/usr/bin/env python

import math
import numpy as np
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3
import tf.transformations as tf
from tf.transformations import euler_from_quaternion

rospy.init_node('livox_imu_transform')
pub = rospy.Publisher('/imu/data', Imu, queue_size=10)

correction_q = tf.quaternion_from_euler(0, math.radians(-45), 0)
correction_q_inverse = tf.quaternion_from_euler(0, math.radians(45), 0)
print(correction_q)
R_corr_inverse = tf.quaternion_matrix(correction_q_inverse)[:3, :3]  # 3x3 rotation matrix

def rotate_vector(v, R):
    vec = np.array([v.x, v.y, v.z])
    vec_rot = R.dot(vec)
    return Vector3(*vec_rot)

def callback(msg):
    q_orig = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
    # r, p, y = euler_from_quaternion(q_orig)
    # print(f"roll={math.degrees(r):.2f}, pitch={math.degrees(p):.2f}, yaw={math.degrees(y):.2f}")

    q_corr = tf.quaternion_multiply(q_orig, correction_q)
    # r, p, y = euler_from_quaternion(q_corr)
    # print(f"roll={math.degrees(r):.2f}, pitch={math.degrees(p):.2f}, yaw={math.degrees(y):.2f}")
    msg.orientation = Quaternion(*q_corr)

    msg.angular_velocity = rotate_vector(msg.angular_velocity, R_corr_inverse)
    msg.linear_acceleration = rotate_vector(msg.linear_acceleration, R_corr_inverse)

    pub.publish(msg)

sub = rospy.Subscriber('/livox/imu_filtered', Imu, callback)
rospy.spin()

