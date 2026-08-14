#!/usr/bin/env python

import rospy
import tf
import tf2_ros
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class StaticTransformPublisher:
    def __init__(self):
        rospy.init_node('livox_odom_transform')

        # Subscribe to the IMU topic
        self.start_time = rospy.Time.now()
        self.imu_sub = rospy.Subscriber('/livox/imu_filtered', Imu, self.imu_callback)

        # TF static broadcaster
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()

        # Transform parameters
        self.odom_frame = "odom"
        self.camera_init_frame = "camera_init"
        self.transform_broadcasted = False

    def imu_callback(self, msg):
        # after 3 seconds of receiving the first IMU message, publish the static transform
        if rospy.Time.now() - self.start_time > rospy.Duration(3) and not self.transform_broadcasted:
            # Get orientation from IMU message
            orientation_q = msg.orientation
            # orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]

            # # Convert quaternion to Euler angles
            # (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
            # print(roll, pitch, yaw)

            # # Assuming the IMU orientation gives the orientation of the LiDAR relative to the odom frame
            # # Invert the orientation to get the transform from odom to camera_init
            # roll_inv, pitch_inv, yaw_inv = -roll, -pitch, -yaw

            # # Convert inverted Euler angles back to quaternion
            # quaternion_inv = quaternion_from_euler(roll_inv, pitch_inv, yaw_inv)

            # Create and populate the TransformStamped message
            static_transform_stamped = tf2_ros.TransformStamped()
            static_transform_stamped.header.stamp = rospy.Time.now()
            static_transform_stamped.header.frame_id = self.odom_frame
            static_transform_stamped.child_frame_id = self.camera_init_frame
            static_transform_stamped.transform.translation.x = 0
            static_transform_stamped.transform.translation.y = 0
            static_transform_stamped.transform.translation.z = 0
            # static_transform_stamped.transform.rotation.x = quaternion_inv[0]
            # static_transform_stamped.transform.rotation.y = quaternion_inv[1]
            # static_transform_stamped.transform.rotation.z = quaternion_inv[2]
            # static_transform_stamped.transform.rotation.w = quaternion_inv[3]
            static_transform_stamped.transform.rotation.x = orientation_q.x
            static_transform_stamped.transform.rotation.y = orientation_q.y
            static_transform_stamped.transform.rotation.z = orientation_q.z
            static_transform_stamped.transform.rotation.w = orientation_q.w

            # Broadcast the static transform
            self.static_broadcaster.sendTransform(static_transform_stamped)

            self.transform_broadcasted = True
            rospy.loginfo("Published static transform from /odom to /camera_init")

if __name__ == '__main__':
    try:
        StaticTransformPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
