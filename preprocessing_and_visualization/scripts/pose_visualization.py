#!/usr/bin/env python3

import numpy as np
import yaml

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix

import pygeodesy
from scipy.spatial.transform import Rotation

from preprocessing_and_visualization.common_utils import \
    invert_transformation_matrix, transformation_matrix_to_transform_stamped

def to_transformation_matrix(rotation, translation):
    transformation = np.eye(4)
    transformation[0:3, 0:3] = rotation
    transformation[0:3, 3] = translation

    return transformation

class PoseHandler:
    def __init__(self):
        # Process arguments
        path_measured_extrinsics = rospy.get_param('~path_measured_extrinsics', default='')

        # Handle the measured extrinsics file if given
        self.messages_static_transform_stamped = []
        if path_measured_extrinsics != '':
            # Open the measured extrinsics file
            with open(path_measured_extrinsics, 'r') as file_object:
                try:
                    data_measured = yaml.safe_load(file_object)
                except yaml.YAMLError as exception:
                    print(exception)
                    exit()

            # Compile the static transforms between the base_link and all sensors
            # NOTE: all static transforms broadcasted within a single node must be broadcasted at once, so these will be
            # broadcast alongside the transform from ECEF to ENU (in the callback)
            for key in data_measured.keys():
                self.messages_static_transform_stamped.append(transformation_matrix_to_transform_stamped(
                    np.asarray(data_measured[key]), 'base_link', key[2:]))

        # Create the global static and dynamic tf broadcasters
        self.broadcaster_static = tf2_ros.StaticTransformBroadcaster()
        self.broadcaster_dynamic = tf2_ros.TransformBroadcaster()

        # Create the NavSatFix publisher
        self.publisher_navsatfix = rospy.Publisher('/applanix/navsatfix_baselink', NavSatFix, queue_size=10)

        # Set the PoseStamped callback
        self.transformation_ecef_to_enu = None
        rospy.Subscriber('/applanix/pose_base_link', PoseStamped, self.callback_pose_stamped)

    def callback_pose_stamped(self, message_pose_stamped):
        # Unpack the ECEF position of the base_link frame
        position_baselink_in_ecef = np.asarray([message_pose_stamped.pose.position.x,
                                                message_pose_stamped.pose.position.y,
                                                message_pose_stamped.pose.position.z])

        # Compute the static transform from ECEF to the ENU frame centered at the first base_link ECEF position and
        # broadcast all static transforms, if this has not yet been done
        if self.transformation_ecef_to_enu is None:
            # Get the rotation from the ENU frame to ECEF
            info_frame_baselink = pygeodesy.ecef.EcefKarney(pygeodesy.datums.Datums.WGS84).reverse(
                position_baselink_in_ecef[0], position_baselink_in_ecef[1], position_baselink_in_ecef[2], M=True)
            rotation_enu_to_ecef = np.array([info_frame_baselink.M.row(0),
                                            info_frame_baselink.M.row(1),
                                            info_frame_baselink.M.row(2)])

            # Form the transformation matrix from ENU to ECEF and invert it
            transformation_enu_to_ecef = to_transformation_matrix(rotation_enu_to_ecef, position_baselink_in_ecef)
            self.transformation_ecef_to_enu = invert_transformation_matrix(transformation_enu_to_ecef)

            # Broadcast all static transforms
            self.messages_static_transform_stamped.append(transformation_matrix_to_transform_stamped(
                transformation_enu_to_ecef, 'earth', 'map'))
            self.broadcaster_static.sendTransform(self.messages_static_transform_stamped)

        # Broadcast the dynamic transform from ENU to the base_link frame
        quaternion_baselink_to_ecef = np.asarray([message_pose_stamped.pose.orientation.x,
                                                message_pose_stamped.pose.orientation.y,
                                                message_pose_stamped.pose.orientation.z,
                                                message_pose_stamped.pose.orientation.w])
        rotation_baselink_to_ecef = Rotation.from_quat(quaternion_baselink_to_ecef).as_matrix()
        transformation_baselink_to_ecef = to_transformation_matrix(rotation_baselink_to_ecef, position_baselink_in_ecef)
        transformation_baselink_to_enu = self.transformation_ecef_to_enu.dot(transformation_baselink_to_ecef)
        self.broadcaster_dynamic.sendTransform(transformation_matrix_to_transform_stamped(
                transformation_baselink_to_enu, 'map', 'base_link', message_pose_stamped.header.stamp))

        # Publish the NavSatFix message (for rviz_satellite)
        info_frame_baselink = pygeodesy.ecef.EcefKarney(pygeodesy.datums.Datums.WGS84).reverse(
                position_baselink_in_ecef[0], position_baselink_in_ecef[1], position_baselink_in_ecef[2], M=True)
        message_nav_sat_fix = NavSatFix()
        message_nav_sat_fix.header.frame_id = 'base_link'
        message_nav_sat_fix.header.stamp = message_pose_stamped.header.stamp
        message_nav_sat_fix.latitude = info_frame_baselink.lat
        message_nav_sat_fix.longitude = info_frame_baselink.lon
        message_nav_sat_fix.altitude = info_frame_baselink.height
        self.publisher_navsatfix.publish(message_nav_sat_fix)

if __name__ == "__main__":
    # Initialize the ROS node
    rospy.init_node('pose_visualization')

    # Create the pose handler
    pose_handler = PoseHandler()

    # Process the callbacks
    rospy.spin()
