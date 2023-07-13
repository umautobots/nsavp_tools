#!/usr/bin/env python3

import numpy as np
import time

import rospy
import message_filters
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge

class StereoRectificationVisualizor:
    def __init__(self, stereo_pair, index_stereo_pair):
        # Set rectified image topics
        self.topic_image_0 = self.get_rectified_topic(stereo_pair[0])
        self.topic_image_1 = self.get_rectified_topic(stereo_pair[1])
        if self.topic_image_0 is None or self.topic_image_1 is None:
            return

        # Set the camera info topic of the second camera
        self.topic_camera_info_1 = stereo_pair[1] + '/camera_info'

        # Create the concatenated Image publisher
        topic_concatenated_image = 'stereo_pair_' + str(index_stereo_pair)
        self.publisher_concatenated = rospy.Publisher(topic_concatenated_image, Image, queue_size=10)

        # Create time synchronizer and set the callback
        self.cv_bridge = CvBridge()
        filter_image_0 = message_filters.Subscriber(self.topic_image_0, Image)
        filter_image_1 = message_filters.Subscriber(self.topic_image_1, Image)
        filter_camera_info_1 = message_filters.Subscriber(self.topic_camera_info_1, CameraInfo)
        if (not 'dvxplorer' in self.topic_image_0) and (not 'dvxplorer' in self.topic_image_1):
            time_synchronizer = message_filters.TimeSynchronizer(
                [filter_image_0, filter_image_1, filter_camera_info_1], queue_size=1)
        else:
            time_synchronizer = message_filters.ApproximateTimeSynchronizer(
                [filter_image_0, filter_image_1, filter_camera_info_1], queue_size=1, slop=0.05)
        time_synchronizer.registerCallback(self.callback_stereo_images)

    def get_rectified_topic(self, namespace):
        # Get all topics under the specified namespace which contain 'rect', wait until there is at least one
        topics_rectified = []
        while (len(topics_rectified) == 0) and not rospy.is_shutdown():
            topics_advertised = rospy.get_published_topics()
            topics_rectified = [topic_advertised[0] for topic_advertised in topics_advertised if
                (namespace in topic_advertised[0] and 'rect' in topic_advertised[0])]
            time.sleep(0.1)
        if len(topics_rectified) == 0:
            return None

        # Remove topics that were cropped after rectification
        topics_rectified = [topic_rectified for topic_rectified in topics_rectified if
            not ('rect_crop' in topic_rectified or 'rect_color_crop' in topic_rectified)]

        # There should be only one or two topics remaining, if two, choose the thresholded one
        for topic_rectified in topics_rectified:
            if 'thresholded' in topic_rectified:
                return topic_rectified
        return topics_rectified[0]

    def callback_stereo_images(self, message_image_0, message_image_1, message_camera_info_1):
        # Convert ROS image messages to OpenCV images
        encoding = 'bgr8' if (message_image_0.encoding == 'bgr8' or message_image_1.encoding == 'bgr8') else 'mono8'
        image_0 = self.cv_bridge.imgmsg_to_cv2(message_image_0, desired_encoding=encoding)
        image_1 = self.cv_bridge.imgmsg_to_cv2(message_image_1, desired_encoding=encoding)

        # Concatenate images and draw lines
        line_thickness = int(np.ceil(image_0.shape[0] // 200))
        if abs(message_camera_info_1.P[3]) > 0:
            if message_camera_info_1.P[3] < 0:
                image_concatenated = np.hstack((image_0, image_1))
            elif message_camera_info_1.P[3] > 0:
                image_concatenated = np.hstack((image_1, image_0))
            for i in range(10):
                index_row = i * (image_concatenated.shape[0] // 10)
                image_concatenated[index_row:index_row + line_thickness, :] = 255
        elif abs(message_camera_info_1.P[7]) > 0:
            if message_camera_info_1.P[7] < 0:
                image_concatenated = np.vstack((image_0, image_1))
            elif message_camera_info_1.P[7] > 0:
                image_concatenated = np.vstack((image_1, image_0))
            for i in range(10):
                index_column = i * (image_concatenated.shape[1] // 10)
                image_concatenated[:, index_column:index_column + line_thickness] = 255
        else:
            print('The projection matrix of the second camera contains no translation. The relative placement of the '
                  'cameras cannot be determined. Exiting.')
            exit()

        # Publish the concatenated image
        message_image_concatenated = self.cv_bridge.cv2_to_imgmsg(image_concatenated, encoding=encoding)
        message_image_concatenated.header = message_image_0.header
        self.publisher_concatenated.publish(message_image_concatenated)

if __name__ == "__main__":
    # Initialize ROS node
    rospy.init_node('stereo_rectification_visualization')

    # Process argument
    debayer = rospy.get_param('~debayer', default=False)
    stereo_pairs = rospy.get_param('~stereo_pairs', default=[])

    # Create stereo rectification visualizors
    stereo_rectification_visualizors = []
    for i, stereo_pair in enumerate(stereo_pairs):
        # Rectification is not performed on bayer encoded images
        if not debayer and ('rgb' in stereo_pair[0] or 'rgb' in stereo_pair[1]):
            continue

        stereo_rectification_visualizors.append(StereoRectificationVisualizor(stereo_pair, i))

    # Process the callbacks
    rospy.spin()
