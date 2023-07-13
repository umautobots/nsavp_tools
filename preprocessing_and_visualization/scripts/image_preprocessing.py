#!/usr/bin/env python3

import numpy as np
import cv2
import yaml

import rospy
import tf2_ros
from sensor_msgs.msg import CameraInfo, Image
from scipy.spatial.transform import Rotation
from cv_bridge import CvBridge

from preprocessing_and_visualization.common_utils import \
    invert_transformation_matrix, transformation_matrix_to_transform_stamped

def get_camera_id_from_namespace(data_calibrated, namespace):
    frame_id = namespace + '_optical'
    for key_camera in data_calibrated.keys():
        if data_calibrated[key_camera]['frame_id'] == frame_id:
            return key_camera
    print('frame_id ' + frame_id + ' is not included in the calibration file. Exiting.')
    exit()

class BoundingBox:
    def __init__(self, x = 0, y = 0, width = 0, height = 0):
        self.x = x
        self.y = y
        self.width = width
        self.height = height

    def get_intersection(self, other):
        intersection = BoundingBox()
        intersection.x = max(self.x, other.x)
        intersection.y = max(self.y, other.y)
        intersection.width = min(self.x + self.width, other.x + other.width) - intersection.x
        intersection.height = min(self.y + self.height, other.y + other.height) - intersection.y
        return intersection

def get_crop_parameters_from_namespace(crop_parameters_list, namespace):
    for crop_parameters in crop_parameters_list:
        if crop_parameters[0] == namespace:
            return BoundingBox(crop_parameters[1], crop_parameters[2], crop_parameters[3], crop_parameters[4])
    return None

def construct_camera_info_message(distortion_coefficients, camera_matrix, resolution, distortion_model,
    rectification_matrix, projection_matrix, frame_id):
    message_camera_info = CameraInfo()
    message_camera_info.header.frame_id = frame_id
    message_camera_info.height = resolution[1]
    message_camera_info.width = resolution[0]
    message_camera_info.distortion_model = distortion_model
    if distortion_model == 'radtan':
        message_camera_info.distortion_model = 'plumb_bob'
    message_camera_info.D = distortion_coefficients
    message_camera_info.K = camera_matrix.flatten().tolist()
    message_camera_info.R = rectification_matrix.flatten().tolist()
    message_camera_info.P = projection_matrix.flatten().tolist()
    message_camera_info.binning_x = 1
    message_camera_info.binning_y = 1
    message_camera_info.roi.x_offset = 0
    message_camera_info.roi.y_offset = 0
    message_camera_info.roi.height = resolution[1]
    message_camera_info.roi.width = resolution[0]
    message_camera_info.roi.do_rectify = False

    return message_camera_info

def unpack_rotation_and_translation(message_transform_stamped):
    translation = np.zeros((3,))
    translation[0] = message_transform_stamped.transform.translation.x
    translation[1] = message_transform_stamped.transform.translation.y
    translation[2] = message_transform_stamped.transform.translation.z

    quaternion = np.zeros((4,))
    quaternion[0] = message_transform_stamped.transform.rotation.x
    quaternion[1] = message_transform_stamped.transform.rotation.y
    quaternion[2] = message_transform_stamped.transform.rotation.z
    quaternion[3] = message_transform_stamped.transform.rotation.w
    rotation_matrix = Rotation.from_quat(quaternion).as_matrix()

    return translation, rotation_matrix

def unpack_calibration_info(info_calibration):
    distortion_coefficients_0 = np.array(info_calibration['distortion_coeffs'])
    intrinsics_0 = info_calibration['intrinsics']
    camera_matrix_0 = np.eye(3)
    camera_matrix_0[0, 0] = intrinsics_0[0]
    camera_matrix_0[1, 1] = intrinsics_0[1]
    camera_matrix_0[0, 2] = intrinsics_0[2]
    camera_matrix_0[1, 2] = intrinsics_0[3]
    resolution_0 = info_calibration['resolution']
    distortion_model_0 = info_calibration['distortion_model']

    return distortion_coefficients_0, camera_matrix_0, resolution_0, distortion_model_0

def generate_camera_info_messages(camera_parameters_dict, data_calibrated, key_camera_0, key_camera_1 = None):
    # Unpack camera calibration info of the first camera
    distortion_coefficients_0, camera_matrix_0, resolution_0, distortion_model_0 = \
        unpack_calibration_info(data_calibrated[key_camera_0])

    # If cropping is to be applied to raw images, handle cropping parameters for the first camera
    camera_parameters_0 = camera_parameters_dict[key_camera_0]
    if camera_parameters_0['crop_raw'] and camera_parameters_0['crop_parameters'] is not None:
        # Subtract crop offset from the principal point
        camera_matrix_0[0, 2] -= camera_parameters_0['crop_parameters'].x
        camera_matrix_0[1, 2] -= camera_parameters_0['crop_parameters'].y

        # Change resolution to that of the cropped image
        resolution_0[0] = camera_parameters_0['crop_parameters'].width
        resolution_0[1] = camera_parameters_0['crop_parameters'].height

    # Set the rectified image resolution
    if not camera_parameters_0['resolution_rectified'] == []:
        resolution_rectified = camera_parameters_0['resolution_rectified']
    else:
        resolution_rectified = resolution_0

    # Handle monocular case
    if key_camera_1 == None:
        # Compute new camera matrix
        if distortion_model_0 == 'radtan':
            # Using alpha = 0 to ensure no invalid pixels
            camera_matrix_0_new, _ = cv2.getOptimalNewCameraMatrix(
                camera_matrix_0, distortion_coefficients_0, tuple(resolution_0), alpha=0,
                newImgSize=tuple(resolution_rectified))
        elif distortion_model_0 == 'equidistant':
            # Using balance = 0 to reduce invalid pixels
            camera_matrix_0_new = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
                camera_matrix_0, distortion_coefficients_0, tuple(resolution_0), np.eye(3), balance=0,
                new_size=tuple(resolution_rectified))
        else:
            print('Unsupported distortion model. Exiting.')
            exit()

        # Set the rectification matrix to identity and construct the projection matrix with no translation
        rectification_matrix_0 = np.eye(3)
        projection_matrix_0 = np.hstack((camera_matrix_0_new, np.zeros((3, 1))))

    # Handle stereo case
    else:
        # Unpack camera calibration info of the second camera
        distortion_coefficients_1, camera_matrix_1, resolution_1, distortion_model_1 = \
            unpack_calibration_info(data_calibrated[key_camera_1])

        # If cropping is to be applied to raw images, handle cropping parameters for the second camera
        camera_parameters_1 = camera_parameters_dict[key_camera_1]
        if camera_parameters_1['crop_raw'] and camera_parameters_1['crop_parameters'] is not None:
            # Subtract crop offset from the principal point
            camera_matrix_1[0, 2] -= camera_parameters_1['crop_parameters'].x
            camera_matrix_1[1, 2] -= camera_parameters_1['crop_parameters'].y

            # Change resolution to that of the cropped image
            resolution_1[0] = camera_parameters_1['crop_parameters'].width
            resolution_1[1] = camera_parameters_1['crop_parameters'].height

        # Get the transformation from the first camera to the second camera
        buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(buffer)
        message_transform_stamped_0_to_1 = buffer.lookup_transform(data_calibrated[key_camera_1]['frame_id'],
            data_calibrated[key_camera_0]['frame_id'], rospy.Time(), timeout=rospy.Duration.from_sec(10.0))
        position_0_in_1, rotation_0_to_1 = unpack_rotation_and_translation(message_transform_stamped_0_to_1)

        camera_matrix_1_resize = np.copy(camera_matrix_1)
        if resolution_0 != resolution_1:
            # Adjust the second camera's intrinsics as though it were resized to the resolution of the first to
            # accomodate OpenCV's stereoRectify algorithm which assumes the same input resolution across both cameras
            camera_matrix_1_resize[0, :] *= (resolution_0[0] / resolution_1[0])
            camera_matrix_1_resize[1, :] *= (resolution_0[1] / resolution_1[1])

        # Compute rectification and projection matrices
        if distortion_model_0 == 'radtan':
            # NOTE: stereoRectify crops images to be unnecessarily small with alpha=0. We are using the workaround
            # described in [this](https://github.com/opencv/opencv/issues/7240) GitHub issue.

            # Perform stereo rectification with alpha=1
            # This returns rectification and projection matrices that yield rectified images that retain all information
            # from the original images, but also contain invalid pixels. The variables roi_valid_0 and roi_valid_1 are
            # bounding boxes represented as tuples (x, y, width, height) which specify the largest rectangular patches
            # of these rectified images which contain only valid pixels.
            rectification_matrix_0, rectification_matrix_1, projection_matrix_0, projection_matrix_1, _, roi_valid_0, \
                roi_valid_1 = cv2.stereoRectify(camera_matrix_0, distortion_coefficients_0, camera_matrix_1_resize,
                distortion_coefficients_1, tuple(resolution_0), rotation_0_to_1, position_0_in_1,
                flags=cv2.CALIB_ZERO_DISPARITY, alpha=1, newImageSize=tuple(resolution_rectified))

            # Determine the intersection of the two valid ROIs in the rectified images
            roi_valid_0 = BoundingBox(roi_valid_0[0], roi_valid_0[1], roi_valid_0[2], roi_valid_0[3])
            roi_valid_1 = BoundingBox(roi_valid_1[0], roi_valid_1[1], roi_valid_1[2], roi_valid_1[3])
            roi_intersection = roi_valid_0.get_intersection(roi_valid_1)
            if roi_intersection.width == 0 or roi_intersection.height == 0:
                print('Stereo rectification between between frame_id ' + data_calibrated[key_camera_0]['frame_id'] +
                      ' and frame_id ' + data_calibrated[key_camera_1]['frame_id'] + ' failed. Exiting.')
                exit()

            # Reduce the width or height of the intersection bounding box to ensure the aspect ratio matches that of
            # the desired rectified images
            aspect_ratio = resolution_rectified[0] / resolution_rectified[1]
            if roi_intersection.width > roi_intersection.height * aspect_ratio:
                roi_intersection.x = roi_intersection.x + \
                    (roi_intersection.width - roi_intersection.height * aspect_ratio) / 2.0
                roi_intersection.width = roi_intersection.height * aspect_ratio
            else:
                roi_intersection.y = roi_intersection.y + \
                    (roi_intersection.height - roi_intersection.width / aspect_ratio) / 2.0
                roi_intersection.height = roi_intersection.width / aspect_ratio

            # Adjust the projection matrices such that the rectified image pixels are sampled within the intersection
            # bounding box. This ensures there will be no invalid pixels in the rectified images
            projection_matrix_1[0, 2] -= roi_intersection.x
            projection_matrix_1[0, :] *= resolution_rectified[0] / roi_intersection.width
            projection_matrix_1[1, 2] -= roi_intersection.y
            projection_matrix_1[1, :] *= resolution_rectified[1] / roi_intersection.height

            projection_matrix_0[0, 2] -= roi_intersection.x
            projection_matrix_0[0, :] *= resolution_rectified[0] / roi_intersection.width
            projection_matrix_0[1, 2] -= roi_intersection.y
            projection_matrix_0[1, :] *= resolution_rectified[1] / roi_intersection.height

        else:
            print('Unsupported distortion model. Currently this script only supports the radtan / plumb_bob distortion '
                  'model for stereo rectification. Exiting.')
            exit()

    # Construct the camera info message or messages
    # NOTE: we set the width and height fields of the camera info messages to that of the rectified resolution. This
    # width and height are later passed to initUndistortRectifyMap. This violates the ROS standard which does not allow
    # for a differing resolution between raw and rectified images (see ROS REP 104), but this is required for rectifying
    # stereo images with different raw resolutions. We also use it here to support arbitrary rectified resolutions.
    camera_parameters_dict[key_camera_0]['message_camera_info'] = \
        construct_camera_info_message(distortion_coefficients_0, camera_matrix_0, resolution_rectified,
                                        distortion_model_0, rectification_matrix_0, projection_matrix_0,
                                        data_calibrated[key_camera_0]['frame_id'])
    if not key_camera_1 == None:
        camera_parameters_dict[key_camera_1]['message_camera_info'] = \
            construct_camera_info_message(distortion_coefficients_1, camera_matrix_1, resolution_rectified,
                                          distortion_model_1, rectification_matrix_1, projection_matrix_1,
                                          data_calibrated[key_camera_1]['frame_id'])

    return camera_parameters_dict

class ImageProcessor:
    def __init__(self, camera_parameters):
        self.camera_parameters = camera_parameters

        # Initialize undistortion/rectification maps from the camera info message
        if self.camera_parameters['rectify']:
            # NOTE: using CV_16SC2 enables faster remapping, see OpenCV's documentation of convertMaps()
            message_camera_info = self.camera_parameters['message_camera_info']
            if message_camera_info.distortion_model == 'plumb_bob':
                self.undistortion_map_1, self.undistortion_map_2 = cv2.initUndistortRectifyMap(
                    np.array(message_camera_info.K).reshape(3, 3),
                    np.array(message_camera_info.D),
                    np.array(message_camera_info.R).reshape(3, 3),
                    np.array(message_camera_info.P).reshape(3, 4),
                    (message_camera_info.width, message_camera_info.height),
                    cv2.CV_16SC2)
            elif message_camera_info.distortion_model == 'equidistant':
                self.undistortion_map_1, self.undistortion_map_2 = cv2.fisheye.initUndistortRectifyMap(
                    np.array(message_camera_info.K).reshape(3, 3),
                    np.array(message_camera_info.D),
                    np.array(message_camera_info.R).reshape(3, 3),
                    np.array(message_camera_info.P).reshape(3, 4),
                    (message_camera_info.width, message_camera_info.height),
                    cv2.CV_16SC2)
            else:
                print('Unsupported distortion model. Exiting.')
                exit()

        # Initialize the OpenCV bridge
        self.cv_bridge = CvBridge()

        # Create camera info publisher
        topic_camera_info = self.camera_parameters['namespace'] + '/camera_info'
        self.publisher_cam_info = rospy.Publisher(topic_camera_info, CameraInfo, queue_size=10)

        # Create image publishers
        image_type = 'image_raw'

        self.publisher_debayer = None
        if self.camera_parameters['debayer']:
            image_type = 'image_color'
            topic_debayer = self.camera_parameters['namespace'] + '/' + image_type
            self.publisher_debayer = rospy.Publisher(topic_debayer, Image, queue_size=10)

        self.publisher_crop_raw = None
        if self.camera_parameters['crop_raw'] and self.camera_parameters['crop_parameters'] is not None:
            image_type = image_type.replace('raw', 'crop').replace('color', 'crop_color')
            topic_crop_raw = self.camera_parameters['namespace'] + '/' + image_type
            self.publisher_crop_raw = rospy.Publisher(topic_crop_raw, Image, queue_size=10)

        self.publisher_rectify = None
        if self.camera_parameters['rectify']:
            image_type = image_type.replace('raw', 'rect').replace('crop', 'crop_rect').replace(
                'image_color', 'image_rect_color')
            topic_rectify = self.camera_parameters['namespace'] + '/' + image_type
            self.publisher_rectify = rospy.Publisher(topic_rectify, Image, queue_size=10)

        self.publisher_crop_rectified = None
        if self.camera_parameters['rectify'] and not self.camera_parameters['crop_raw'] and \
            self.camera_parameters['crop_parameters'] is not None:
            image_type = image_type + '_crop'
            topic_crop_rectified = self.camera_parameters['namespace'] + '/' + image_type
            self.publisher_crop_rectified = rospy.Publisher(topic_crop_rectified, Image, queue_size=10)

        self.publisher_threshold = None
        if self.camera_parameters['threshold']:
            image_type = image_type + '_thresholded'
            topic_thresholded = self.camera_parameters['namespace'] + '/' + image_type
            self.publisher_threshold = rospy.Publisher(topic_thresholded, Image, queue_size=10)

        topic_processed = self.camera_parameters['namespace'] + '/image_processed'
        self.publisher_processed = rospy.Publisher(topic_processed, Image, queue_size=10)

        # Set the Image callback
        topic_image = self.camera_parameters['namespace'] + '/image_raw'
        rospy.Subscriber(topic_image, Image, self.callback_image)

    def publish(self, publisher, image, header):
        if publisher.get_num_connections() > 0:
            message_image = self.cv_bridge.cv2_to_imgmsg(image)
            message_image.header = header
            if message_image.encoding == '8UC1':
                message_image.encoding = 'mono8'
            elif message_image.encoding == '16UC1':
                message_image.encoding = 'mono16'
            elif message_image.encoding == '8UC3':
                message_image.encoding = 'bgr8'
            publisher.publish(message_image)

    def callback_image(self, message_image):
        # Publish camera info message
        self.camera_parameters['message_camera_info'].header.stamp = message_image.header.stamp
        self.publisher_cam_info.publish(self.camera_parameters['message_camera_info'])

        # Convert ROS image message to OpenCV image
        image = self.cv_bridge.imgmsg_to_cv2(message_image)

        # Debayer
        if self.publisher_debayer is not None:
            # NOTE: OpenCV supports other options such as COLOR_BayerBG2BGR_VNG and COLOR_BayerBG2BGR_EA
            image = cv2.cvtColor(image, cv2.COLOR_BayerBG2BGR)
            self.publish(self.publisher_debayer, image, message_image.header)

        # Crop raw (or debayered) image
        if self.publisher_crop_raw is not None:
            roi_crop = self.camera_parameters['crop_parameters']
            image = image[roi_crop.y : roi_crop.y + roi_crop.height, roi_crop.x : roi_crop.x + roi_crop.width]
            self.publish(self.publisher_crop_raw, image, message_image.header)

        # Rectify
        if self.publisher_rectify is not None:
            # NOTE: OpenCV supports several other interpolation options, see the InterpolationFlags in the documentation
            image = cv2.remap(image, self.undistortion_map_1, self.undistortion_map_2, cv2.INTER_LINEAR)
            self.publish(self.publisher_rectify, image, message_image.header)

        # Crop rectified image
        if self.publisher_crop_rectified is not None:
            roi_crop = self.camera_parameters['crop_parameters']
            image = image[roi_crop.y : roi_crop.y + roi_crop.height, roi_crop.x : roi_crop.x + roi_crop.width]
            self.publish(self.publisher_crop_rectified, image, message_image.header)

        # Threshold
        if self.publisher_threshold is not None:
            threshold_minimum = self.camera_parameters['threshold_thermal_minimum']
            threshold_maximum = self.camera_parameters['threshold_thermal_maximum']
            image[image < threshold_minimum] = threshold_minimum
            image[image > threshold_maximum] = threshold_maximum
            image = (((image.astype(np.float32) - threshold_minimum) * 255) /
                (threshold_maximum - threshold_minimum)).astype(np.uint8)
            self.publish(self.publisher_threshold, image, message_image.header)

        # Publish the processed image
        self.publish(self.publisher_processed, image, message_image.header)

if __name__ == "__main__":
    # Initialize ROS node
    rospy.init_node('image_preprocessing')

    # Process arguments
    path_calibration_results = rospy.get_param('~path_calibration_results', default='')
    debayer = rospy.get_param('~debayer', default=True)
    crop_parameters_list = rospy.get_param('~crop_parameters_list', default=[])
    crop_raw = rospy.get_param('~crop_raw', default=False)
    rectify = rospy.get_param('~rectify', default=True)
    resolution_rectified = rospy.get_param('~resolution_rectified', default=[])
    stereo_pairs = rospy.get_param('~stereo_pairs', default=[])
    threshold_thermal = rospy.get_param('~threshold_thermal', default=True)
    threshold_thermal_minimum = rospy.get_param('~threshold_thermal_minimum', default=22500)
    threshold_thermal_maximum = rospy.get_param('~threshold_thermal_maximum', default=25000)
    visualize_calibrated_transforms = rospy.get_param('~visualize_calibrated_transforms', default=False)

    if path_calibration_results == '':
        print('The `path_calibration_results` is empty. Exiting.')
        exit()

    # Open calibration results file
    with open(path_calibration_results, 'r') as file_object:
        try:
            data_calibrated = yaml.safe_load(file_object)
        except yaml.YAMLError as exception:
            print(exception)
            exit()

    # Compile the chain of static transforms between camera optical frames
    # NOTE: all static transforms broadcasted within a single node must be broadcasted at once
    messages_static_transform_stamped = []
    for i, key_camera in enumerate(data_calibrated.keys()):
        if not i == 0:
            messages_static_transform_stamped.append(transformation_matrix_to_transform_stamped(
                invert_transformation_matrix(np.asarray(data_calibrated[key_camera]['T_cn_cnm1'])),
                data_calibrated[key_camera_previous]['frame_id'],
                data_calibrated[key_camera]['frame_id']))
        key_camera_previous = key_camera

    # If the calibrated transforms are to be visualized, broadcast a static transform between the ADK optical frame and
    # housing frame assuming zero translation between the frames (note that this is imprecise)
    if visualize_calibrated_transforms:
        # Construct the transformation matrix from the optical frame to the housing frame
        # Optical frame axes: x-axis pointing right, y-axis pointing down, and z-axis pointing forward
        # Housing frame axes: x-axis pointing forward, y-axis pointing left, and z-axis pointing up
        # Therefore, the rotation swaps the axes as follows: z -> x, -x -> y, -y -> z
        transformation_optical_to_housing = np.eye(4)
        transformation_optical_to_housing[0:3, 0:3] = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
        messages_static_transform_stamped.append(transformation_matrix_to_transform_stamped(
            transformation_optical_to_housing, 'adk_left_housing', 'adk_left_optical'))

    # Broadcast static transforms
    broadcaster_static = tf2_ros.StaticTransformBroadcaster()
    broadcaster_static.sendTransform(messages_static_transform_stamped)

    # Initialize nested dictionary of camera parameters
    camera_parameters_dict = {}
    for key_camera in data_calibrated.keys():
        camera_parameters_dict[key_camera] = {}
        camera_parameters = camera_parameters_dict[key_camera]
        camera_parameters['namespace'] = data_calibrated[key_camera]['frame_id'][0:-(len('_optical'))]
        camera_parameters['crop_parameters'] = get_crop_parameters_from_namespace(crop_parameters_list,
            camera_parameters['namespace'])
        camera_parameters['crop_raw'] = crop_raw
        camera_parameters['resolution_rectified'] = resolution_rectified
        if 'rgb' in camera_parameters['namespace']:
            camera_parameters['debayer'] = debayer
            camera_parameters['rectify'] = rectify and debayer
        else:
            camera_parameters['debayer'] = False
            camera_parameters['rectify'] = rectify
        if 'adk' in camera_parameters['namespace']:
            camera_parameters['threshold'] = threshold_thermal
            camera_parameters['threshold_thermal_minimum'] = threshold_thermal_minimum
            camera_parameters['threshold_thermal_maximum'] = threshold_thermal_maximum
        else:
            camera_parameters['threshold'] = False

    # Generate CameraInfo messages for all specified stereo pairs
    for stereo_pair in stereo_pairs:
        key_camera_0 = get_camera_id_from_namespace(data_calibrated, stereo_pair[0])
        if 'message_camera_info' in camera_parameters_dict[key_camera_0].keys():
            print('namespace ' + stereo_pair[0] + ' appears twice in the `stereo_pairs` list. Exiting.')
            exit()

        key_camera_1 = get_camera_id_from_namespace(data_calibrated, stereo_pair[1])
        if 'message_camera_info' in camera_parameters_dict[key_camera_1].keys():
            print('namespace ' + stereo_pair[1] + ' appears twice in the `stereo_pairs` list. Exiting.')
            exit()

        camera_parameters_dict = generate_camera_info_messages(camera_parameters_dict, data_calibrated, key_camera_0,
            key_camera_1)

    # Generate CameraInfo messages for all cameras not part of a specified stereo pair
    for key_camera in data_calibrated.keys():
        if not 'message_camera_info' in camera_parameters_dict[key_camera].keys():
            camera_parameters_dict = generate_camera_info_messages(camera_parameters_dict, data_calibrated, key_camera)

    # Create image processors
    image_processors = []
    for key_camera in data_calibrated.keys():
        image_processors.append(ImageProcessor(camera_parameters_dict[key_camera]))

    # Process the callbacks
    rospy.spin()
