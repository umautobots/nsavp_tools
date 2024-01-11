# Silence all tensorflow output except errors
import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'
import warnings
warnings.filterwarnings("ignore", category=UserWarning)

import tensorflow as tf
tf.compat.v1.disable_eager_execution()

import argparse
import h5py
import numpy as np
from tqdm.auto import tqdm
import cv2
from scipy.spatial.distance import cdist
import json
import yaml
from pathlib import Path

from netvlad_tf.image_descriptor import ImageDescriptor

def compute_place_descriptors(netvlad, group_image_raw, indices_equispaced_images, undistortion_map_1,
    undistortion_map_2, path_file_timestamps):
    # Open image and timestamp datasets
    dataset_images = group_image_raw['images']
    dataset_timestamps = group_image_raw['timestamps']

    # Read in all timestamps and convert from nanoseconds to seconds
    timestamps = dataset_timestamps[:] * 1e-9

    # Open output timestamps file
    file_timestamps = open(path_file_timestamps, 'w')

    # Generate place descriptors
    print('Computing descriptors...')
    place_descriptors = []
    for index in tqdm(indices_equispaced_images):
        # Write out the timestamp
        file_timestamps.write(str(timestamps[index]) + '\n')

        # Read in the image
        image = dataset_images[index, :, :]

        # Debayer RGB images
        if group_image_raw.attrs['encoding'] == 'bayer_rggb8':
            image = cv2.cvtColor(image, cv2.COLOR_BayerBG2RGB)

        # Threshold and normalize thermal images
        elif group_image_raw.attrs['encoding'] == 'mono16':
            threshold_minimum = 22500
            threshold_maximum = 25000
            image[image < threshold_minimum] = threshold_minimum
            image[image > threshold_maximum] = threshold_maximum
            image = (((image.astype(np.float32) - threshold_minimum) * 255) /
                (threshold_maximum - threshold_minimum)).astype(np.uint8)

        # Undistort the image
        image = cv2.remap(image, undistortion_map_1, undistortion_map_2, cv2.INTER_LINEAR)

        # Crop RGB images to remove the hood
        if group_image_raw.attrs['encoding'] == 'bayer_rggb8':
            image = image[:850,:1224]

        # Compute NetVLAD descriptor
        place_descriptors.append(netvlad.describe(image))

    # Close timestamps file
    file_timestamps.close()

    return np.array(place_descriptors)

def open_h5_files(path_h5_images, path_h5_poses):
    # Open H5 image file
    print('Opening H5 image file: ' + path_h5_images)
    h5_file_image = h5py.File(path_h5_images, 'r')
    try:
        group_image_raw = h5_file_image['image_raw']
    except:
        print("Failed to open '/image_raw' group in H5 file: " + path_h5_images)
        exit()

    # Open H5 pose file
    print('Opening H5 pose file: ' + path_h5_poses)
    h5_file_pose = h5py.File(path_h5_poses, 'r')
    try:
        group_poses = h5_file_pose['pose_base_link']
    except:
        print("Failed to open '/pose_base_link' group in H5 file: " + path_h5_poses)
        exit()

    return group_image_raw, group_poses

def get_undistortion_maps(path_calibration_results, frame_id):
    # Open calibration results file
    print('Reading in calibration results for ' + frame_id + ' from file: ' + path_calibration_results)
    with open(path_calibration_results, 'r') as file_object:
        try:
            dict_calibration = yaml.safe_load(file_object)
            for key in dict_calibration.keys():
                if dict_calibration[key]['frame_id'] == frame_id:
                    info_calibration = dict_calibration[key]
        except yaml.YAMLError as exception:
            print(exception)
            exit()

    print('Computing undistortion maps.')

    # Unpack camera calibration info
    distortion_coefficients = np.array(info_calibration['distortion_coeffs'])
    intrinsics = info_calibration['intrinsics']
    camera_matrix = np.eye(3)
    camera_matrix[0, 0] = intrinsics[0]
    camera_matrix[1, 1] = intrinsics[1]
    camera_matrix[0, 2] = intrinsics[2]
    camera_matrix[1, 2] = intrinsics[3]
    resolution = info_calibration['resolution']

    # Compute new camera matrix (using alpha = 0 to ensure no invalid pixels)
    camera_matrix_new, _ = cv2.getOptimalNewCameraMatrix(camera_matrix, distortion_coefficients, tuple(resolution),
        alpha=0)

    # Initialize undistortion maps
    undistortion_map_1, undistortion_map_2 = cv2.initUndistortRectifyMap(camera_matrix, distortion_coefficients, None,
        camera_matrix_new, tuple(resolution), cv2.CV_16SC2)

    return undistortion_map_1, undistortion_map_2

def select_images(group_image_raw, group_poses, position_endpoint_0, position_endpoint_1, spacing):
    # Unpack required data from H5 groups
    positions = group_poses['positions'][:]
    timestamps_positions = group_poses['timestamps'][:] * 1e-9
    timestamps_images = group_image_raw['timestamps'][:] * 1e-9

    # Determine equispaced images along the route
    position_endpoint_0 = np.expand_dims(np.asarray(position_endpoint_0), 1)
    position_endpoint_1 = np.expand_dims(np.asarray(position_endpoint_1), 1)
    index_endpoint_0 = np.argmin(np.linalg.norm(position_endpoint_0 - positions.T, axis=0))
    index_endpoint_1 = np.argmin(np.linalg.norm(position_endpoint_1 - positions.T, axis=0))
    index_begin = index_endpoint_0
    index_end = index_endpoint_1
    if (index_endpoint_0 > index_endpoint_1):
        index_begin = index_endpoint_1
        index_end = index_endpoint_0

    distance_path = 0
    position_prev = np.zeros((3))
    indices_equispaced_images = []
    for index_ground_truth in tqdm(range(index_begin, index_end + 1)):
        distance_path += np.linalg.norm(positions[index_ground_truth, :] - position_prev)
        if distance_path > spacing:
            indices_equispaced_images.append(
                np.argmin(np.abs(timestamps_positions[index_ground_truth] - timestamps_images)))
            distance_path = 0
        position_prev = positions[index_ground_truth, :]

    return indices_equispaced_images

if __name__ == "__main__":
    # Process arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('path_h5_query_images', type=str, help='Path to query image H5 file')
    parser.add_argument('path_h5_query_poses', type=str, help='Path to query pose H5 file')
    parser.add_argument('path_calibration_results_query', type=str, help='Path to query calibration YAML file.')
    parser.add_argument('path_h5_reference_images', type=str, help='Path to reference image H5 file')
    parser.add_argument('path_h5_reference_poses', type=str, help='Path to reference pose H5 file')
    parser.add_argument('path_calibration_results_reference', type=str, help='Path to reference calibration YAML file.')
    parser.add_argument('path_output_folder', type=str, help='Path to output folder')
    parser.add_argument('--position-endpoint-0', nargs='+', type=float,
        default=[512123.256980, -4698109.353169, 4269242.045445], help='ECEF position of first route endpoint '
        '(meters).')
    parser.add_argument('--position-endpoint-1', nargs='+', type=float,
        default=[517731.144315, -4699324.549431, 4267227.657290], help='ECEF position of second route endpoint '
        '(meters)')
    parser.add_argument('--spacing', type=float, default=2.0, help='Path distance between processed images (meters).')
    parser.add_argument('--save-distance-matrix', action='store_true', help='Save the distance matrix (default: False)')

    args = parser.parse_args()

    # Open H5 files and check the image encoding matches
    group_image_raw_query, group_poses_query = open_h5_files(args.path_h5_query_images, args.path_h5_query_poses)
    group_image_raw_reference, group_poses_reference = open_h5_files(args.path_h5_reference_images,
        args.path_h5_reference_poses)
    frame_id_query = Path(args.path_h5_query_images).parts[-1][7:-3] + '_optical'
    frame_id_reference = Path(args.path_h5_reference_images).parts[-1][7:-3] + '_optical'

    # Check encoding matches
    if group_image_raw_query.attrs['encoding'] != group_image_raw_reference.attrs['encoding']:
        print('The image encoding does not match across the two input H5 files')
        exit()

    # Open calibration results file and compute undistortion maps
    undistortion_map_1_query, undistortion_map_2_query = get_undistortion_maps(args.path_calibration_results_query,
        frame_id_query)
    undistortion_map_1_reference, undistortion_map_2_reference = get_undistortion_maps(
        args.path_calibration_results_reference, frame_id_reference)

    # Select equispaced images to process
    print('Processing ground truth positions to determine equispaced images in each sequence')
    indices_equispaced_images_query = select_images(group_image_raw_query, group_poses_query, args.position_endpoint_0,
        args.position_endpoint_1, args.spacing)
    indices_equispaced_images_reference = select_images(group_image_raw_reference, group_poses_reference,
        args.position_endpoint_0, args.position_endpoint_1, args.spacing)

    # Create output directory
    print('Creating output directory: ' + args.path_output_folder)
    if not os.path.exists(args.path_output_folder):
        os.makedirs(args.path_output_folder)

    # Initialize NetVLAD descriptor generator
    print('Initializing NetVLAD descriptor generator')
    is_grayscale = not (group_image_raw_query.attrs['encoding'] == 'bayer_rggb8')
    netvlad = ImageDescriptor(is_grayscale)

    # Compute reference place descriptors
    print('Computing reference place descriptors')
    place_descriptors_reference = compute_place_descriptors(netvlad, group_image_raw_reference,
        indices_equispaced_images_reference, undistortion_map_1_reference, undistortion_map_2_reference,
        args.path_output_folder + '/timestamps_reference.txt')

    # Compute query place descriptors
    print('Computing query place descriptors')
    place_descriptors_query = compute_place_descriptors(netvlad, group_image_raw_query,
        indices_equispaced_images_query, undistortion_map_1_query, undistortion_map_2_query,
        args.path_output_folder + '/timestamps_query.txt')

    # Compute distance matrix
    print('Computing distance matrix')
    distance_matrix = cdist(place_descriptors_reference, place_descriptors_query, 'sqeuclidean')
    if args.save_distance_matrix:
        print('Writing out distance matrix')
        np.savetxt(args.path_output_folder + '/distance_matrix.txt', distance_matrix, fmt='%f')

    # Determine match candidates
    print('Determining and writing out match candidates')
    indices_query = np.arange(distance_matrix.shape[1])
    indices_reference = np.argmin(distance_matrix, axis=0)
    distances = distance_matrix[indices_reference, np.arange(distance_matrix.shape[1])]
    match_candidates_dict = {
        "distances" : distances.tolist(),
        "query_ids" : indices_query.tolist(),
        "reference_ids" : indices_reference.tolist()}
    file_match_candidates = open(args.path_output_folder + "/match_candidates.json", 'w')
    file_match_candidates.write(json.dumps(match_candidates_dict))
    file_match_candidates.close()

    # Write out the trial settings
    file_arguments = open(args.path_output_folder + "/arguments.json", 'w')
    file_arguments.write(json.dumps(vars(args)))
    file_arguments.close()
