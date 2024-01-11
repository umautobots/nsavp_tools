import numpy as np
import h5py
from scipy.interpolate import interp1d
from scipy.spatial.transform import Rotation, Slerp
import json
import argparse
import dearpygui.dearpygui as dpg
from screeninfo import get_monitors
from matplotlib import cm
import pygeodesy
import cv2
import yaml

class PlaceRecognitionEvaluator:
    """Computes and visualizes multi-session place recognition results.

    This class computes and visualizes multi-session place recognition results given match candidates and ground truth
    positions.

    A query i is considered to return an accepted reference match j if the descriptor distance (or match score) is less
    than or equal to a given threshold. For a given descriptor distance threshold, precision and recall are computed by
    assigning a label to each query as follows:
    - If query i returns an accepted reference match j
        - If |c_i - c_j| <= r_m the query is labeled a true positive
        - Otherwise, the query is labeled a false positive
    - If query i returns no accepted match
        - If there exists a reference j' such that |c_i - c_j'| <= r_m the query is labeled a false negative
        - Otherwise, the query is labeled a true negative
    where c_i, c_j, and c_j' are ground truth positions, and r_m is a localization radius.

    A precision-recall (PR) curve is drawn by varying the score threshold from the minimum to the maximum descriptor
    distance across all queries.

    From the PR curve two metrics are computed: maximum recall at 100 percent precision and the area under the PR curve
    (AUC). The maximum recall at 100 percent precision indicates the percentage of all possible matches successfully
    attained without any false positives while the AUC provides a summary of the performance that is less sensitive to
    individual false positives. In the ideal case, both metrics would equal 1.

    If multiple localization radii are specified a PR curve and its correponding metrics will be computed for each one.

    Note that this class handles cases where some query place descriptors are not assigned a candidate reference match,
    i.e. when there are entries in the timestamps_query.txt file with no corresponding match in the match candidates
    JSON file. This happens, for example, with sequence matching algorithms which require a search window of query
    keyframes such that no candidate reference match is produced for the first and last (w - 1) / 2 queries, where w is
    length of the search window.

    If a query has no candidate reference match, it is considered to return no accepted match by default. Therefore,
    it is either a false negative, if a reference match exists within the localization radius, or a true negative.

    False negatives resulting from queries that were not assigned a candidate reference match can lead to situations
    where all match candidates are correct yet recall is less than 1 (and therefore the maximum recall at 100 percent
    precision and the AUC are less than 1). The attributes n_false_negatives_no_candidate, n_false_positives_max_thresh,
    max_recalls_possible are included to identify when this situations arises. If n_false_negatives_no_candidate is
    greater than 0 and n_false_positives_max_thresh is zero, then all the match candidates are correct but the metrics
    will be less than 1. Note that in this situation the maximum recall at 100 percent precision and the AUC will also
    be equal. The max_recalls_possible attrubute gives the maximum recall obtainable given the false negatives resulting
    from queries that were not assigned a candidate reference match.

    Attributes (excluding those required only for plotting):
        positions_ground_truth_query: An array containing the ground truth positions for each place descriptor in the
            query sequence. The place descriptors are indexed down the rows and the xyz ECEF coordinates are indexed
            across the columns. The number of rows is equal to the number of entries in timestamps_query.txt.
        positions_ground_truth_reference: An array containing the ground truth positions for each place descriptor in
            the reference sequence. The place descriptors are indexed down the rows and the xyz ECEF coordinates are
            indexed across the columns. The number of rows is equal to the number of entries in
            timestamps_reference.txt.
        localization_radii: A list containing localization radii. A query and reference pair are considered a correct
            match if the physical distance between them is less than the localization radius. All metrics computed by
            this class are computed for each localization radius in localization_radii.
        match_candidates_estimate: A dictionary with the following keys:
            query_ids: A list containing indices for query place descriptors. The indices correspond to entries in the
                timestamps_query.txt file.
            reference_ids: A list (with the same length as query_ids) containing indices for reference place
                descriptors. The indices correspond to entries in the timestamps_reference.txt file. These reference
                IDs are match candidates for the corresponding query IDs.
            distances: A list (with the same length as query_ids) containing the descriptor distances between the
                corresponding query and reference place descriptors.
            physical_distances: A list (with the same length as query_ids) containing the physical distances between
                the corresponding query and reference places, computed using their ground truth positions.
        match_candidates_ground_truth: A dictionary with the following keys:
            query_ids: A list containing indices for query place descriptors, equivalent to
                match_candidates_estimate['query_ids'].  The indices correspond to entries in the timestamps_query.txt
                file.
            reference_ids: A list (with the same length as query_ids) containing indices for reference place
                descriptors. The indices correspond to entries in the timestamps_reference.txt file. These reference IDs
                indicate the closest reference place, by physical distance, for each of the corresponding query IDs.
            physical_distances: A list (with the same length as query_ids) containing the physical distances between
                the corresponding query and reference places, computed using their ground truth positions.
        aucs: A list (with the same length as localization_radii) containing the area under the precision recall
            curve computed for each localization radius.
        max_recalls_perfect_precision: A list (with the same length as localization_radii) containing the maximum
            recall at 100 percent precision computed for each localization radius.
        distance_thresholds_perfect_precison: A list (with the same length as localization_radii) containing the
            descriptor distance thresholds that correspond to the maximum recall at 100 percent precision for each
            localization radius.
        recalls: An array containing the recall values for various descriptor distance thresholds and localization
            radii. The localization radii are indexed down the rows and the descriptor distance thresholds are
            indexed across the columns.
        precisions: An array containing the precision values for various descriptor distance thresholds and localization
            radii. The localization radii are indexed down the rows and the descriptor distance thresholds are indexed
            across the columns.
        n_false_negatives_no_candidate: A list (with the same length as localization_radii) with the number of false
            negatives assigned to queries for which no candidate match was predicted.
        n_false_positives_max_thresh: A list (with the same length as localization_radii) with the number of false
            positives at the maximum descriptor distance threshold.
        max_recalls_possible: A list (with the same length as localization_radii) with the maximum recall possible
            (given false negatives assigned to queries with no candidate match)
    """

    def __init__(self, path_timestamps_query, path_timestamps_reference, path_ground_truth_query,
        path_ground_truth_reference, path_measured_extrinsics_query, path_measured_extrinsics_reference,
        namespace_query, namespace_reference, path_match_candidates, localization_radii):
        """
        Args:
            path_timestamps_query: A filepath to a txt file containing timestamps of query place descriptors.
            path_timestamps_reference: A filepath to a txt file containing timestamps of reference place descriptors.
            path_ground_truth_query: A filepath to a H5 file containing ground truth poses for the query sequence.
            path_ground_truth_reference: A filepath to a H5 file containing ground truth poses for the reference
                sequence.
            path_measured_extrinsics_query: A filepath to a YAML file containing the measured extrinsics for the query
                sequence.
            path_measured_extrinsics_reference: A filepath to a YAML file containing the measured extrinsics for the
                reference sequence.
            namespace_query: A string specifying a namespace (e.g. mono_left) which corresponds to the sensor
                used in generating the query place descriptors.
            namespace_reference: A string specifying a namespace (e.g. mono_left) which corresponds to the sensor
                used in generating the reference place descriptors.
            path_match_candidates: A filepath to a JSON file containing match candidates.
            localization_radii: A list containing localization radii. A query and reference pair are considered a
            correct match if the physical distance between them is less than the localization radius. All metrics
            computed by this class are computed for each localization radius in localization_radii.
        """

        # Compute interpolated ground truth positions
        self.timestamps_place_descriptors_query, self.positions_ground_truth_query = self.interpolate_ground_truth(
            path_timestamps_query, path_ground_truth_query, path_measured_extrinsics_query, namespace_query)
        self.timestamps_place_descriptors_reference, self.positions_ground_truth_reference = \
            self.interpolate_ground_truth(path_timestamps_reference, path_ground_truth_reference,
                path_measured_extrinsics_reference, namespace_reference)

        # Compute place recognition metrics
        self.localization_radii = localization_radii
        self.compute_metrics_single_trial(path_match_candidates)

    def interpolate_ground_truth(self, path_timestamps, path_ground_truth, path_measured_extrinsics, namespace):
        """Interpolates ground truth positions to the times of the place descriptors.

        Args:
            path_timestamps: A filepath to a txt file containing timestamps of place descriptors.
            path_ground_truth: A filepath to a H5 file containing ground truth poses.
            path_measured_extrinsics: A filepath to a YAML file containing measured extrinsics.
            namespace_query: A string specifying a namespace (e.g. mono_left) which corresponds to the sensor
                used in generating the place descriptors.

        Returns:
            A two element tuple. The first element is a 1D array of the place descriptor timestamps. The second element
            is an array containing the ground truth positions of each place descriptor. The place descriptors are
            indexed down the rows and the xyz ECEF coordinates are indexed across the columns. The number of rows is
            equal to the number of entries in the input timestamp txt file.
        """

        # Import place descriptor timestamps
        timestamps_estimate = np.loadtxt(path_timestamps)

        # Import ground truth
        h5_file = h5py.File(path_ground_truth, 'r')
        positions_ground_truth = h5_file['pose_base_link/positions'][:]
        quaternions_ground_truth = h5_file['pose_base_link/quaternions'][:]
        timestamps_ground_truth = h5_file['pose_base_link/timestamps'][:] * 1e-9

        # Interpolate ground truth poses to the timestamps of the place descriptors
        interpolator_positions = interp1d(timestamps_ground_truth, positions_ground_truth, axis=0, assume_sorted=True,
            fill_value='extrapolate')
        positions_interpolated = interpolator_positions(timestamps_estimate)
        interpolator_rotations = Slerp(timestamps_ground_truth, Rotation.from_quat(quaternions_ground_truth))
        rotations_interpolated = interpolator_rotations(timestamps_estimate)

        # Translate the ground truth positions from the base_link origin to the sensor origin
        # The interpolated ground truth positions are of the base_link in the ECEF frame and the interpolated rotations
        # are from the base_link frame to the ECEF frame. To compute the position of the sensor in the ECEF frame we
        # rotate the position of the sensor in the base_link frame into the ECEF frame and add this with the position
        # of the base_link in the ECEF frame.
        with open(path_measured_extrinsics, 'r') as file_object:
            try:
                transformation_sensor_to_base_link = np.asarray(
                    yaml.safe_load(file_object)['T_' + namespace + '_housing'])
            except yaml.YAMLError as exception:
                print(exception)
                exit()
        position_sensor_in_base_link = transformation_sensor_to_base_link[0:3, 3]
        positions_sensor_in_ecef = rotations_interpolated.apply(position_sensor_in_base_link) + positions_interpolated

        return timestamps_estimate, positions_sensor_in_ecef

    def compute_metrics_single_trial(self, path_match_candidates):
        """Computes multi-session place recognition metrics.

        Args:
            path_match_candidates: A filepath to a JSON file containing match candidates.
        """

        # Import estimated match candidates
        with open(path_match_candidates, 'r') as file_object:
            self.match_candidates_estimate = json.load(file_object)
        n_match_candidates = len(self.match_candidates_estimate['query_ids'])

        # Determine ground truth match candidates
        n_localization_radii = len(self.localization_radii)
        self.match_candidates_ground_truth = {}
        self.match_candidates_ground_truth['query_ids'] = self.match_candidates_estimate['query_ids']
        self.match_candidates_ground_truth['reference_ids'] = [None] * n_match_candidates
        self.match_candidates_ground_truth['physical_distances'] = [None] * n_match_candidates
        self.n_false_negatives_no_candidate = [0] * n_localization_radii
        for i in range(self.positions_ground_truth_query.shape[0]):
            # Determine the minimum physical distance between the query position and a reference position and the
            # corresponding reference ID
            physical_distances_ground_truth = np.linalg.norm(
                self.positions_ground_truth_query[i, :] - self.positions_ground_truth_reference, axis=1)
            reference_id_min_physical_distance = np.argmin(physical_distances_ground_truth)
            physical_distance_min = physical_distances_ground_truth[reference_id_min_physical_distance]

            # If there is an estimated match candidate for the current query, add a corresponding ground truth match
            # candidate
            if i in self.match_candidates_estimate['query_ids']:
                index_candidate = self.match_candidates_estimate['query_ids'].index(i)
                self.match_candidates_ground_truth['reference_ids'][index_candidate] = \
                    reference_id_min_physical_distance
                self.match_candidates_ground_truth['physical_distances'][index_candidate] = physical_distance_min

            # If there is no estimated match candidate for the current query and the physical distance to the closest
            # reference is beneath the localization radius, then this is a false negative match
            else:
                for j, localization_radius in enumerate(self.localization_radii):
                    if (physical_distance_min <= localization_radius):
                        self.n_false_negatives_no_candidate[j] += 1

        # Compute physical distances between match candidate queries and references
        self.match_candidates_estimate['physical_distances'] = []
        for query_id, reference_id in \
            zip(self.match_candidates_estimate['query_ids'], self.match_candidates_estimate['reference_ids']):
            physical_distance = np.linalg.norm(self.positions_ground_truth_query[query_id, :] -
                self.positions_ground_truth_reference[reference_id, :])
            self.match_candidates_estimate['physical_distances'].append(physical_distance)

        # Compute list of candidate indices sorted by descriptor distance in ascending order
        candidate_indices_best_to_worst = np.argsort(self.match_candidates_estimate['distances'])

        # Sort the match candidate ground truth physical distances in order of ascending descriptor distance
        ground_truth_physical_distances_sorted = \
            np.asarray(self.match_candidates_ground_truth['physical_distances'])[candidate_indices_best_to_worst]

        # Compute place recognition metrics for all localization radii
        self.aucs = []
        self.max_recalls_perfect_precision = [0] * n_localization_radii
        self.distance_thresholds_perfect_precison = [-np.inf] * n_localization_radii
        self.precisions = np.zeros((n_localization_radii, n_match_candidates))
        self.recalls = np.zeros((n_localization_radii, n_match_candidates))
        self.max_recalls_possible = []
        self.n_false_positives_max_thresh = []
        for i, localization_radius in enumerate(self.localization_radii):
            # Compute the maximum possible recall
            n_true_positive_max = np.sum(np.asarray(self.match_candidates_ground_truth['physical_distances'])
                <= localization_radius)
            self.max_recalls_possible.append(n_true_positive_max /
                (n_true_positive_max + self.n_false_negatives_no_candidate[i]))

            # Compute the PR curve
            n_true_positives = 0
            n_false_positives = 0
            for j in range(n_match_candidates):
                # Set the match candidate index, starting from the best descriptor distance and continuing to the worst
                candidate_index = candidate_indices_best_to_worst[j]

                # All match candidates with higher descriptor distances and references under the localization radius are
                # false negatives. These false negatives are summed with the false negatives from queries for which no
                # match candidate was assigned
                n_false_negatives_with_candidate = np.sum(ground_truth_physical_distances_sorted[j + 1:] <=
                    localization_radius)
                n_false_negatives = n_false_negatives_with_candidate + self.n_false_negatives_no_candidate[i]

                # If the physical distance is less than the localization radius, increment the number of true positives
                if self.match_candidates_estimate['physical_distances'][candidate_index] <= localization_radius:
                    n_true_positives += 1
                # Otherwise, increment the number of false positives
                else:
                    n_false_positives += 1

                # Compute precision and recall for this descriptor distance
                self.precisions[i, j] = n_true_positives / (n_true_positives + n_false_positives)
                self.recalls[i, j] = n_true_positives / (n_true_positives + n_false_negatives)

                # Store the maximum recall at 100% precision and the descriptor distance threshold that achieves it
                if self.precisions[i, j] == 1:
                    self.max_recalls_perfect_precision[i] = self.recalls[i, j]
                    self.distance_thresholds_perfect_precison[i] = \
                        self.match_candidates_estimate['distances'][candidate_index]

                # Store the number of false positives at the maximum descriptor distance threshold
                if j == n_match_candidates - 1:
                    self.n_false_positives_max_thresh.append(n_false_positives)

            # Compute the area under the PR curve
            # NOTE: when computing the AUC we add a point at recall = 0 and precision = 1 so the curve begins on the
            # y-axis. This point is also added in sklearn.metrics.precision_recall_curve as documented here:
            # https://scikit-learn.org/stable/modules/generated/sklearn.metrics.precision_recall_curve.html
            self.aucs.append(np.trapz(np.hstack((1, self.precisions[i, :])), np.hstack((0, self.recalls[i, :]))))

    def add_image(self, image, tag, label_plot, height, width, equal_aspects, parent, type, label_x=None, label_y=None):
        """Helper function for adding an image to the dearpygui visualization.

        Args:
            image: Numpy array of image data. If the type is 'heat' this should be 2D (rows x cols), otherwise this
                should be 3D (rows x cols x channels) in RGBA format.
            tag: Image or heat series tag.
            label_plot: Plot lable (title).
            height: Plot height.
            width: Plot width.
            equal_aspects: Whether the x and y axes should be constrained to have the same units/pixel.
            parent: Plot parent.
            type: Image type ('static', 'dynamic', or 'heat').
            label_x: x-axis label. If None, the x-axis will have no label, tick marks, or tick values. Defaults to None.
            label_y: y-axis label. if None, the y-axis will have no label, tick marks, or tick values. Defaults to None.
        """

        no_ticks_x = label_x == None
        no_ticks_y = label_y == None
        if type == 'static' or type=='dynamic':
            with dpg.texture_registry():
                if type=='static':
                    dpg.add_static_texture(image.shape[1], image.shape[0], image.flatten(), tag=tag)
                elif type=='dynamic':
                    dpg.add_dynamic_texture(image.shape[1], image.shape[0], image.flatten(), tag=tag)
        with dpg.plot(label=label_plot, height=height, width=width, equal_aspects=equal_aspects, parent=parent):
            if type=='heat':
                dpg.bind_colormap(dpg.last_item(), dpg.mvPlotColormap_Plasma)
            dpg.add_plot_axis(dpg.mvXAxis, label=label_x, no_gridlines=True, no_tick_labels=no_ticks_x,
                no_tick_marks=no_ticks_x)
            with dpg.plot_axis(dpg.mvYAxis, label=label_y, no_gridlines=True, no_tick_labels=no_ticks_y,
                no_tick_marks=no_ticks_y):
                if type=='heat':
                    dpg.add_heat_series(image, image.shape[0], image.shape[1], format="", scale_min=0, scale_max=1,
                        tag=tag)
                else:
                    dpg.add_image_series(tag, [0, 0], [image.shape[1], image.shape[0]])

    def add_plot(self, elements, label_plot, label_x, label_y, parent, legend=True, limits_x=None, limits_y=None,
        height=-1, width=-1):
        """Helper function for adding a line/shade/scatter plot to the dearpygui visualization.

        Args:
            elements: List of dictionaries, each defining an element to add to the plot. The dictionaries must contain
                the following keys:
                tag: Element tag.
                label: Element label.
                color: Element color.
                type: Element type ('line', 'shade', 'scatter', or 'cross'). 'cross' is a scatter plot with large cross
                    symbols instead of dots.
                x: x-data.
                y: y-data.
            label_plot: Plot label (title).
            label_x: x-axis label.
            label_y: y-axis label.
            parent: Plot parent.
            legend: Whether to add a legend to the plot. Defaults to True.
            limits_x: A two element list [min, max] specifying the x-axis limits. If None, no x-axis limits will be set.
                Defaults to None.
            limits_y: A two element list [min, max] specifying the y-axis limits. If None, no y-axis limits will be set.
                Defaults to None.
            height: Plot height. Defaults to -1.
            width: Plot width. Defaults to -1.
        """

        with dpg.plot(label=label_plot, height=height, width=width, parent=parent) as plot:
            dpg.add_plot_axis(dpg.mvXAxis, label=label_x)
            if limits_x is not None:
                dpg.set_axis_limits(dpg.last_item(), limits_x[0], limits_x[1])
            axis_y = dpg.add_plot_axis(dpg.mvYAxis, label=label_y)
            if limits_y is not None:
                dpg.set_axis_limits(dpg.last_item(), limits_y[0], limits_y[1])

        for element in elements:
            with dpg.theme() as theme:
                if element['type'] == 'line':
                    with dpg.theme_component(dpg.mvLineSeries):
                        dpg.add_theme_color(dpg.mvPlotCol_Line, element['color'], category=dpg.mvThemeCat_Plots)
                elif element['type'] == 'shade':
                    with dpg.theme_component(dpg.mvShadeSeries):
                        dpg.add_theme_color(dpg.mvPlotCol_Fill, element['color'], category=dpg.mvThemeCat_Plots)
                elif element['type'] == 'scatter' or element['type'] == 'cross':
                    with dpg.theme_component(dpg.mvScatterSeries):
                        dpg.add_theme_color(dpg.mvPlotCol_MarkerFill, element['color'], category=dpg.mvThemeCat_Plots)
                        dpg.add_theme_color(dpg.mvPlotCol_MarkerOutline, element['color'],
                            category=dpg.mvThemeCat_Plots)
                        if element['type'] == 'cross':
                            dpg.add_theme_style(dpg.mvPlotStyleVar_Marker, dpg.mvPlotMarker_Cross,
                                category=dpg.mvThemeCat_Plots)
                            dpg.add_theme_style(dpg.mvPlotStyleVar_MarkerSize, 40,
                                category=dpg.mvThemeCat_Plots)
            if element['type'] == 'line':
                dpg.add_line_series(element['x'], element['y'], tag=element['tag'], label=element['label'],
                    parent=axis_y)
            elif element['type'] == 'shade':
                dpg.add_shade_series(element['x'], element['y'], tag=element['tag'], label=element['label'],
                    parent=axis_y)
            elif element['type'] == 'scatter' or element['type'] == 'cross':
                dpg.add_scatter_series(element['x'], element['y'], tag=element['tag'], label=element['label'],
                    parent=axis_y)
            dpg.bind_item_theme(dpg.last_item(), theme)

        if legend:
            dpg.add_plot_legend(location=dpg.mvPlot_Location_North, outside=True, parent=plot)

    def format_descriptor(self, descriptor):
        """Helper function to format a descriptor for visualization.

        Args:
            descriptor: 1D numpy array of raw descriptor data.

        Returns:
            The reshaped and normalized descriptor.
        """

        descriptor = np.flipud(descriptor.reshape(self.n_descriptor_rows, self.n_descriptor_cols))
        descriptor = (descriptor - self.min_descriptor_value) / (self.max_descriptor_value - self.min_descriptor_value)

        return descriptor

    def format_image(self, image):
        """Helper function to format an image for visualization.

        Args:
            descriptor: Numpy array of raw image data.

        Returns:
            Image as a 3D numpy array (rows x cols x channels) in RGBA format.
        """
        if self.encoding == 'bayer_rggb8': # RGB
            image = cv2.cvtColor(image, cv2.COLOR_BayerBG2RGB)
            image = image / 255
            image = np.concatenate((image, np.ones((image.shape[0], image.shape[1], 1))), axis=2)
        elif self.encoding == 'mono16': # Thermal
            threshold_minimum = 22500
            threshold_maximum = 25000
            image[image < threshold_minimum] = threshold_minimum
            image[image > threshold_maximum] = threshold_maximum
            image = (image - threshold_minimum) / (threshold_maximum - threshold_minimum)
            image = np.stack((image, image, image, np.ones(image.shape)), axis=2)
        elif self.encoding == 'mono8': # Monochrome
            image = image / 255
            image = np.stack((image, image, image, np.ones(image.shape)), axis=2)

        return image

    def update_figure(self, sender):
        """Slider callback function to update visualization.

        Args:
            sender: Slider tag.
        """

        if sender == 'slider_localization_radius':
            # Find the nearest localization radius and snap the slider to that value
            index_radius = np.argmin(np.abs(np.asarray(self.localization_radii) - dpg.get_value(sender)))
            localization_radius = self.localization_radii[index_radius]
            dpg.set_value(sender, localization_radius)

            # Update the precision-recall plot
            # NOTE: the third element in the shade series value tuple is the `y2` input. The filled in area is between
            # `y1` and `y2`. `y2` was not given when the shade series was created, so it defaulted to all zeros.
            dpg.set_value('line_precision_recall', (self.recalls[index_radius, :], self.precisions[index_radius, :]))
            dpg.set_value('shade_precision_recall', (self.recalls[index_radius, :], self.precisions[index_radius, :],
                np.zeros(self.precisions[index_radius, :].shape)))

            # Update the matches plot
            indices_false_positive = np.nonzero(np.asarray(self.match_candidates_estimate['physical_distances']) >
                    localization_radius)
            dpg.set_value('scatter_false_positive',
                (np.asarray(self.match_candidates_estimate['query_ids'])[indices_false_positive],
                np.asarray(self.match_candidates_estimate['reference_ids'])[indices_false_positive]))

            indices_true_positive = np.nonzero(np.asarray(self.match_candidates_estimate['physical_distances']) <=
                localization_radius)
            dpg.set_value('scatter_true_positive',
                (np.asarray(self.match_candidates_estimate['query_ids'])[indices_true_positive],
                np.asarray(self.match_candidates_estimate['reference_ids'])[indices_true_positive]))

            indices_true_positive_perfect_precision = np.nonzero(
                np.asarray(self.match_candidates_estimate['distances']) <=
                self.distance_thresholds_perfect_precison[index_radius])
            dpg.set_value('scatter_true_positive_perfect_precision',
                (np.asarray(self.match_candidates_estimate['query_ids'])[indices_true_positive_perfect_precision],
                np.asarray(self.match_candidates_estimate['reference_ids'])[indices_true_positive_perfect_precision]))

            # Update the trajectory plot
            indices_true_positive_reference = np.asarray(
                self.match_candidates_estimate['reference_ids'])[indices_true_positive]
            dpg.set_value('scatter_true_positive_trajectory',
                (self.positions_ground_truth_reference_enu[indices_true_positive_reference, 0],
                self.positions_ground_truth_reference_enu[indices_true_positive_reference, 1]))

            indices_true_positive_perfect_precision_reference = np.asarray(
                self.match_candidates_estimate['reference_ids'])[indices_true_positive_perfect_precision]
            dpg.set_value('scatter_true_positive_perfect_precision_trajectory',
                (self.positions_ground_truth_reference_enu[indices_true_positive_perfect_precision_reference, 0],
                self.positions_ground_truth_reference_enu[indices_true_positive_perfect_precision_reference, 1]))

            # Update statistics table
            values = [str(localization_radius), str(self.max_recalls_perfect_precision[index_radius]),
                str(self.aucs[index_radius]), str(self.distance_thresholds_perfect_precison[index_radius])]
            for i, value in enumerate(values):
                dpg.set_value('value_cell_' + str(i), value)

        if sender == 'slider_match_index':
            # Set the query index, match candidate reference index, and ground truth reference index
            index_match = dpg.get_value(sender)
            index_query = self.match_candidates_estimate['query_ids'][index_match]
            index_reference_estimate = self.match_candidates_estimate['reference_ids'][index_match]
            index_reference_ground_truth = self.match_candidates_ground_truth['reference_ids'][index_match]

            # Update the descriptors, if available
            # NOTE: set_value is currently not well documented for the heat series. It is unclear what the second
            # element in the heat series value tuple represents, but get_value indicates it should be set to [0, 1]
            if self.plot_descriptors:
                dpg.set_value('descriptor_query',
                    (self.format_descriptor(self.descriptors_query[index_query, :]), [0, 1]))
                dpg.set_value('descriptor_reference_estimate',
                    (self.format_descriptor(self.descriptors_reference[index_reference_estimate, :]), [0, 1]))
                dpg.set_value('descriptor_reference_ground_truth',
                    (self.format_descriptor(self.descriptors_reference[index_reference_ground_truth, :]), [0, 1]))

            # Update the images, if available
            if self.plot_images:
                dpg.set_value("image_query",
                    self.format_image(self.dataset_images_query[self.indices_images_query[index_query]]).flatten())
                dpg.set_value("image_reference_estimate",
                    self.format_image(self.dataset_images_reference[
                        self.indices_images_reference[index_reference_estimate]]).flatten())
                dpg.set_value("image_reference_ground_truth",
                    self.format_image(self.dataset_images_reference[
                        self.indices_images_reference[index_reference_ground_truth]]).flatten())

            # Update the matches plot
            dpg.set_value('scatter_selected_ground_truth', ([index_query], [index_reference_ground_truth]))
            dpg.set_value('scatter_selected_match_candidate', ([index_query], [index_reference_estimate]))

            # Update the trajectory plot
            dpg.set_value('scatter_selected_ground_truth_trajectory',
                ([self.positions_ground_truth_reference_enu[index_reference_ground_truth, 0]],
                [self.positions_ground_truth_reference_enu[index_reference_ground_truth, 1]]))
            dpg.set_value('scatter_selected_match_candidate_trajectory',
                ([self.positions_ground_truth_reference_enu[index_reference_estimate, 0]],
                [self.positions_ground_truth_reference_enu[index_reference_estimate, 1]]))

            # Update statistics table
            values = [str(index_match), str(index_query), str(index_reference_estimate),
                str(self.match_candidates_estimate['distances'][index_match]),
                str(self.match_candidates_estimate['physical_distances'][index_match]),
                str(index_reference_ground_truth),
                str(self.match_candidates_ground_truth['physical_distances'][index_match])]
            for i, value in enumerate(values):
                dpg.set_value('value_cell_' + str(i + 4), value)

    def step_match_index(self, sender):
        """Key press callback function to enable arrow keys to increment/decrement the match index slider.

        Args:
            sender: Key press handler tag.
        """

        step = 1 if sender == 'key_increment' else -1
        value_slider = dpg.get_value('slider_match_index') + step
        value_slider = 0 if value_slider < 0 else value_slider
        value_slider_max = dpg.get_item_configuration('slider_match_index')['max_value']
        value_slider = value_slider_max if value_slider > value_slider_max else value_slider
        dpg.set_value('slider_match_index', value_slider)
        self.update_figure('slider_match_index')

    def visualize_results(self, path_distance_matrix=None, path_images_query=None, path_images_reference=None,
        path_descriptors_query=None, path_descriptors_reference=None, n_descriptor_rows=None, n_descriptor_cols=None,
        title='Multi-Session Place Recognition Results'):
        """Visualizes place recognition results.

        Args:
            path_distance_matrix: A filepath to the distance matrix txt file. If given, the distance matrix will be
                displayed.
            path_images_query: A filepath to the query image H5 file. If given along with path_images_reference, the
                images will be displayed.
            path_images_reference: A filepath to the reference image H5 file. If given along with path_images_query,
                the images will be displayed.
            path_descriptors_query: A filepath to the query descriptor txt file. If given along with
                path_descriptors_reference, n_descriptor_rows, and n_descriptor_cols, the descriptors will be displayed.
            path_descriptors_reference: A filepath to the reference descriptor txt file. If given along with
                path_descriptors_query, n_descriptor_rows, and n_descriptor_cols, the descriptors will be displayed.
            n_descriptor_rows: Descriptor height. If given along with path_descriptors_query,
                path_descriptors_reference, and n_descriptor_cols, the descriptors will be displayed.
            n_descriptor_cols: Descriptor width. If given along with path_descriptors_query,
                path_descriptors_reference, and n_descriptor_rows, the descriptors will be displayed.
            title: Title of the display window.
        """

        # Only plot images if both H5 files have been specified
        self.plot_images = path_images_query and path_images_reference

        # Only plot descriptors if all necessary inputs have been specified
        self.plot_descriptors = path_descriptors_query and path_descriptors_reference and n_descriptor_rows and \
            n_descriptor_cols

        # Set the figure size to roughly match the resolution of the smallest available monitor
        width_viewport = 1824
        height_viewpoint = 1026
        try:
            min_width_monitor = 1e6
            for monitor in get_monitors():
                if monitor.width < min_width_monitor:
                    min_width_monitor = monitor.width
                    width_viewport = monitor.width
                    height_viewpoint = monitor.height
            width_viewport = int(width_viewport * 0.95) # Account for unusable screen space
            height_viewpoint = int(height_viewpoint * 0.95)
        except:
            pass

        # Create the primary window and tabs
        dpg.create_context()
        with dpg.window(tag='primary_window'):
            with dpg.tab_bar():
                tab_interactive_plot = dpg.add_tab(label='Interactive Plots')
                tab_metrics_across_radii = dpg.add_tab(label='Metrics Across Localization Radii')

        # Create the child windows within the interactive plot tab, handle all variations of available information
        height_row_0 = int(height_viewpoint * 0.50)
        height_row_1 = int(height_viewpoint * 0.40)
        if self.plot_images and self.plot_descriptors:
            height_row_0 = int(height_viewpoint * 0.40)
            height_row_1 = int(height_viewpoint * 0.50)

        width_trajectory_window = int(width_viewport * 0.25) if path_distance_matrix else int(width_viewport * 0.5)
        with dpg.group(horizontal=True, parent=tab_interactive_plot):
            # Create the child windows in the first row
            window_precision_recall = dpg.add_child_window(width=int(width_viewport * 0.25), height=height_row_0)
            window_trajectory = dpg.add_child_window(width=width_trajectory_window, height=height_row_0)
            window_matches = dpg.add_child_window(width=int(width_viewport * 0.25), height=height_row_0)
            if path_distance_matrix:
                window_distance_matrix = dpg.add_child_window(width=int(width_viewport * 0.25), height=height_row_0)

        width_descriptors_window = int(width_viewport * 0.5045) if self.plot_images else int(width_viewport * 0.625)
        height_descriptors_window = int(height_row_1 * 0.498) if self.plot_images else height_row_1
        width_images_window = int(width_viewport * 0.5045) if self.plot_descriptors else int(width_viewport * 0.625)
        height_images_window = int(height_row_1 * 0.498) if self.plot_descriptors else height_row_1
        width_statistics_window = -1
        if self.plot_images or self.plot_descriptors:
            width_statistics_window = int(width_viewport * 0.5) if self.plot_images and self.plot_descriptors else \
                int(width_viewport * 0.375)
        with dpg.group(horizontal=True, parent=tab_interactive_plot):
            # Create the child windows in the second row
            with dpg.group():
                if self.plot_descriptors:
                    window_descriptors = dpg.add_child_window(width=width_descriptors_window,
                        height=height_descriptors_window)
                if self.plot_images:
                    window_images = dpg.add_child_window(width=width_images_window, height=height_images_window)
            window_statistics = dpg.add_child_window(width=width_statistics_window, height=height_row_1)

        window_sliders = dpg.add_child_window(autosize_x=True, autosize_y=True, parent=tab_interactive_plot)

        # Plot the distance matrix, if the filepath was provided
        if path_distance_matrix:
            # Load the distance matrix
            distance_matrix = np.loadtxt(path_distance_matrix)

            # Convert to heatmap
            # NOTE: the built-in dearpygui heat_series is slow for large matrices so we convert the matrix to a heat
            # map ourselves and display it as an image (static_texture)
            max_distance_matrix = np.max(distance_matrix[distance_matrix != np.inf])
            distance_matrix[distance_matrix == np.inf] = max_distance_matrix
            distance_matrix_normalized = (distance_matrix - np.min(distance_matrix)) / \
                (np.max(distance_matrix) - np.min(distance_matrix))
            heatmap = cm.plasma(distance_matrix_normalized)

            # Flip the image about the x-axis
            heatmap = np.flipud(heatmap)

            # Create the plot
            group_distance_matrix = dpg.add_group(horizontal=True, parent=window_distance_matrix)
            self.add_image(heatmap, 'image_distance_matrix', 'Distance Matrix', -1,
                dpg.get_item_width(window_distance_matrix) - 80, False, group_distance_matrix, 'static',
                'Query Descriptor ID', 'Reference Descriptor ID')
            dpg.add_colormap_scale(min_scale=np.min(distance_matrix), max_scale=np.max(distance_matrix),
                colormap=dpg.mvPlotColormap_Plasma, height=-1, width=50, parent=group_distance_matrix)

        # Set the localization radius index and value
        index_radius = np.argmax(self.localization_radii)
        localization_radius = self.localization_radii[index_radius]

        # Plot the precision-recall curve
        elements = []
        elements.append(dict(tag='line_precision_recall', label=None, color=[0, 255, 255], type='line',
            x=self.recalls[index_radius, :], y=self.precisions[index_radius, :]))
        elements.append(dict(tag='shade_precision_recall', label=None, color=[0, 255, 255, 50], type='shade',
            x=self.recalls[index_radius, :], y=self.precisions[index_radius, :]))
        self.add_plot(elements, 'Precision-Recall Curve', 'Recall', 'Precision', window_precision_recall, False, [0, 1],
            [0, 1.01])

        # Set the query index, match candidate reference index, and ground truth reference index
        index_match = 0
        index_query = self.match_candidates_estimate['query_ids'][index_match]
        index_reference_estimate = self.match_candidates_estimate['reference_ids'][index_match]
        index_reference_ground_truth = self.match_candidates_ground_truth['reference_ids'][index_match]

        # Plot the descriptors, if available
        if self.plot_descriptors:
            self.n_descriptor_rows = n_descriptor_rows
            self.n_descriptor_cols = n_descriptor_cols

            # Import descriptors
            self.descriptors_query = np.loadtxt(path_descriptors_query)
            self.descriptors_reference = np.loadtxt(path_descriptors_reference)
            self.min_descriptor_value = np.min(self.descriptors_query) if \
                np.min(self.descriptors_query) < np.min(self.descriptors_reference) else \
                np.min(self.descriptors_reference)
            self.max_descriptor_value = np.max(self.descriptors_query) if \
                np.max(self.descriptors_query) > np.max(self.descriptors_reference) else \
                np.max(self.descriptors_reference)

            # Create the plots
            group_descriptors = dpg.add_group(horizontal=True, parent=window_descriptors)
            common_arguments_descriptor = dict(height=-1, width=int(width_descriptors_window * 0.33),
                equal_aspects=True, parent=group_descriptors, type='heat')
            self.add_image(self.format_descriptor(self.descriptors_query[index_query, :]), 'descriptor_query',
                'Query Descriptor', **common_arguments_descriptor)
            self.add_image(self.format_descriptor(self.descriptors_reference[index_reference_estimate, :]),
                'descriptor_reference_estimate', 'Candidate Reference Descriptor', **common_arguments_descriptor)
            self.add_image(self.format_descriptor(self.descriptors_reference[index_reference_ground_truth, :]),
                'descriptor_reference_ground_truth', 'Ground Truth Reference Descriptor', **common_arguments_descriptor)

        # Plot the images, if available
        if self.plot_images:
            # Import images and their corresponding timestamps
            h5_file_query = h5py.File(path_images_query, 'r')
            self.dataset_images_query = h5_file_query['image_raw/images']
            timestamps_images_query = h5_file_query['image_raw/timestamps'][:] * 1e-9
            h5_file_reference = h5py.File(path_images_reference, 'r')
            self.dataset_images_reference = h5_file_reference['image_raw/images']
            timestamps_images_reference = h5_file_reference['image_raw/timestamps'][:] * 1e-9

            # Check encoding matches
            if h5_file_query['image_raw'].attrs['encoding'] != h5_file_reference['image_raw'].attrs['encoding']:
                print('The image encoding does not match across the two input H5 files')
                exit()
            self.encoding = h5_file_query['image_raw'].attrs['encoding']

            # Determine H5 image indices for each place descriptor index
            self.indices_images_query = [np.argmin(np.abs(timestamp - timestamps_images_query)) for timestamp in
                self.timestamps_place_descriptors_query]
            self.indices_images_reference = [np.argmin(np.abs(timestamp - timestamps_images_reference)) for timestamp in
                self.timestamps_place_descriptors_reference]

            # Create the plots
            group_images = dpg.add_group(horizontal=True, parent=window_images)
            common_arguments_image = dict(height=-1, width=int(width_images_window * 0.33), equal_aspects=True,
                parent=group_images, type='dynamic')
            self.add_image(self.format_image(self.dataset_images_query[self.indices_images_query[index_query]]),
                'image_query', 'Query Image', **common_arguments_image)
            self.add_image(self.format_image(
                self.dataset_images_reference[self.indices_images_reference[index_reference_estimate]]),
                'image_reference_estimate', 'Candidate Reference Image', **common_arguments_image)
            self.add_image(self.format_image(
                self.dataset_images_reference[self.indices_images_reference[index_reference_ground_truth]]),
                'image_reference_ground_truth', 'Ground Truth Reference Image', **common_arguments_image)

        # Plot the candidate and ground truth matches
        elements = []
        # Ground truth matches
        elements.append(dict(tag='scatter_ground_truth', label='Ground Truth', color=[255, 0, 255], type='scatter',
            x=self.match_candidates_ground_truth['query_ids'], y=self.match_candidates_ground_truth['reference_ids']))
        # False positive matches (when the descriptor distance threshold is at its maximum)
        indices_false_positive = np.nonzero(np.asarray(self.match_candidates_estimate['physical_distances']) >
            localization_radius)
        elements.append(dict(tag='scatter_false_positive', label='Incorrect Matches', color=[255, 0, 0], type='scatter',
            x=np.asarray(self.match_candidates_estimate['query_ids'])[indices_false_positive],
            y=np.asarray(self.match_candidates_estimate['reference_ids'])[indices_false_positive]))
        # True positive matches (when the descriptor distance threshold is at its maximum)
        indices_true_positive = np.nonzero(np.asarray(self.match_candidates_estimate['physical_distances']) <=
            localization_radius)
        elements.append(dict(tag='scatter_true_positive', label='Correct Matches', color=[0, 0, 255], type='scatter',
            x=np.asarray(self.match_candidates_estimate['query_ids'])[indices_true_positive],
            y=np.asarray(self.match_candidates_estimate['reference_ids'])[indices_true_positive]))
        # True positive matches with descriptor distances beneath the 100% precision cutoff, i.e. the matches
        # obtainable without any false positives
        indices_true_positive_perfect_precision = np.nonzero(np.asarray(self.match_candidates_estimate['distances']) <=
            self.distance_thresholds_perfect_precison[index_radius])
        elements.append(dict(tag='scatter_true_positive_perfect_precision', label='Correct Matches 100% Prec.',
            color=[0, 255, 0], type='scatter',
            x=np.asarray(self.match_candidates_estimate['query_ids'])[indices_true_positive_perfect_precision],
            y=np.asarray(self.match_candidates_estimate['reference_ids'])[indices_true_positive_perfect_precision]))
        # Currently selected ground truth match
        elements.append(dict(tag='scatter_selected_ground_truth', label='Selected Ground Truth (X symbol)',
            color=[0, 255, 255], type='cross', x=[index_query], y=[index_reference_ground_truth]))
        # Currently selected match candidate
        elements.append(dict(tag='scatter_selected_match_candidate', label='Selected Match Candidate (X symbol)',
            color=[255, 255, 0], type='cross', x=[index_query], y=[index_reference_estimate]))

        self.add_plot(elements, 'Matches', 'Query ID', 'Reference ID', window_matches)

        # Convert ground truth positions from ECEF to ENU
        position_center_ecef = np.mean(self.positions_ground_truth_query, axis=0, keepdims=True).T
        info_frame_baselink = pygeodesy.ecef.EcefKarney(pygeodesy.datums.Datums.WGS84).reverse(position_center_ecef[0],
            position_center_ecef[1], position_center_ecef[2], M=True)
        rotation_enu_to_ecef = np.array([info_frame_baselink.M.row(0), info_frame_baselink.M.row(1),
            info_frame_baselink.M.row(2)])
        self.positions_ground_truth_query_enu = rotation_enu_to_ecef.T.dot(
            self.positions_ground_truth_query.T - position_center_ecef).T
        self.positions_ground_truth_reference_enu = rotation_enu_to_ecef.T.dot(
            self.positions_ground_truth_reference.T - position_center_ecef).T

        # Plot match candidates along trajectory
        elements = []
        # Query trajectory
        elements.append(dict(tag='line_query_trajectory', label='Query Trajectory', color=[255, 0, 255], type='line',
            x=self.positions_ground_truth_query_enu[:, 0], y=self.positions_ground_truth_query_enu[:, 1]))
        # Reference trajectory
        elements.append(dict(tag='line_reference_trajectory', label='Reference Trajectory', color=[255, 0, 0],
            type='line', x=self.positions_ground_truth_reference_enu[:, 0],
            y=self.positions_ground_truth_reference_enu[:, 1]))
        # True positive matches (when the descriptor distance threshold is at its maximum)
        indices_true_positive_query = np.asarray(
            self.match_candidates_estimate['query_ids'])[indices_true_positive]
        elements.append(dict(tag='scatter_true_positive_trajectory', label='Correct Matchess', color=[0, 0, 255],
            type='scatter', x=self.positions_ground_truth_query_enu[indices_true_positive_query, 0],
            y=self.positions_ground_truth_query_enu[indices_true_positive_query, 1]))
        # True positive matches with descriptor distances beneath the 100% precision cutoff, i.e. the
        # matches obtainable without any false positives
        indices_true_positive_perfect_precision_query = np.asarray(
            self.match_candidates_estimate['query_ids'])[indices_true_positive_perfect_precision]
        elements.append(dict(tag='scatter_true_positive_perfect_precision_trajectory', label='Correct Matches 100% Prec.',
            color=[0, 255, 0], type='scatter',
            x=self.positions_ground_truth_query_enu[indices_true_positive_perfect_precision_query, 0],
            y=self.positions_ground_truth_query_enu[indices_true_positive_perfect_precision_query, 1]))
        # Currently selected ground truth match
        elements.append(dict(tag='scatter_selected_ground_truth_trajectory', label='Selected Ground Truth (X symbol)',
            color=[0, 255, 255], type='cross',
            x=[self.positions_ground_truth_reference_enu[index_reference_ground_truth, 0]],
            y=[self.positions_ground_truth_reference_enu[index_reference_ground_truth, 1]]))
        # Currently selected match candidate
        elements.append(dict(tag='scatter_selected_match_candidate_trajectory',
            label='Selected Match Candidate (X symbol)', color=[255, 255, 0], type='cross',
            x=[self.positions_ground_truth_reference_enu[index_reference_estimate, 0]],
            y=[self.positions_ground_truth_reference_enu[index_reference_estimate, 1]]))

        self.add_plot(elements, 'Trajectory', 'East (m)', 'North (m)', window_trajectory)

        # Create statistics table
        with dpg.table(header_row=True, borders_innerH=True, borders_outerH=True, borders_innerV=True,
            borders_outerV=True, row_background=True, freeze_rows=1, freeze_columns=1, scrollY=True,
            scrollX=True, policy=dpg.mvTable_SizingFixedFit, parent=window_statistics, resizable=True):

            # Compile table data
            rows = []
            rows.append(('Localization Radius (meters)', str(localization_radius)))
            rows.append(('Maximum Recall at 100% Precision', str(self.max_recalls_perfect_precision[index_radius])))
            rows.append(('AUC', str(self.aucs[index_radius])))
            rows.append(('100% Precision Descriptor Distance Threshold',
                str(self.distance_thresholds_perfect_precison[index_radius])))
            rows.append(('Match Index', '0'))
            rows.append(('Query Index', str(index_query)))
            rows.append(('Candidate Reference Index', str(index_reference_estimate)))
            rows.append(('Candidate Reference Descriptor Distance',
                str(self.match_candidates_estimate['distances'][index_match])))
            rows.append(('Candidate Reference Physical Distance (meters)',
                str(self.match_candidates_estimate['physical_distances'][index_match])))
            rows.append(('Ground Truth Reference Index', str(index_reference_ground_truth)))
            rows.append(('Ground Truth Reference Physical Distance (meters)',
                str(self.match_candidates_ground_truth['physical_distances'][index_match])))

            # Write out table data
            dpg.add_table_column(label="Name")
            dpg.add_table_column(label="Value")
            for i, row in enumerate(rows):
                with dpg.table_row():
                    dpg.add_text(row[0])
                    dpg.add_text(row[1], tag='value_cell_' + str(i))

        # Create sliders
        with dpg.group(horizontal=True, parent=window_sliders):
            # Create a localization radius slider
            dpg.add_slider_float(label='Localization Radius', callback=self.update_figure, clamped=True,
                min_value=np.min(self.localization_radii), max_value=np.max(self.localization_radii),
                default_value=np.max(self.localization_radii), height=-1, width=int(width_viewport * 0.25),
                tag='slider_localization_radius')

            # Create match candidate index slider
            dpg.add_slider_int(label='Match Index', callback=self.update_figure, clamped=True, min_value=0,
                max_value=len(self.match_candidates_estimate['query_ids']) - 1, default_value=0, height=-1,
                width=int(width_viewport * 0.62), tag='slider_match_index')

            # Create key callbacks to allow stepping through match candidates
            with dpg.handler_registry():
                dpg.add_key_press_handler(dpg.mvKey_Right, callback=self.step_match_index, tag='key_increment')
                dpg.add_key_press_handler(dpg.mvKey_Left, callback=self.step_match_index, tag='key_decrement')

        # Create non-interactive plots in the second tab
        group_metrics_across_radii = dpg.add_group(horizontal=True, parent=tab_metrics_across_radii)
        elements = [dict(tag='line_max_recall', label=None, color=[0, 255, 255], type='line',
            x=self.localization_radii, y=self.max_recalls_perfect_precision)]
        self.add_plot(elements, None, 'Localization Radius (meters)', 'Maximum Recall at 100% Precision',
            group_metrics_across_radii, False, None, [-0.01, 1], width=int(width_viewport * 0.49))
        elements = [dict(tag='line_auc', label=None, color=[0, 255, 255], type='line',
            x=self.localization_radii, y=self.aucs)]
        self.add_plot(elements, None, 'Localization Radius (meters)', 'AUC', group_metrics_across_radii, False, None,
            [-0.01, 1], width=int(width_viewport * 0.49))

        dpg.create_viewport(title=title, width=width_viewport,
            height=height_viewpoint)
        dpg.setup_dearpygui()
        dpg.show_viewport()
        dpg.set_primary_window('primary_window', True)
        dpg.start_dearpygui()
        dpg.destroy_context()

if __name__ == "__main__":
    # Process arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('path_timestamps_query', type=str, help='Path to query timestamp txt file')
    parser.add_argument('path_timestamps_reference', type=str, help='Path to reference timestamp txt file')
    parser.add_argument('path_ground_truth_query', type=str, help='Path to query ground truth H5 file')
    parser.add_argument('path_ground_truth_reference', type=str, help='Path to reference ground truth H5 file')
    parser.add_argument('path_measured_extrinsics_query', type=str, help='Path to query measured extrinsics YAML file')
    parser.add_argument('path_measured_extrinsics_reference', type=str, help='Path to reference measured extrinsics '
        'YAML file')
    parser.add_argument('namespace_query', type=str, help='Query sensor namespace (e.g. mono_left)')
    parser.add_argument('namespace_reference', type=str, help='Reference sensor namespace (e.g. mono_left)')
    parser.add_argument('path_match_candidates', type=str, help='Path to match candidate JSON file')
    parser.add_argument('--localization-radii', nargs='+', type=int, default=[5, 30, 1],
        help='Localization radii in meters specified with three integers as a list [start, stop, step]. A list can '
        'given from the command line with elements separated by spaces (default: [5 30 1])')
    parser.add_argument('--path-distance-matrix', type=str, default=None,
        help='Path to distance matrix txt file. If given, the distance matrix will be displayed.')
    parser.add_argument('--path-images-query', type=str, default=None,
        help='Path to query image H5 file. If given along with --path-images-reference, the images will be displayed.')
    parser.add_argument('--path-images-reference', type=str, default=None,
        help='Path to reference image H5 file. If given along with --path-images-query, the images will be displayed.')
    parser.add_argument('--path-descriptors-query', type=str, default=None,
        help='Path to query descriptor txt file. If given along with --path-descriptors-reference, --n-descriptor-rows '
        ', and --n-descriptor-cols, the descriptors will be displayed.')
    parser.add_argument('--path-descriptors-reference', type=str, default=None,
        help='Path to reference descriptor txt file. If given along with --path-descriptors-query, --n-descriptor-rows '
        ', and --n-descriptor-cols, the descriptors will be displayed.')
    parser.add_argument('--n-descriptor-rows', type=int, default=None,
        help='Descriptor height. If given along with --path-descriptors-query, --path-descriptors-reference '
        ', and --n-descriptor-cols, the descriptors will be displayed.')
    parser.add_argument('--n-descriptor-cols', type=int, default=None,
        help='Descriptor width. If given along with --path-descriptors-query, --path-descriptors-reference '
        ', and --n-descriptor-rows, the descriptors will be displayed.')

    args = parser.parse_args()

    localization_radii = list(range(args.localization_radii[0], args.localization_radii[1] + 1,
        args.localization_radii[2]))

    # Compute place recognition metrics
    place_recognition_evaluator = PlaceRecognitionEvaluator(args.path_timestamps_query, args.path_timestamps_reference,
        args.path_ground_truth_query, args.path_ground_truth_reference, args.path_measured_extrinsics_query,
        args.path_measured_extrinsics_reference, args.namespace_query, args.namespace_reference,
        args.path_match_candidates, localization_radii)

    # Visualize results
    place_recognition_evaluator.visualize_results(args.path_distance_matrix, args.path_images_query,
        args.path_images_reference, args.path_descriptors_query, args.path_descriptors_reference,
        args.n_descriptor_rows, args.n_descriptor_cols)
