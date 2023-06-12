# Data Format

All sensor data is provided in the [HDF5](https://www.hdfgroup.org/solutions/hdf5/) file format (abbreviated H5). For an introduction to the H5 file format, we recommend [this video series](https://youtube.com/playlist?list=PLPyhR4PdEeGYWHRhzmCP5stzfIha8bqVg).

Extrinsic and intrinsic calibration information is provided as [YAML](https://yaml.org/) files.

A summary of the files provided for each sequence is given in the table below:

| Filename                                                          | Description                  | Sensor Model        |
|-------------------------------------------------------------------|------------------------------|---------------------|
| `<sequence_prefix>_adk_left.h5`                                   | Left Uncooled Thermal        | FLIR ADK USB-C      |
| `<sequence_prefix>_adk_right.h5`                                  | Right Uncooled Thermal       | FLIR ADK USB-C      |
| `<sequence_prefix>_mono_left.h5`                                  | Left Monochrome              | FLIR BFS-PGE-16S2M  |
| `<sequence_prefix>_mono_right.h5`                                 | Right Monochrome             | FLIR BFS-PGE-16S2M  |
| `<sequence_prefix>_rgb_left.h5`                                   | Left RGB                     | FLIR BFS-PGE-50S5C  |
| `<sequence_prefix>_rgb_right.h5`                                  | Right RGB                    | FLIR BFS-PGE-50S5C  |
| `<sequence_prefix>_dvxplorer_left.h5`                             | Left Event (DVS)             | Inivation DVXplorer |
| `<sequence_prefix>_dvxplorer_right.h5`                            | Right Event (DVS)            | Inivation DVXplorer |
| `<sequence_prefix>_applanix_right.h5`                             | Ground Truth Pose            | Applanix POS LV 420 |
| `<sequence_prefix>_<calibration_prefix>_calibration_results.yaml` | Camera Calibration Results   | NA                  |
| `<sequence_prefix>_<measured_prefix>_measured_extrinsics.yaml`    | Manually Measured Extrinsics | NA                  |

The `<sequence_prefix>` is a unique identifier for the sequence, e.g. `R0_FA0`, which contains the following components:
- `R<#>`: Route index, denoting the route driven, e.g. `R0`
- `<direction_symbol><lighting_condition_symbol><subcategory_index>`: Descriptor, e.g. `FA0`, with the following subcomponents:
    - `<direction_symbol>`: Direction symbol, denoting the direction driven along the route, which takes the following values:
        - `F`: Forward direction
        - `R`: Reverse direction
        - `D`: Forward direction with divergences from the route
    - `<lighting_condition_symbol>`: Lighting condition symbol, denoting the lighting condition the route was driven under, which takes the following values:
        - `A`: Afternoon
        - `S`: Sunset
        - `N`: Night
    - `<subcategory_index>`: Subcategory index, used to denote multiple sequences driven on the same route, in the same direction, and under the same lighting condition, i.e. an index that counts within the subcategory defined by the route index, direction symbol, and lighting condition symbol

> Note: the initial uploaded sample sequences are additionally denoted with `_sample` in the `<sequence_prefix>`.

The `<calibration_prefix>`, e.g. `C0`, and `<measured_prefix>`, e.g. `M0`, contain indices denoting which calibration sequence / measurement session the values were derived from.

## 1. General Sensor Data H5 Format

All sensor data was originally collected as a ROS1 rosbag and was converted to H5 files using the following strategy:
- An H5 file is created for each topic's namespace with filename `<sequence_prefix>_<namespace>`, with forward slashes replaced by underscores in nested namespaces (e.g. `/mono_left/image_raw` is written to `<sequence_prefix>_mono_left.h5` and `/adk/left/image_raw` is written to `<sequence_prefix>_adk_left.h5`).
- If available, sensor configuration information is attached to the H5 root group `/` as H5 scalar attributes.
- H5 groups are created for each topic's name (e.g. the topic `/mono_left/image_raw` is written to the H5 group `/image_raw` in the `<sequence_prefix>_mono_left.h5` file) and the message type of the topic is attached as a H5 scalar attribute to this group (e.g. the H5 group `/image_raw` has an H5 attribute named `ros_message_type` with value `sensor_msgs/Image`).
- Constant message fields are written as H5 scalar attributes attached to their corresponding H5 group (e.g. the `height` field of a `sensor_msgs/Image` message on the topic `/mono_left/image_raw` is written as an H5 attribute named `height` attached to the `/image_raw` H5 group).
- Variable message fields are written as H5 datasets in their corresponding H5 groups using descriptive names that do not necessarily match the message field name (e.g. the `data` field of a `sensor_msgs/Image` message on the topic `/mono_left/image_raw` is written to the H5 dataset `/image_raw/images`).
- Timestamps are written in nanoseconds as unsigned 64-bit integer H5 datasets.
- Where applicable, units or descriptors are written as H5 scalar attributes attached to H5 datasets (e.g. the H5 dataset `/image_raw/timestamps` will have an attribute named `units` with value `nanoseconds`).
- The first dimension of each H5 dataset indexes across messages and chunking is performed along the first dimension only. Additionally, any two chunked datasets from the same message have the same chunk size along the first dimension such that data from all message fields can be read in as full chunks.

We recommend using the [HDFView GUI](https://www.hdfgroup.org/downloads/hdfview/) to view the contents of the H5 files. Commandline tools `h5ls` and `h5dump` can also be used print the contents of the H5 files.

## 2. Sensor Specific Data Format and Characteristics

### 2.1. Uncooled Thermal

The uncooled thermal camera data files, `*_adk_left.h5` and `*_adk_right.h5`, contain the following:
- `image_raw`: A group, derived from [sensor_msgs/Image](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html) messages, including the following:
    - `height`, `width`, `encoding`, `is_bigendian`, `step`: Attributes for constant messages fields.
    - `images`: A three dimensional dataset (messages x rows x columns) of raw (distorted) thermal images. Note that the firmware version we use with our ADK cameras embeds telemetry information in the first four pixels of the first row in the image. After extracting this information we write the value of the fifth pixel to the first four.
    - `timestamps`: A one dimensional dataset of TAI timestamps in nanoseconds.
- `image_meta`: A group, derived from `image_meta_msgs/ImageMeta` messages (a custom message type), including multiple one dimensional datasets of various image metadata parameters. In particular, this includes:
    - `driver_timestamps`: A one dimensional dataset of TAI timestamps in nanoseconds. Note that each timestamp matches exactly with the image the metadata corresponds to, but matches may not always exist due to some corresponding messages being dropped on one topic but not the other.
    - `ffc_flags`: A one dimensional dataset of hexadecimal codes where the least significant digit indicates whether a flat field correction (FFC) is not being performed (0x0000), imminent (0x0001), in progress (0x0002), or complete (0x0003). Note that the thermal images are not valid during a FFC.

### 2.2. Monochrome

The monochrome camera data files, `*_mono_left.h5` and `*_mono_right.h5`, contain the following:
- `image_raw`: A group, derived from [sensor_msgs/Image](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html) messages, including the following:
    - `height`, `width`, `encoding`, `is_bigendian`, `step`: Attributes for constant messages fields
    - `images`: A three dimensional dataset (messages x rows x columns) of raw (distorted) monochrome images.
    - `timestamps`: A one dimensional dataset of TAI timestamps in nanoseconds.
- `image_meta`: A group, derived from `image_meta_msgs/ImageMeta` messages (a custom message type), including multiple one dimensional datasets of various image metadata parameters. In particular, this includes:
    - `driver_timestamps`: A one dimensional dataset of TAI timestamps in nanoseconds. Note that each timestamp matches exactly with the image the metadata corresponds to, but matches may not always exist due to some corresponding messages being dropped on one topic but not the other.
    - `exposure_time_us`: A one dimensional dataset of exposure times in microseconds.
    - `gain`: A one dimensional dataset of gain values in dB.
- `cam_diags`: A group, derived from `image_meta_msgs/CameraDiagnostics` messages (a custom message type), including multiple one dimensional datasets of various camera diagnostic parameters polled in between image acquisitions.
- Configuration information written as attributes to the root group `/`. This includes nearly all camera parameters readable through the Spinnaker SDK.

### 2.3. RGB

The RGB camera data files, `*_rgb_left.h5` and `*_rgb_right.h5`, contain the following:
- `image_raw`: A group, derived from [sensor_msgs/Image](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html) messages, including the following:
    - `height`, `width`, `encoding`, `is_bigendian`, `step`: Attributes for constant messages fields
    - `images`: A three dimensional dataset (messages x rows x columns) of raw (distorted) bayer encoded RGB images.
    - `timestamps`: A one dimensional dataset of TAI timestamps in nanoseconds.
- `image_meta`: A group, derived from `image_meta_msgs/ImageMeta` messages (a custom message type), including multiple one dimensional datasets of various image metadata parameters. In particular, this includes:
    - `driver_timestamps`: A one dimensional dataset of TAI timestamps in nanoseconds. Note that each timestamp matches exactly with the image the metadata corresponds to, but matches may not always exist due to some corresponding messages being dropped on one topic but not the other.
    - `exposure_time_us`: A one dimensional dataset of exposure times in microseconds.
    - `gain`: A one dimensional dataset of gain values in dB.
- `cam_diags`: A group, derived from `image_meta_msgs/CameraDiagnostics` messages (a custom message type), including multiple one dimensional datasets of various camera diagnostic parameters polled in between image acquisitions.
- Configuration information written as attributes to the root group `/`. This includes nearly all camera parameters readable through the Spinnaker SDK.

### 2.4. Event (DVS)

The event camera data files, `*_dvxplorer_left.h5` and `*_dvxplorer_right.h5`, contain the following:
- `events`: A group, derived from [dvs_msgs/EventArray](https://github.com/uzh-rpg/rpg_dvs_ros/blob/master/dvs_msgs/msg/EventArray.msg) and [dvs_msgs/Event](https://github.com/uzh-rpg/rpg_dvs_ros/blob/master/dvs_msgs/msg/Event.msg) messages, including the following:
    - `height`, `width`: Attributes for constant message fields
    - `x_coordinates`: A one dimensional dataset of event pixel x coordinates.
    - `y_coordinates`: A one dimensional dataset of event pixel y coordinates.
    - `timestamps`: A one dimensional dataset of event TAI timestamps in nanoseconds.
    - `polarities`: A one dimensional dataset of event polarities (0 denotes off/negative and 1 denotes on/positive).
- Configuration information written as attributes to the root group `/`. This includes a subset of the camera parameters readable through libcaer (with attribute names given as `[libcaer_module_address_symbolic_constant] libcaer_parameter_address_symbolic_constant`). Note that the bias preset `[DVX_DVS_CHIP_BIAS] DVX_DVS_CHIP_BIAS_SIMPLE` can be set but not read, so instead, the configuration includes the underlying parameters affected by the preset (the `[DVX_DVS_CHIP_BIAS] *` attributes).

### 2.5. Ground Truth Pose

The applanix data file, `*_applanix.h5`, contains poses of the `base_link` frame in the Earth Centered Earth Fixed (ECEF) frame. The `base_link` frame is defined with its origin on the ground at the midpoint of the vehicle's rear wheels with the x-axis pointing forward, y-axis pointing left, and z-axis pointing up. See the [Measured Extrinsics](#4-measured-extrinsics) section for information relating the `base_link` frame and the camera frames.

Specifically, the file contains the following:
- `pose_base_link`: A group, derived from [geometry_msgs/PoseStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html) messages, including the following:
    - `positions`: A two dimensional dataset (messages x coordinate axes) of `base_link` positions in the ECEF frame in meters. The order of the coordinates across the columns is x, y, z.
    - `quaternions`: A two dimensional dataset (messages x quaternion components) of quaternion rotations from the `base_link` frame to the ECEF frame. The order of the components across the columns is x, y, z, w.
    - `timestamps`: A one dimensional dataset of event TAI timestamps in nanoseconds.

## 3. Calibration Results

The calibration result file, `*_calibration_results.yaml`, contains the results of a [Kalibr](https://github.com/ethz-asl/kalibr) multi-camera calibration run with all eight cameras. The format matches that of the [camchain.yaml](https://github.com/ethz-asl/kalibr/wiki/yaml-formats) file output by Kalibr, with the one exception that the `rostopic` field, e.g. `/mono_left/image_raw`, has been changed to `frame_id`, e.g. `mono_left_optical`.

## 4. Measured Extrinsics

The measured extrinsics file, `*_measured_extrinsics.yaml`, contains manually measured extrinsics between the `base_link` frame and camera housing frames, e.g. `mono_left_housing`. Note that the camera housing frames are not identical to the camera optical frames, e.g. `mono_left_optical`, for which the Kalibr extrinsics results are given. The camera housing frames have the same orientation as the base_link frame, x-axis pointing forward, y-axis pointing left, and z-axis pointing up, while the camera optical frames have the x-axis pointing right, the y-axis pointing down, and the z-axis pointing forward. Additionally, there is an unmeasured, assumed small, translation between the camera housing and camera optical frames.