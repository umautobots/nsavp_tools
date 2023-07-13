# Conversion

We provide scripts for converting rosbag data to H5 and vice versa, as well as combining H5 files.

# 1. Setup

We recommend building and running the conversion code with Docker. This section describes Docker installation, building the `nsavp_conversion` Docker image, and running the image with an interactive bash session. The following has been tested with Ubuntu 22.04.

To begin, install [Docker Engine](https://docs.docker.com/engine/install/ubuntu/) and perform [this](https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user) post-installation step.

After installing Docker, a Docker image for running the code described in this page can be built as follows:
```
cd nsavp_tools/
docker build -t nsavp_conversion -f docker/Dockerfile_conversion .
```
The result will be a new Docker image named `nsavp_conversion` with the `conversion` package built in the `/catkin_ws/` directory within the image.

All commands in subsequent sections are run through an interactive bash session in a Docker container running the `nsavp_conversion` image. The following commands will run the image and start an interactive bash session in the running container:
```
xhost +
docker run -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v /path/to/nsavp/data/:/nsavp_data nsavp_conversion
```
The arguments `-e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix` are included to allow display windows to be opened from within the container. The `/path/to/nsavp/data/` path should be replaced with the folder storing NSAVP data on your host machine. This folder will appear within the container as the `/nsavp_data` folder. Once finished, the interactive session can be exited with `Ctrl+D` and the container can be removed by first finding it's name with `docker ps` and then removing it with `docker rm <container name>`.

Note that the [HDFView GUI](https://www.hdfgroup.org/downloads/hdfview/) is installed in the `nsavp_conversion` Docker image and can be used to view the contents of H5 files with the following command:
```
HDFView /path/to/file.h5
```

# 2. Converting a ROS1 Rosbag File to H5 Files

The `rosbag_to_h5` script can be used to convert a ROS1 rosbag file to multiple H5 files using the strategy described in the [General Sensor Data H5 Format](data_format.md#1-general-sensor-data-h5-format) section of the data format documentation page.

The script can be run with `rosbag_to_h5.launch`. The launch file takes the following arguments:
- `path_bag_in`: A path specifying the ROS1 rosbag file to convert.
- `path_h5_folder_out`: Path specifying a folder to write the output H5 files to.
- `prefix`: The sequence prefix used in the output H5 filenames. If empty, no prefix is used. Defaults to an empty string `""`.
- `topics`: A list of the topics to process in the conversion. If empty, all topics with supported message types will be processed. Defaults to an empty list `[]`.

For example, if there is a rosbag file `R0_FA0.bag` located in the folder `/nsavp_data/R0_FA0/` within the running Docker container (see [Setup](#1-setup)) with topics `/mono_left/image_raw`, `/mono_left/image_meta`, `/mono_right/image_raw`, and `/mono_right/image_meta`, the rosbag can be converted to H5 files with the following command:
```
roslaunch conversion rosbag_to_h5.launch path_bag_in:=/nsavp_data/R0_FA0/R0_FA0.bag path_h5_folder_out:=/nsavp_data/R0_FA0/ prefix:=R0_FA0
```
The result would be two new H5 files, `R0_FA0_mono_left.h5` and `R0_FA0_mono_right.h5`, located in the folder `/nsavp_data/R0_FA0/`, each containing `/image_raw` and `/image_meta` groups.

# 3. Combining H5 Files

The `h5_combine` script can be used to symbollically and directly combine H5 files. In the symbolic case, a new H5 file is created with external links to the original H5 files without copying any data. In the direct case, a new H5 file is created and data from the original H5 files is copied into it. The resulting files are functionally equivalent, as long as the external link filepaths remain valid in the symbolic case. In both cases, the root groups in the original H5 files appear as top level groups in the new H5 file with names derived from the filenames of the original H5 files.

The script can be run with `h5_combine.launch`. The launch file takes the following arguments:
- `path_base_folder`: A path specifying a base folder that all other paths are appended to. If this path is given and the `filepaths_h5_in` list is empty, the script will combine all `.h5` files in the top level of the `path_base_folder` folder. Defaults to an empty string `""`.
- `filepath_h5_out`: A filepath for the new H5 file to be created. This path is appended to `path_base_folder`.  Defaults to an empty string `""`.
- `filepaths_h5_in`: A list of filepaths specifying the original H5 files to be combined. These paths are appended to `path_base_folder`.  Defaults to an empty list `[]`.
- `remove_prefix`: A boolean specifying whether (`true`) or not (`false`) to remove the sequence prefix when converting the filenames of the original H5 files to group names in the new H5 file. For example, if `true` the file `R0_FA0_mono_left.h5` would appear as the group `/mono_left` in the new file, and if `false` it would appear as the group `/R0_FA0_mono_left`. Note that is not possible to create two groups with the same name (e.g. attempting to combining files `R0_FA0_mono_left.h5` and `R0_FA1_mono_left.h5` with `remove_prefix` set to `true` will fail). Defaults to `true`.
- `symbolic`: A boolean specifying whether (`true`) to symbolically combine the original H5 files using external links or (`false`) copy the data from the original H5 files into the new file. Defaults to `true`.
- `relative`: A boolean specifying whether (`true`) to create external links with relative filepaths (i.e. relative to the folder containing `filepath_h5_out`) or (`false`) absolute filepaths. Ignored if `symbolic` is `false`. Defaults to `true`.

For example, if the files `R0_FA0_mono_left.h5` and `R0_FA0_mono_right.h5` are located in the folder `/nsavp_data/R0_FA0/` within the running Docker container (see [Setup](#1-setup)) they can be combined with the following command:
```
roslaunch conversion h5_combine.launch path_base_folder:=/nsavp_data/R0_FA0/ filepath_h5_out:=R0_FA0.h5
```
The result would be a new H5 file, `/nsavp_data/R0_FA0/R0_FA0.h5`, containing groups `/mono_left` and `/mono_right`.

As another example, if additionally the file `R0_RA0_mono_left.h5` was located in the folder `/nsavp_data/R0_RA0/` within the running Docker container, then the `R0_FA0_mono_left.h5` and `R0_RA0_mono_left.h5` files could be combined with the following command:
```
roslaunch conversion h5_combine.launch filepath_h5_out:=/nsavp_data/mono_left.h5 filepaths_h5_in:=[/nsavp_data/R0_FA0/R0_FA0_mono_left.h5,/nsavp_data/R0_RA0/R0_RA0_mono_left.h5] remove_prefix:=false
```
The result would be a new H5 file, `/nsavp_data/mono_left.h5`, containing groups `/R0_FA0_mono_left` and `/R0_RA0_mono_left`. Note that without `remove_prefix:=false` this operation would fail because there cannot be two groups with the same name.

In the above examples, the groups are external links with relative paths, i.e. in the `/nsavp_data/mono_left.h5` file the `/R0_FA0_mono_left` group has an external link with path `R0_FA0/R0_FA0_mono_left.h5`. This means that if the `/nsavp_data/mono_left.h5` file is moved, the links will no longer point to the files. Absolute paths can instead be used with the argument `relative:=false`, but note that the absolute paths would be within the Docker container's filesystem in this example. Another option is to use the argument `symbolic:=false`, in which case the original data will be copied and the new H5 file will contain no external links.

# 4. Converting H5 Files to a ROS1 Rosbag File

The `h5_to_rosbag` script can be used to convert an H5 file to a ROS1 rosbag file. To use the script, first combine the H5 files to be converted using `h5_combine`, as described above in [Combining H5 Files](#3-combining-h5-files), and then run this script on the resulting H5 file. Note that `h5_combine` must be used even if converting a single H5 file (e.g. `R0_FA0_mono_left.h5`) because the `h5_to_rosbag` script assumes the top level groups in the H5 file correspond to ROS namespaces (e.g. `/mono_left`) and that subgroups correspond to topics (e.g. `/mono_left/image_raw`). The script additionally assumes the subgroups are structured as described in the [General Sensor Data H5 Format](data_format.md#1-general-sensor-data-h5-format) section of the data format documentation page.

The script can be run with `h5_to_rosbag.launch`. The launch file takes the following arguments:
- `path_h5_in`: A path specifying the H5 file to convert.
- `path_bag_out`: A path specifying the rosbag to write.
- `topics`: A list of topics to process in the conversion. The ROS topic names match the subgroup paths in the input H5 file (e.g. `/mono_left/image_raw`). If empty, all topics with supported message types will be processed. Defaults to an empty list `[]`.
- `event_array_rate`: The rate at which to write out [dvs_msgs/EventArray](https://github.com/uzh-rpg/rpg_dvs_ros/blob/master/dvs_msgs/msg/EventArray.msg) messages when writing out event data. Some applications require a high event array message rate (e.g. [ESVO](https://github.com/HKUST-Aerial-Robotics/ESVO) recommends 1000 Hz). Defaults to 20.1413 Hz (which matches the monochrome and RGB framerate).

Continuing the first example given above in [Combining H5 Files](#3-combining-h5-files), the `R0_FA0.h5` file located in the folder `/nsavp_data/R0_FA0/` within the running Docker container (see [Setup](#1-setup)) can be converted to a rosbag with the following command:
```
roslaunch conversion h5_to_rosbag.launch path_h5_in:=/nsavp_data/R0_FA0/R0_FA0.h5 path_bag_out:=/nsavp_data/R0_FA0/R0_FA0.bag
```
The result would be a new rosbag file, `/nsavp_data/R0_FA0/R0_FA0.bag`, containing a topic for each subgroup in `R0_FA0.h5` (`/mono_left/image_raw`, `/mono_left/image_meta`, etc.).

Note that the resulting rosbag is uncompressed and will be larger than the input H5 files combined. This is especially true of the event data, which is typically ~5 times larger in the rosbag format.