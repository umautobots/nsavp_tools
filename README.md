# NSAVP

**NOTE: the dataset and software toolkit are still under development: we plan to release one additional route and add the place recognition tutorial described below. If you are interested in these additions check back and/or Watch the repository on GitHub for updates. [CHANGELOG.md](CHANGELOG.md) will be updated to record changes since the initial release.**

This repository contains documentation and software tools for the Novel Sensors for Autonomous Vehicle Perception (NSAVP) dataset, introduced for the [IROS 2023 Workshop](https://sites.google.com/umich.edu/novelsensors2023) of the same name. The data is available for download on [Deep Blue Data](https://deepblue.lib.umich.edu/data/collections/v118rf157).

Additional information is available in the following documentation pages:
- [Sensors](documentation/sensors.md): Description of the sensor suite.
- [Data Format](documentation/data_format.md): Description of the data collected and the data format.
- [Conversion](documentation/conversion.md): Guide to convert the data from H5 to ROS1 rosbag and back, as well as combine H5 files.
- [Preprocessing and Visualization](documentation/preprocessing_and_visualization.md): Guide to perform preprocessing operations (e.g., cropping, debayering, rectification) and visualize the data.
- Place Recognition (coming soon): Tutorial for evaluating a place recognition method on the dataset.

If you have any questions or comments, please create a GitHub issue.

## Citation

An arXiv paper describing the details of the dataset is planned. Once it is released, this section will be updated with the citation.