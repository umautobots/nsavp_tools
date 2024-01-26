# NSAVP

This repository contains documentation and software tools for the Novel Sensors for Autonomous Vehicle Perception (NSAVP) dataset. See the [project page](https://umautobots.github.io/nsavp) for links to all resources, including data download.

Additional information is available in the following documentation pages:
- [Sensors](documentation/sensors.md): Description of the sensor suite.
- [Data Format](documentation/data_format.md): Description of the data collected and the data format.
- [Conversion](documentation/conversion.md): Guide to combine H5 files and convert a combined H5 file to a ROS1 rosbag or a folder containing CSV files and PNG images.
- [Preprocessing and Visualization](documentation/preprocessing_and_visualization.md): Guide to perform preprocessing operations (e.g., cropping, debayering, rectification) and visualize the data.
- [Place Recognition](documentation/place_recognition.md): Tutorial for evaluating a place recognition method on the dataset.

If you have any questions or comments, please create a GitHub issue.

## Citation

```
@misc{nsavp2024,
    title={Dataset and Benchmark: Novel Sensors for Autonomous Vehicle Perception},
    author={Spencer Carmichael and Austin Buchan and Mani Ramanagopal and Radhika Ravi and Ram Vasudevan and Katherine A. Skinner},
    year={2024},
    eprint={2401.13853},
    archivePrefix={arXiv},
    primaryClass={cs.RO}}
```