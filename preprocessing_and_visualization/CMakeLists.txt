cmake_minimum_required(VERSION 3.0.2)
project(preprocessing_and_visualization)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    sensor_msgs
    rospy
)

catkin_package()
catkin_python_setup()

catkin_install_python(
    PROGRAMS
        scripts/pose_visualization.py
        scripts/image_preprocessing.py
        scripts/stereo_rectification_visualization.py
        scripts/image_plotting.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
