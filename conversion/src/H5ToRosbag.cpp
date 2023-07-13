#include <iostream>
#include <vector>
#include <string>

#include <ros/ros.h>
#include <rosbag/bag.h>

#include "H5Cpp.h"

#include "h5_io/h5_to_ros/H5ToRosMessageWriter.h"
#include "h5_io/h5_to_ros/H5ToRosImageWriter.h"
#include "h5_io/h5_to_ros/H5ToRosEventsWriter.h"
#include "h5_io/h5_to_ros/H5ToRosImageMetaWriter.h"
#include "h5_io/h5_to_ros/H5ToRosCameraDiagnosticsWriter.h"
#include "h5_io/h5_to_ros/H5ToRosConfigurationWriter.h"
#include "h5_io/h5_to_ros/H5ToRosPoseStampedWriter.h"

int main(int argc, char* argv[])
{
    // Initialize ROS node
    ros::init(argc, argv, "h5_to_rosbag");
    ros::NodeHandle nh_private("~");

    // Process arguments
    std::string path_h5_in, path_bag_out;
    std::vector<std::string> topics;
    double event_array_rate;
    nh_private.param<std::string>("path_h5_in", path_h5_in, "");
    nh_private.param<std::string>("path_bag_out", path_bag_out, "");
    nh_private.param<std::vector<std::string>>("topics", topics, std::vector<std::string>{});
    nh_private.param<double>("event_array_rate", event_array_rate, 20.1413);

    // Open the h5 file
    std::cout << "Opening input H5 file: " << path_h5_in.c_str() << "\n";
    H5::H5File h5_file_in(path_h5_in, H5F_ACC_RDONLY);

    // Open the output rosbag
    std::cout << "Opening output rosbag: " << path_bag_out.c_str() << "\n";
    rosbag::Bag bag_out;
    bag_out.open(path_bag_out.c_str(), rosbag::bagmode::Write);

    // Get namespace (top level) group paths
    std::vector<std::string> paths_namespace_group;
    H5Literate(h5_file_in.getLocId(), H5_INDEX_NAME, H5_ITER_NATIVE, NULL, H5LIterateCallbackFunction,
        &paths_namespace_group);

    // Loop over namespace groups
    for (std::string path_namespace_group : paths_namespace_group)
    {
        // Open namespace group
        std::cout << "Found namespace group " << path_namespace_group << "\n";
        H5::Group group_namespace = h5_file_in.openGroup(path_namespace_group);

        // Get topic (second level) group names
        std::vector<std::string> names_topic_group;
        H5Literate(group_namespace.getLocId(), H5_INDEX_NAME, H5_ITER_NATIVE, NULL, H5LIterateCallbackFunction,
            &names_topic_group);

        // Loop over topic groups
        for (std::string name_topic_group : names_topic_group)
        {
            std::string path_topic_group = "/" + path_namespace_group + "/" + name_topic_group;

            // Process the current topic if it was specified, or if no topics were specified
            if (!topics.empty() && (std::find(topics.begin(), topics.end(), path_topic_group) == topics.end()))
            {
                continue;
            }
            std::cout << "Found topic group " << path_topic_group << "\n";

            H5::Group group_topic = h5_file_in.openGroup(path_topic_group);
            if (!group_topic.attrExists("ros_message_type"))
            {
                std::cout << "Skipping, the topic group has no ros_message_type attribute\n";
                continue;
            }

            H5::Attribute attribute_ros_message_type = group_topic.openAttribute("ros_message_type");
            std::string message_type;
            attribute_ros_message_type.read(H5::StrType(H5::PredType::C_S1, H5T_VARIABLE), message_type);
            std::cout << "Topic group derived from ROS message type: "  << message_type << "\n";
            if (message_type == "sensor_msgs/Image")
            {
                std::cout << "Writing messages...\n";
                H5ToRosImageWriter h5_to_ros_image_writer(group_topic);
                h5_to_ros_image_writer.writeH5TopicGroup(path_topic_group, bag_out);
            }
            else if (message_type == "image_meta_msgs/ImageMeta")
            {
                std::cout << "Writing messages...\n";
                H5ToRosImageMetaWriter h5_to_ros_image_meta_writer(group_topic);
                h5_to_ros_image_meta_writer.writeH5TopicGroup(path_topic_group, bag_out);
            }
            else if (message_type == "image_meta_msgs/CameraDiagnostics")
            {
                std::cout << "Writing messages...\n";
                H5ToRosCameraDiagnosticsWriter h5_to_ros_camera_diagnostics_writer(group_topic);
                h5_to_ros_camera_diagnostics_writer.writeH5TopicGroup(path_topic_group, bag_out);
            }
            else if (message_type == "dvs_msgs/EventArray")
            {
                std::cout << "Writing messages...\n";
                H5ToRosEventsWriter h5_to_ros_events_writer(group_topic, event_array_rate);
                h5_to_ros_events_writer.writeH5TopicGroup(path_topic_group, bag_out);
            }
            else if (message_type == "geometry_msgs/PoseStamped")
            {
                std::cout << "Writing messages...\n";
                H5ToRosPoseStampedWriter h5_to_ros_pose_stamped_writer(group_topic);
                h5_to_ros_pose_stamped_writer.writeH5TopicGroup(path_topic_group, bag_out);
            }
            else
            {
                std::cout << "Skipping, the message type is not supported\n";
                continue;
            }
        }

        // Process the configuration topic if it was specified, or if no topics were specified
        std::string topic_configuration = "/" + path_namespace_group + "/config";
        if (!topics.empty() && (std::find(topics.begin(), topics.end(), topic_configuration) == topics.end()))
        {
            continue;
        }
        H5ToRosConfigurationWriter h5_to_ros_configuration_writer(group_namespace);
        h5_to_ros_configuration_writer.writeH5TopicGroup(path_namespace_group, bag_out);
    }

    // Close the rosbag
    bag_out.close();

    return 0;
}