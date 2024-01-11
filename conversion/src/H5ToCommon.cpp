#include <iostream>
#include <vector>
#include <string>
#include <filesystem>

#include <ros/ros.h>

#include "H5Cpp.h"

#include "h5_io/h5_to_common/H5ToCommonImageWriter.h"
#include "h5_io/h5_to_common/H5ToCommonEventsWriter.h"
#include "h5_io/h5_to_common/H5ToCommonImageMetaWriter.h"
#include "h5_io/h5_to_common/H5ToCommonCameraDiagnosticsWriter.h"
#include "h5_io/h5_to_common/H5ToCommonConfigurationWriter.h"
#include "h5_io/h5_to_common/H5ToCommonPoseStampedWriter.h"

int main(int argc, char* argv[])
{
    // Initialize ROS node
    ros::init(argc, argv, "h5_to_common");
    ros::NodeHandle nh_private("~");

    // Process arguments
    std::string path_h5_in, path_folder_out;
    std::vector<std::string> topics;
    nh_private.param<std::string>("path_h5_in", path_h5_in, "");
    nh_private.param<std::string>("path_folder_out", path_folder_out, "");
    nh_private.param<std::vector<std::string>>("topics", topics, std::vector<std::string>{});

    // Open the h5 file
    std::cout << "Opening input H5 file: " << path_h5_in.c_str() << "\n";
    H5::H5File h5_file_in(path_h5_in, H5F_ACC_RDONLY);

    // Create the output folder
    std::cout << "Creating output folder: " << path_folder_out.c_str() << "\n";
    std::filesystem::create_directories(path_folder_out);

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
                std::cout << "Writing PNG files...\n";
                H5ToCommonImageWriter h5_to_ros_image_writer(group_topic);
                std::filesystem::create_directories(path_folder_out + "/" + path_namespace_group);
                std::string path_output = path_folder_out + "/" + path_namespace_group + "/" + name_topic_group + "/";
                h5_to_ros_image_writer.writeH5TopicGroup(path_output);
            }
            else if (message_type == "image_meta_msgs/ImageMeta")
            {
                std::cout << "Writing CSV file...\n";
                H5ToCommonImageMetaWriter h5_to_ros_image_meta_writer(group_topic);
                std::filesystem::create_directories(path_folder_out + "/" + path_namespace_group);
                std::string path_output = path_folder_out + "/" + path_namespace_group + "/" + name_topic_group +
                    ".csv";
                h5_to_ros_image_meta_writer.writeH5TopicGroup(path_output);
            }
            else if (message_type == "image_meta_msgs/CameraDiagnostics")
            {
                std::cout << "Writing CSV file...\n";
                H5ToCommonCameraDiagnosticsWriter h5_to_ros_camera_diagnostics_writer(group_topic);
                std::filesystem::create_directories(path_folder_out + "/" + path_namespace_group);
                std::string path_output = path_folder_out + "/" + path_namespace_group + "/" + name_topic_group +
                    ".csv";
                h5_to_ros_camera_diagnostics_writer.writeH5TopicGroup(path_output);
            }
            else if (message_type == "dvs_msgs/EventArray")
            {
                std::cout << "Writing CSV file...\n";
                H5ToCommonEventsWriter h5_to_ros_events_writer(group_topic);
                std::filesystem::create_directories(path_folder_out + "/" + path_namespace_group);
                std::string path_output = path_folder_out + "/" + path_namespace_group + "/" + name_topic_group +
                    ".csv";
                h5_to_ros_events_writer.writeH5TopicGroup(path_output);
            }
            else if (message_type == "geometry_msgs/PoseStamped")
            {
                std::cout << "Writing CSV file...\n";
                H5ToCommonPoseStampedWriter h5_to_ros_pose_stamped_writer(group_topic);
                std::filesystem::create_directories(path_folder_out + "/" + path_namespace_group);
                std::string path_output = path_folder_out + "/" + path_namespace_group + "/" + name_topic_group +
                    ".csv";
                h5_to_ros_pose_stamped_writer.writeH5TopicGroup(path_output);
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
        H5ToCommonConfigurationWriter h5_to_ros_configuration_writer(group_namespace);
        std::filesystem::create_directories(path_folder_out + "/" + path_namespace_group);
        std::string path_output = path_folder_out + "/" + path_namespace_group + "/config.csv";
        h5_to_ros_configuration_writer.writeH5TopicGroup(path_output);
    }

    return 0;
}