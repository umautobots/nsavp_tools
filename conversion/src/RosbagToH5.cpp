#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <filesystem>
#include <memory>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include "H5Cpp.h"

#include "h5_io/ros_to_h5/RosMessageToH5Writer.h"
#include "h5_io/ros_to_h5/RosImageToH5Writer.h"
#include "h5_io/ros_to_h5/RosEventsToH5Writer.h"
#include "h5_io/ros_to_h5/RosImageMetaToH5Writer.h"
#include "h5_io/ros_to_h5/RosCameraDiagnosticsToH5Writer.h"
#include "h5_io/ros_to_h5/RosConfigurationToH5Writer.h"
#include "h5_io/ros_to_h5/RosPoseStampedToH5Writer.h"

#include "h5_io/ros_to_h5/compression/GzipCompressionMethod.h"

using fspath = std::filesystem::path;

class H5FileOpener
{
public:
    H5FileOpener() {}

    void openFile(
        const std::string& folder,
        const std::string& topic,
        std::string prefix,
        std::shared_ptr<H5::H5File>& h5_file)
    {
        prefix = (prefix.empty()) ? prefix : prefix + "_";
        std::string filepath = fspath(folder) / fspath(prefix + getNameSpaceWithoutForwardSlashes(topic) + ".h5");
        std::cout << "Writing to " << filepath << "\n";

        // If the file has not been previously opened by this process, delete the current contents of any existing file
        // with this filepath before writing to it
        if (std::find(opened_files_.begin(), opened_files_.end(), filepath) == opened_files_.end())
        {
            h5_file = std::make_shared<H5::H5File>(filepath, H5F_ACC_TRUNC);
        }
        // If the file has already been opened by this process, open the file in read/write mode
        else
        {
            h5_file = std::make_shared<H5::H5File>(filepath, H5F_ACC_RDWR);
        }

        opened_files_.push_back(filepath);
    }

private:
    /**
     * Returns the namespace of the given topic with forward slashes replaced with underscores.
     *
     * @param[in] topic The topic.
     *
     * @return The namespace of the given topic with forward slashes replaced with underscores.
     */
    std::string getNameSpaceWithoutForwardSlashes(const std::string& topic)
    {
        // Initialize string as the namespace
        std::string namespace_without_forward_slashes = topic.substr(1, topic.find_last_of("/") - 1);

        // Replace all remaining forward slashes with underscores
        size_t index_forward_slash = namespace_without_forward_slashes.find("/");
        while (index_forward_slash != std::string::npos)
        {
            namespace_without_forward_slashes[index_forward_slash] = '_';
            index_forward_slash = namespace_without_forward_slashes.find("/");
        }

        return namespace_without_forward_slashes;
    }

    std::vector<std::string> opened_files_;
};

int main(int argc, char* argv[])
{
    // Initialize ROS node
    ros::init(argc, argv, "rosbag_to_h5");
    ros::NodeHandle nh_private("~");

    // Process arguments
    std::string path_bag_in, path_h5_folder_out, prefix;
    std::vector<std::string> topics;
    nh_private.param<std::string>("path_bag_in", path_bag_in, "");
    nh_private.param<std::string>("path_h5_folder_out", path_h5_folder_out, "");
    nh_private.param<std::string>("prefix", prefix, "");
    nh_private.param<std::vector<std::string>>("topics", topics, std::vector<std::string>{});

    // Open the input rosbag
    std::cout << "Opening input rosbag: " << path_bag_in.c_str() << "\n";
    rosbag::Bag bag_in;
    bag_in.open(path_bag_in.c_str(), rosbag::bagmode::Read);

    // Create the output folder
    std::filesystem::create_directories(path_h5_folder_out);

    // Set the compression method
    std::shared_ptr<GzipCompressionMethod> compression_method = std::make_shared<GzipCompressionMethod>(6);

    // Loop over topics
    H5FileOpener h5_file_opener;
    rosbag::View view_full(bag_in);
    std::vector<const rosbag::ConnectionInfo*> connections = view_full.getConnections();
    for (const rosbag::ConnectionInfo* connection : connections)
    {
        // Process the current topic if it was specified, or if no topics were specified
        std::string topic = connection->topic;
        if (!topics.empty() && (std::find(topics.begin(), topics.end(), topic) == topics.end()))
        {
            continue;
        }

        std::string message_type = connection->datatype;
        rosbag::View view_topic(bag_in, rosbag::TopicQuery(topic));
        std::shared_ptr<H5::H5File> h5_file;
        std::string path_group = topic.substr(topic.find_last_of("/") + 1);
        std::cout << "Found topic " << topic << " with ROS message type " << message_type << "\n";
        if (message_type == "sensor_msgs/Image")
        {
            h5_file_opener.openFile(path_h5_folder_out, topic, prefix, h5_file);
            RosImageToH5Writer ros_image_to_h5_writer(1, 0, true, compression_method, h5_file);
            ros_image_to_h5_writer.writeRosbagTopicView(path_group, view_topic);
        }
        else if (message_type == "image_meta_msgs/ImageMeta")
        {
            h5_file_opener.openFile(path_h5_folder_out, topic, prefix, h5_file);
            RosImageMetaToH5Writer ros_image_meta_to_h5_writer(0, 0, false, compression_method, h5_file);
            ros_image_meta_to_h5_writer.writeRosbagTopicView(path_group, view_topic);
        }
        else if (message_type == "image_meta_msgs/CameraDiagnostics")
        {
            if (view_topic.size() == 1)
            {
                h5_file_opener.openFile(path_h5_folder_out, topic, prefix, h5_file);
                RosConfigurationToH5Writer ros_configuration_to_h5_writer(0, 0, false, compression_method, h5_file);
                ros_configuration_to_h5_writer.writeRosbagTopicView("/", view_topic);
            }
            else
            {
                h5_file_opener.openFile(path_h5_folder_out, topic, prefix, h5_file);
                RosCameraDiagnosticsToH5Writer ros_camera_diagnostics_to_h5_writer(0, 0, false, compression_method,
                    h5_file);
                ros_camera_diagnostics_to_h5_writer.writeRosbagTopicView(path_group, view_topic);
            }
        }
        else if (message_type == "dvs_msgs/EventArray")
        {
            h5_file_opener.openFile(path_h5_folder_out, topic, prefix, h5_file);
            // Note: the chunk length used here (2^15) matches that used in the TUM VIE dataset
            RosEventsToH5Writer ros_events_to_h5_writer(32768, 0, true, compression_method, h5_file);
            ros_events_to_h5_writer.writeRosbagTopicView(path_group, view_topic);
        }
        else if (message_type == "geometry_msgs/PoseStamped")
        {
            h5_file_opener.openFile(path_h5_folder_out, topic, prefix, h5_file);
            RosPoseStampedToH5Writer ros_pose_stamped_to_h5_writer(2000, 0, true, compression_method, h5_file);
            ros_pose_stamped_to_h5_writer.writeRosbagTopicView(path_group, view_topic);
        }
        else
        {
            std::cout << "Skipping, the message type is not supported\n";
        }
    }

    // Close the rosbag
    bag_in.close();

    return 0;
}