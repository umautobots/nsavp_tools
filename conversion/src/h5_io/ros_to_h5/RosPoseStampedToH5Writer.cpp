#include "h5_io/ros_to_h5/RosPoseStampedToH5Writer.h"

#include <cassert>
#include <iostream>

#include <rosbag/view.h>
#include <geometry_msgs/PoseStamped.h>

RosPoseStampedToH5Writer::RosPoseStampedToH5Writer(
    const int chunk_length,
    const int chunk_cache_size,
    const bool shuffle_enable,
    const std::shared_ptr<const CompressionMethod> compression_method,
    const std::shared_ptr<H5::H5File> h5_file)
    :   RosMessageToH5Writer(
            chunk_length,
            chunk_cache_size,
            shuffle_enable,
            compression_method,
            h5_file)
{}

void RosPoseStampedToH5Writer::writeRosbagTopicView(
    const std::string path_group,
    rosbag::View& view_topic)
{
    std::vector<const rosbag::ConnectionInfo*> connections = view_topic.getConnections();

    // Check that the view only has a single topic
    if (connections.size() > 1)
    {
        std::cout << "writeRosbagTopicView only supports views of a single topic\n";
        assert(false);
    }

    // Check the message type
    if (connections[0]->datatype != "geometry_msgs/PoseStamped")
    {
        std::cout << "RosPoseStampedToH5Writer only supports geometry_msgs/PoseStamped messages\n";
        assert(false);
    }

    // Add the group, if it does not already exist
    addGroup(path_group);

    // Write meta information as attributes to the group
    writeAttributeToObject(path_group, "ros_message_type", connections[0]->datatype);
    writeAttributeToObject<std::string>(path_group, "parent_frame", "ECEF");
    writeAttributeToObject<std::string>(path_group, "child_frame", "base_link");

    // Create datasets, with units and descriptors written as attributes
    size_t n_messages = view_topic.size();

    std::string path_timestamp_dataset = path_group + "/timestamps";
    addDataset<uint64_t>(path_timestamp_dataset, n_messages);
    writeAttributeToObject<std::string>(path_timestamp_dataset, "units", "nanoseconds");

    std::string path_positions_dataset = path_group + "/positions";
    std::vector<hsize_t> dimensions_position_data{3};
    addDataset<double>(path_positions_dataset, n_messages, dimensions_position_data);
    writeAttributeToObject<std::string>(path_positions_dataset, "units", "meters");
    writeAttributeToObject<std::string>(path_positions_dataset, "column order", "x, y, z");

    std::string path_quaternions_dataset = path_group + "/quaternions";
    std::vector<hsize_t> dimensions_quaternion_data{4};
    addDataset<double>(path_quaternions_dataset, n_messages, dimensions_quaternion_data);
    writeAttributeToObject<std::string>(path_quaternions_dataset, "column order", "x, y, z, w");

    size_t message_count = 0;
    std::vector<uint64_t> timestamps_to_write;
    std::vector<double> positions_to_write;
    std::vector<double> quaternions_to_write;
    for (rosbag::MessageInstance const message_instance : view_topic)
    {
        geometry_msgs::PoseStamped::Ptr message_pose_stamped =
            message_instance.instantiate<geometry_msgs::PoseStamped>();

        timestamps_to_write.push_back(message_pose_stamped->header.stamp.toNSec());
        positions_to_write.push_back(message_pose_stamped->pose.position.x);
        positions_to_write.push_back(message_pose_stamped->pose.position.y);
        positions_to_write.push_back(message_pose_stamped->pose.position.z);
        quaternions_to_write.push_back(message_pose_stamped->pose.orientation.x);
        quaternions_to_write.push_back(message_pose_stamped->pose.orientation.y);
        quaternions_to_write.push_back(message_pose_stamped->pose.orientation.z);
        quaternions_to_write.push_back(message_pose_stamped->pose.orientation.w);
        if (timestamps_to_write.size() == chunk_length_)
        {
            size_t index_message = message_count - chunk_length_ + 1;
            writeDataToDataset(path_timestamp_dataset, timestamps_to_write, index_message, chunk_length_);
            writeDataToDataset(path_positions_dataset, positions_to_write, index_message, chunk_length_);
            writeDataToDataset(path_quaternions_dataset, quaternions_to_write, index_message, chunk_length_);
            timestamps_to_write.clear();
            positions_to_write.clear();
            quaternions_to_write.clear();
        }

        message_count++;

        if (message_count % chunk_length_ == 0)
        {
            std::cout << "\33[2K\rPercent completion: " << 100.0 * message_count / n_messages << std::flush;
        }
    }

    // Write any remaining messages that didn't fit perfectly in a chunk
    if (timestamps_to_write.size() > 0)
    {
        size_t index_message = message_count - timestamps_to_write.size();
        writeDataToDataset(path_timestamp_dataset, timestamps_to_write, index_message, timestamps_to_write.size());
        writeDataToDataset(path_positions_dataset, positions_to_write, index_message, timestamps_to_write.size());
        writeDataToDataset(path_quaternions_dataset, quaternions_to_write, index_message, timestamps_to_write.size());
    }

    std::cout << "\33[2K\rPercent completion: " << 100.0 << std::flush << std::endl;
}