#include "h5_io/ros_to_h5/RosImageMetaToH5Writer.h"

#include <cassert>
#include <iostream>

#include <rosbag/view.h>
#include <image_meta_msgs/ImageMeta.h>
#include <image_meta_msgs/NameValueFloat.h>
#include <image_meta_msgs/NameValueInt.h>
#include <image_meta_msgs/NameValueString.h>

RosImageMetaToH5Writer::RosImageMetaToH5Writer(
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

void RosImageMetaToH5Writer::writeRosbagTopicView(
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
    if (connections[0]->datatype != "image_meta_msgs/ImageMeta")
    {
        std::cout << "RosImageMetaToH5Writer only supports image_meta_msgs/ImageMeta messages\n";
        assert(false);
    }

    // Add the group, if it does not already exist
    addGroup(path_group);

    // Write the message type as an attribute to the group
    writeAttributeToObject(path_group, "ros_message_type", connections[0]->datatype);

    // Create datasets for fixed fields, with units written as attributes
    size_t n_messages = view_topic.size();

    std::string path_hardware_timestamp_dataset = path_group + "/hardware_timestamps";
    addDataset<uint64_t>(path_hardware_timestamp_dataset, n_messages);
    writeAttributeToObject<std::string>(path_hardware_timestamp_dataset, "units", "nanoseconds");

    std::string path_hardware_index_dataset = path_group + "/hardware_indices";
    addDataset<uint32_t>(path_hardware_index_dataset, n_messages);

    std::string path_driver_timestamp_dataset = path_group + "/driver_timestamps";
    addDataset<uint64_t>(path_driver_timestamp_dataset, n_messages);
    writeAttributeToObject<std::string>(path_driver_timestamp_dataset, "units", "nanoseconds");

    std::string path_driver_index_dataset = path_group + "/driver_indices";
    addDataset<uint32_t>(path_driver_index_dataset, n_messages);

    std::string path_crc32_dataset = path_group + "/image_crc32_checksums";
    addDataset<uint32_t>(path_crc32_dataset, n_messages);

    // Get the first image meta message and create datasets for the name-value fields
    auto iterator_image_meta = view_topic.begin();
    image_meta_msgs::ImageMeta::ConstPtr message_first_image_meta =
        iterator_image_meta->instantiate<image_meta_msgs::ImageMeta>();

    for (const image_meta_msgs::NameValueFloat& message_name_value_float : message_first_image_meta->name_value_floats)
    {
        addDataset<double>(path_group + "/" + message_name_value_float.name, n_messages);
    }

    for (const image_meta_msgs::NameValueInt& message_name_value_int : message_first_image_meta->name_value_ints)
    {
        addDataset<int64_t>(path_group + "/" + message_name_value_int.name, n_messages);
    }

    for (const image_meta_msgs::NameValueString& message_name_value_string :
        message_first_image_meta->name_value_strings)
    {
        addDataset<std::string>(path_group + "/" + message_name_value_string.name, n_messages);
    }

    size_t message_count = 0;
    for (rosbag::MessageInstance const message_instance : view_topic)
    {
        image_meta_msgs::ImageMeta::ConstPtr message_image_meta =
            message_instance.instantiate<image_meta_msgs::ImageMeta>();

        // Note: image metadata is assumed be low bandwidth, so no care is taken here to write chunk by chunk
        writeDataToDataset(path_hardware_timestamp_dataset, message_image_meta->hardware_header.stamp.toNSec(),
            message_count);
        writeDataToDataset(path_hardware_index_dataset, message_image_meta->hardware_header.seq, message_count);
        writeDataToDataset(path_driver_timestamp_dataset, message_image_meta->driver_header.stamp.toNSec(),
            message_count);
        writeDataToDataset(path_driver_index_dataset, message_image_meta->driver_header.seq, message_count);
        writeDataToDataset(path_crc32_dataset, message_image_meta->crc32, message_count);

        for (const image_meta_msgs::NameValueFloat& message_name_value_float : message_image_meta->name_value_floats)
        {
            writeDataToDataset(path_group + "/" + message_name_value_float.name, message_name_value_float.value,
                message_count);
        }

        for (const image_meta_msgs::NameValueInt& message_name_value_int : message_image_meta->name_value_ints)
        {
            writeDataToDataset(path_group + "/" + message_name_value_int.name, message_name_value_int.value,
                message_count);
        }

        for (const image_meta_msgs::NameValueString& message_name_value_string : message_image_meta->name_value_strings)
        {
            writeDataToDataset(path_group + "/" + message_name_value_string.name, message_name_value_string.value,
                message_count);
        }

        message_count++;
        std::cout << "\33[2K\rPercent completion: " << 100.0 * message_count / n_messages << std::flush;
    }
    std::cout << "\33[2K\rPercent completion: " << 100.0 << std::flush << std::endl;
}