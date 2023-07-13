#include "h5_io/ros_to_h5/RosCameraDiagnosticsToH5Writer.h"

#include <cassert>
#include <iostream>

#include <rosbag/view.h>
#include <image_meta_msgs/CameraDiagnostics.h>
#include <image_meta_msgs/NameValueFloat.h>
#include <image_meta_msgs/NameValueInt.h>
#include <image_meta_msgs/NameValueString.h>

RosCameraDiagnosticsToH5Writer::RosCameraDiagnosticsToH5Writer(
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

void RosCameraDiagnosticsToH5Writer::writeRosbagTopicView(
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
    if (connections[0]->datatype != "image_meta_msgs/CameraDiagnostics")
    {
        std::cout << "RosCameraDiagnosticsToH5Writer only supports image_meta_msgs/CameraDiagnostics messages\n";
        assert(false);
    }

    // Add the group, if it does not already exist
    addGroup(path_group);

    // Write the message type as an attribute to the group
    writeAttributeToObject(path_group, "ros_message_type", connections[0]->datatype);

    // Get the first camera diagnostics message write the hardware ID as an attribute to the group
    auto iterator_camera_diagnostics = view_topic.begin();
    image_meta_msgs::CameraDiagnostics::ConstPtr message_first_camera_diagnostics =
        iterator_camera_diagnostics->instantiate<image_meta_msgs::CameraDiagnostics>();
    writeAttributeToObject(path_group, "hardware_id", message_first_camera_diagnostics->hardware_id);

    // Create datasets for fixed fields, with units written as attributes
    size_t n_messages = view_topic.size();

    std::string path_request_timestamp_dataset = path_group + "/request_timestamps";
    addDataset<uint64_t>(path_request_timestamp_dataset, n_messages);
    writeAttributeToObject<std::string>(path_request_timestamp_dataset, "units", "nanoseconds");

    std::string path_response_timestamp_dataset = path_group + "/response_timestamps";
    addDataset<uint64_t>(path_response_timestamp_dataset, n_messages);
    writeAttributeToObject<std::string>(path_response_timestamp_dataset, "units", "nanoseconds");

    // Create datasets for the name-value fields
    for (const image_meta_msgs::NameValueFloat& message_name_value_float :
        message_first_camera_diagnostics->name_value_floats)
    {
        addDataset<double>(path_group + "/" + message_name_value_float.name, n_messages);
    }

    for (const image_meta_msgs::NameValueInt& message_name_value_int :
        message_first_camera_diagnostics->name_value_ints)
    {
        addDataset<int64_t>(path_group + "/" + message_name_value_int.name, n_messages);
    }

    for (const image_meta_msgs::NameValueInt& message_name_value_bool :
        message_first_camera_diagnostics->name_value_bools)
    {
        addDataset<int64_t>(path_group + "/" + message_name_value_bool.name + " (bool)", n_messages);
    }

    for (const image_meta_msgs::NameValueString& message_name_value_string :
        message_first_camera_diagnostics->name_value_strings)
    {
        addDataset<std::string>(path_group + "/" + message_name_value_string.name, n_messages);
    }

    for (const image_meta_msgs::NameValueString& message_name_value_enum :
        message_first_camera_diagnostics->name_value_enums)
    {
        addDataset<std::string>(path_group + "/" + message_name_value_enum.name + " (enum)", n_messages);
    }

    size_t message_count = 0;
    for (rosbag::MessageInstance const message_instance : view_topic)
    {
        image_meta_msgs::CameraDiagnostics::ConstPtr message_camera_diagnostics =
            message_instance.instantiate<image_meta_msgs::CameraDiagnostics>();

        // Check that constant message fields haven't changed
        if (!(message_first_camera_diagnostics->hardware_id == message_camera_diagnostics->hardware_id))
        {
            std::cout << "The camera diagnostics hardware ID is not constant on topic " << connections[0]->topic <<
                "\n";
            assert(false);
        }

        // Note: camera diagnostics are assumed be low bandwidth, so no care is taken here to write chunk by chunk
        writeDataToDataset(path_request_timestamp_dataset, message_camera_diagnostics->request_stamp.toNSec(),
            message_count);
        writeDataToDataset(path_response_timestamp_dataset, message_camera_diagnostics->header.stamp.toNSec(),
            message_count);

        for (const image_meta_msgs::NameValueFloat& message_name_value_float :
            message_camera_diagnostics->name_value_floats)
        {
            writeDataToDataset(path_group + "/" + message_name_value_float.name, message_name_value_float.value,
                message_count);
        }

        for (const image_meta_msgs::NameValueInt& message_name_value_int :
            message_camera_diagnostics->name_value_ints)
        {
            writeDataToDataset(path_group + "/" + message_name_value_int.name, message_name_value_int.value,
                message_count);
        }

        for (const image_meta_msgs::NameValueInt& message_name_value_bool :
            message_camera_diagnostics->name_value_bools)
        {
             writeDataToDataset(path_group + "/" + message_name_value_bool.name + " (bool)",
                message_name_value_bool.value, message_count);
        }

        for (const image_meta_msgs::NameValueString& message_name_value_string :
            message_camera_diagnostics->name_value_strings)
        {
            writeDataToDataset(path_group + "/" + message_name_value_string.name, message_name_value_string.value,
                message_count);
        }

        for (const image_meta_msgs::NameValueString& message_name_value_enum :
            message_camera_diagnostics->name_value_enums)
        {
            writeDataToDataset(path_group + "/" + message_name_value_enum.name + " (enum)",
                message_name_value_enum.value, message_count);
        }

        message_count++;
        std::cout << "\33[2K\rPercent completion: " << 100.0 * message_count / n_messages << std::flush;
    }
    std::cout << "\33[2K\rPercent completion: " << 100.0 << std::flush << std::endl;
}