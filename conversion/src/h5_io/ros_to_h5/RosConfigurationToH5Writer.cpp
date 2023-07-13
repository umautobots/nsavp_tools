#include "h5_io/ros_to_h5/RosConfigurationToH5Writer.h"

#include <cassert>
#include <iostream>

#include <rosbag/view.h>
#include <image_meta_msgs/CameraDiagnostics.h>
#include <image_meta_msgs/NameValueFloat.h>
#include <image_meta_msgs/NameValueInt.h>
#include <image_meta_msgs/NameValueString.h>

RosConfigurationToH5Writer::RosConfigurationToH5Writer(
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

void RosConfigurationToH5Writer::writeRosbagTopicView(
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
        std::cout << "RosConfigurationToH5Writer only supports image_meta_msgs/CameraDiagnostics messages\n";
        assert(false);
    }

    // Check the view only has a single message
    if (view_topic.size() != 1)
    {
        std::cout << "RosConfigurationToH5Writer expects only a single configuration message\n";
        assert(false);
    }

    // Add the group, if it does not already exist
    addGroup(path_group);

    // Get the camera diagnostics message and write all the configuration information contained within as attributes to
    // the group
    auto iterator_camera_diagnostics = view_topic.begin();
    image_meta_msgs::CameraDiagnostics::ConstPtr message_camera_diagnostics =
        iterator_camera_diagnostics->instantiate<image_meta_msgs::CameraDiagnostics>();

    writeAttributeToObject(path_group, "rosbag_start_time", iterator_camera_diagnostics->getTime().toNSec());
    writeAttributeToObject(path_group, "response_timestamp_nanoseconds",
        message_camera_diagnostics->header.stamp.toNSec());
    writeAttributeToObject(path_group, "request_timestamp_nanoseconds",
        message_camera_diagnostics->request_stamp.toNSec());
    writeAttributeToObject(path_group, "hardware_id", message_camera_diagnostics->hardware_id);

    for (const image_meta_msgs::NameValueFloat& message_name_value_float :
        message_camera_diagnostics->name_value_floats)
    {
        writeAttributeToObject(path_group, message_name_value_float.name, message_name_value_float.value);
    }

    for (const image_meta_msgs::NameValueInt& message_name_value_int :
        message_camera_diagnostics->name_value_ints)
    {
        writeAttributeToObject(path_group, message_name_value_int.name, message_name_value_int.value);
    }

    for (const image_meta_msgs::NameValueInt& message_name_value_bool :
        message_camera_diagnostics->name_value_bools)
    {
        writeAttributeToObject(path_group, message_name_value_bool.name + " (bool)", message_name_value_bool.value);
    }

    for (const image_meta_msgs::NameValueString& message_name_value_string :
        message_camera_diagnostics->name_value_strings)
    {
        writeAttributeToObject(path_group, message_name_value_string.name, message_name_value_string.value);
    }

    for (const image_meta_msgs::NameValueString& message_name_value_enum :
        message_camera_diagnostics->name_value_enums)
    {
        writeAttributeToObject(path_group, message_name_value_enum.name + " (enum)", message_name_value_enum.value);
    }
}