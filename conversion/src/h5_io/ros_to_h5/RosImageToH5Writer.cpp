#include "h5_io/ros_to_h5/RosImageToH5Writer.h"

#include <cassert>
#include <iostream>

#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

RosImageToH5Writer::RosImageToH5Writer(
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

void RosImageToH5Writer::writeRosbagTopicView(
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
    if (connections[0]->datatype != "sensor_msgs/Image")
    {
        std::cout << "RosImageToH5Writer only supports sensor_msgs/Image messages\n";
        assert(false);
    }

    // Add the group, if it does not already exist
    addGroup(path_group);

    // Write the message type as an attribute to the group
    writeAttributeToObject(path_group, "ros_message_type", connections[0]->datatype);

    // Get the first image message and write the constant message fields as attributes attached to the topic name group
    auto iterator_image = view_topic.begin();
    sensor_msgs::Image::Ptr message_first_image = iterator_image->instantiate<sensor_msgs::Image>();
    writeAttributeToObject(path_group, "height", message_first_image->height);
    writeAttributeToObject(path_group, "width", message_first_image->width);
    writeAttributeToObject(path_group, "encoding", message_first_image->encoding);
    writeAttributeToObject(path_group, "is_bigendian", message_first_image->is_bigendian);
    writeAttributeToObject(path_group, "step", message_first_image->step);

    // Create the timestamp dataset (with no chunking or compression) and write the units as an attribute
    size_t n_messages = view_topic.size();
    std::string path_timestamp_dataset = path_group + "/timestamps";
    std::vector<hsize_t> dimensions_timestamp_data; // Empty to imply scalar
    addDataset<uint64_t>(path_timestamp_dataset, n_messages, dimensions_timestamp_data, true);
    writeAttributeToObject<std::string>(path_timestamp_dataset, "units", "nanoseconds");

    // Create the image data dataset
    std::string path_image_dataset = path_group + "/images";
    std::vector<hsize_t> dimensions_image_data{message_first_image->height, message_first_image->width};
    H5::DataType datatype_image;
    if (message_first_image->encoding == sensor_msgs::image_encodings::MONO8 ||
        message_first_image->encoding == sensor_msgs::image_encodings::BAYER_RGGB8)
    {
        datatype_image = getH5DataType<uint8_t>();
        addDataset<uint8_t>(path_image_dataset, n_messages, dimensions_image_data);
    }
    else if (message_first_image->encoding == sensor_msgs::image_encodings::MONO16)
    {
        datatype_image = getH5DataType<uint16_t>();
        addDataset<uint16_t>(path_image_dataset, n_messages, dimensions_image_data);
    }
    else
    {
        std::cout << "Image message encoding currently not supported\n";
        assert(false);
    }

    size_t message_count = 0;
    std::vector<uint64_t> timestamps_to_write;
    std::vector<uint8_t> images_to_write;
    for (rosbag::MessageInstance const message_instance : view_topic)
    {
        sensor_msgs::Image::Ptr message_image = message_instance.instantiate<sensor_msgs::Image>();

        // Check that the constant message fields haven't changed
        if (!(message_image->height == message_first_image->height &&
            message_image->width == message_first_image->width &&
            message_image->encoding == message_first_image->encoding &&
            message_image->is_bigendian == message_first_image->is_bigendian &&
            message_image->step == message_first_image->step))
        {
            std::cout << "A field expected to be constant is not on topic " << connections[0]->topic << "\n";
            assert(false);
        }

        // If the chunk length is 1 or 0, avoid copying image data into a vector
        if (chunk_length_ <= 1)
        {
            timestamps_to_write.push_back(message_image->header.stamp.toNSec());
            writeDataToDataset(path_timestamp_dataset, timestamps_to_write, message_count, 1);
            writeDataToDataset(path_image_dataset, message_image->data.data(), datatype_image, message_count, 1);
            timestamps_to_write.clear();
        }
        else
        {
            timestamps_to_write.push_back(message_image->header.stamp.toNSec());
            images_to_write.insert(images_to_write.end(), message_image->data.begin(), message_image->data.end());
            if (timestamps_to_write.size() == chunk_length_)
            {
                size_t index_message = message_count - chunk_length_ + 1;
                writeDataToDataset(path_timestamp_dataset, timestamps_to_write, index_message, chunk_length_);
                writeDataToDataset(path_image_dataset, images_to_write.data(), datatype_image, index_message,
                    chunk_length_);
                timestamps_to_write.clear();
                images_to_write.clear();
            }
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
        writeDataToDataset(path_image_dataset, images_to_write.data(), datatype_image, index_message,
            timestamps_to_write.size());
    }

    std::cout << "\33[2K\rPercent completion: " << 100.0 << std::flush << std::endl;
}