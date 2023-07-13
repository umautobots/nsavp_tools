#include "h5_io/ros_to_h5/RosEventsToH5Writer.h"

#include <cassert>
#include <iostream>

#include <rosbag/view.h>
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>

RosEventsToH5Writer::RosEventsToH5Writer(
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

void RosEventsToH5Writer::writeRosbagTopicView(
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
    if (connections[0]->datatype != "dvs_msgs/EventArray")
    {
        std::cout << "RosEventsToH5Writer only supports dvs_msgs/EventArray messages\n";
        assert(false);
    }

    // Add the group, if it does not already exist
    addGroup(path_group);

    // Write the message type as an attribute to the group
    writeAttributeToObject(path_group, "ros_message_type", connections[0]->datatype);

    // Get the first event array message and write the constant message fields as attributes attached to the topic name
    // group
    auto iterator_event_array = view_topic.begin();
    dvs_msgs::EventArray::Ptr message_first_event_array = iterator_event_array->instantiate<dvs_msgs::EventArray>();
    writeAttributeToObject(path_group, "height", message_first_event_array->height);
    writeAttributeToObject(path_group, "width", message_first_event_array->width);

    // Get the total number of event messages
    // TODO: this is inefficient, should use extendible H5 datasets instead
    std::cout << "Getting the total number of event messages\n";
    size_t n_messages = 0;
    for(rosbag::MessageInstance const message_instance : view_topic)
    {
        dvs_msgs::EventArray::ConstPtr message_event_array = message_instance.instantiate<dvs_msgs::EventArray>();
        n_messages += message_event_array->events.size();
    }

    // Create datasets, with units and descriptors written as attributes
    std::string path_x_coordinates_dataset = path_group + "/x_coordinates";
    addDataset<uint16_t>(path_x_coordinates_dataset, n_messages);
    writeAttributeToObject<std::string>(path_x_coordinates_dataset, "units", "pixels");

    std::string path_y_coordinates_dataset = path_group + "/y_coordinates";
    addDataset<uint16_t>(path_y_coordinates_dataset, n_messages);
    writeAttributeToObject<std::string>(path_y_coordinates_dataset, "units", "pixels");

    std::string path_timestamp_dataset = path_group + "/timestamps";
    addDataset<uint64_t>(path_timestamp_dataset, n_messages);
    writeAttributeToObject<std::string>(path_timestamp_dataset, "units", "nanoseconds");

    std::string path_polarity_dataset = path_group + "/polarities";
    addDataset<uint8_t>(path_polarity_dataset, n_messages);
    writeAttributeToObject<std::string>(path_polarity_dataset, "on/positive", "1");
    writeAttributeToObject<std::string>(path_polarity_dataset, "off/negative", "0");

    size_t message_count = 0;
    std::vector<uint16_t> x_coordinates_to_write;
    std::vector<uint16_t> y_coordinates_to_write;
    std::vector<uint64_t> timestamps_to_write;
    std::vector<uint8_t> polarities_to_write;
    for (rosbag::MessageInstance const message_instance : view_topic)
    {
        dvs_msgs::EventArray::ConstPtr message_event_array = message_instance.instantiate<dvs_msgs::EventArray>();

        // Check that the constant message fields haven't changed
        if (!(message_event_array->height == message_first_event_array->height &&
            message_event_array->width == message_first_event_array->width))
        {
            std::cout << "A field expected to be constant is not on topic " << connections[0]->topic << "\n";
            assert(false);
        }

        // Loop over events in the EventArray
        for (const dvs_msgs::Event& event : message_event_array->events)
        {
            x_coordinates_to_write.push_back(event.x);
            y_coordinates_to_write.push_back(event.y);
            timestamps_to_write.push_back(event.ts.toNSec());
            polarities_to_write.push_back(event.polarity);
            if (x_coordinates_to_write.size() == chunk_length_)
            {
                size_t index_message = message_count - chunk_length_ + 1;
                writeDataToDataset(path_x_coordinates_dataset, x_coordinates_to_write, index_message, chunk_length_);
                writeDataToDataset(path_y_coordinates_dataset, y_coordinates_to_write, index_message, chunk_length_);
                writeDataToDataset(path_timestamp_dataset, timestamps_to_write, index_message, chunk_length_);
                writeDataToDataset(path_polarity_dataset, polarities_to_write, index_message, chunk_length_);
                x_coordinates_to_write.clear();
                y_coordinates_to_write.clear();
                timestamps_to_write.clear();
                polarities_to_write.clear();
            }
            message_count++;

            if (message_count % chunk_length_ == 0)
            {
                std::cout << "\33[2K\rPercent completion: " << 100.0 * message_count / n_messages << std::flush;
            }
        }
    }

    // Write any remaining messages that didn't fit perfectly in a chunk
    if (x_coordinates_to_write.size() > 0)
    {
        size_t index_message = message_count - x_coordinates_to_write.size();
        writeDataToDataset(path_x_coordinates_dataset, x_coordinates_to_write, index_message,
            x_coordinates_to_write.size());
        writeDataToDataset(path_y_coordinates_dataset, y_coordinates_to_write, index_message,
            x_coordinates_to_write.size());
        writeDataToDataset(path_timestamp_dataset, timestamps_to_write, index_message, x_coordinates_to_write.size());
        writeDataToDataset(path_polarity_dataset, polarities_to_write, index_message, x_coordinates_to_write.size());
    }

    std::cout << "\33[2K\rPercent completion: " << 100.0 << std::flush << std::endl;
}