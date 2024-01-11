#include "h5_io/h5_to_ros/H5ToRosEventsWriter.h"

#include <cassert>
#include <iostream>

#include <rosbag/view.h>
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>

H5ToRosEventsWriter::H5ToRosEventsWriter(
    H5::Group& group,
    const double rate_message)
    :   H5ToRosMessageWriter(group),
        period_message_nanoseconds_((1.0 / rate_message) * 1e9)
{}

void H5ToRosEventsWriter::writeH5TopicGroup(
    const std::string topic,
    rosbag::Bag& bag)
{
    // Initialize the event array message with constant fields
    dvs_msgs::EventArray message_event_array;
    readAttributeFromGroup("height", message_event_array.height);
    readAttributeFromGroup("width", message_event_array.width);

    // Read in the data chunk by chunk
    bool is_first_message = true;
    uint64_t timestamp_last_message;
    size_t n_messages_read = 0;
    size_t message_count = 0;
    std::vector<uint16_t> data_x_coordinates;
    std::vector<uint16_t> data_y_coordinates;
    std::vector<uint64_t> data_timestamps;
    std::vector<uint8_t> data_polarities;
    dvs_msgs::Event message_event;
    while (message_count < n_messages_)
    {
        readNextChunkFromDataset("x_coordinates", data_x_coordinates, n_messages_read);
        readNextChunkFromDataset("y_coordinates", data_y_coordinates, n_messages_read);
        readNextChunkFromDataset("timestamps", data_timestamps, n_messages_read);
        readNextChunkFromDataset("polarities", data_polarities, n_messages_read);

        if (is_first_message)
        {
            timestamp_last_message = data_timestamps[0];
            is_first_message = false;
        }

        // Convert each message to a ROS Event message and write out EventArrays at the requested rate
        for (size_t i = 0; i < n_messages_read; i++)
        {
            if (data_timestamps[i] > (timestamp_last_message + period_message_nanoseconds_))
            {
                message_event_array.header.stamp.fromNSec(timestamp_last_message + period_message_nanoseconds_);
                bag.write(topic, message_event_array.header.stamp, message_event_array);
                timestamp_last_message += period_message_nanoseconds_;
                message_event_array.events.clear();
            }
            message_event.x = data_x_coordinates[i];
            message_event.y = data_y_coordinates[i];
            message_event.ts.fromNSec(data_timestamps[i]);
            message_event.polarity = data_polarities[i];
            message_event_array.events.push_back(message_event);
        }

        message_count += n_messages_read;
        std::cout << "\33[2K\rPercent completion: " << 100.0 * message_count / n_messages_ << std::flush;
    }

    // Write out final EventArray
    if (message_event_array.events.size() > 0)
    {
        message_event_array.header.stamp = message_event_array.events.back().ts;
        bag.write(topic, message_event_array.header.stamp, message_event_array);
    }

    std::cout << "\33[2K\rPercent completion: " << 100.0 << std::flush << std::endl;
}
