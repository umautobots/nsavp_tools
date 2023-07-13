#include "h5_io/h5_to_ros/H5ToRosPoseStampedWriter.h"

#include <cassert>
#include <iostream>

#include <rosbag/view.h>
#include <geometry_msgs/PoseStamped.h>

H5ToRosPoseStampedWriter::H5ToRosPoseStampedWriter(H5::Group& group)
    :   H5ToRosMessageWriter(group)
{}

void H5ToRosPoseStampedWriter::writeH5TopicGroup(
    const std::string topic,
    rosbag::Bag& bag)
{
    // Read in the data chunk by chunk
    size_t n_messages_read = 0;
    size_t message_count = 0;
    std::vector<uint64_t> data_timestamps;
    std::vector<double> data_positions;
    std::vector<double> data_quaternions;
    geometry_msgs::PoseStamped message_pose_stamped;
    while (message_count < n_messages_)
    {
        readNextChunkFromDataset("timestamps", data_timestamps, n_messages_read);
        readNextChunkFromDataset("positions", data_positions, n_messages_read);
        readNextChunkFromDataset("quaternions", data_quaternions, n_messages_read);

        // Write out the chunk data as separate pose stamped messages
        for (size_t i = 0; i < n_messages_read; i++)
        {
            message_pose_stamped.header.stamp.fromNSec(data_timestamps[i]);
            message_pose_stamped.pose.position.x = data_positions[i * 3];
            message_pose_stamped.pose.position.y = data_positions[i * 3 + 1];
            message_pose_stamped.pose.position.z = data_positions[i * 3 + 2];
            message_pose_stamped.pose.orientation.x = data_quaternions[i * 4];
            message_pose_stamped.pose.orientation.y = data_quaternions[i * 4 + 1];
            message_pose_stamped.pose.orientation.z = data_quaternions[i * 4 + 2];
            message_pose_stamped.pose.orientation.w = data_quaternions[i * 4 + 3];
            bag.write(topic, message_pose_stamped.header.stamp, message_pose_stamped);
        }

        message_count += n_messages_read;
        std::cout << "\33[2K\rPercent completion: " << 100.0 * message_count / n_messages_ << std::flush;
    }
    std::cout << "\33[2K\rPercent completion: " << 100.0 << std::flush << std::endl;
}
