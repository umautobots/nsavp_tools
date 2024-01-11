#include "h5_io/h5_to_common/H5ToCommonPoseStampedWriter.h"

#include <cassert>
#include <iostream>
#include <fstream>
#include <iomanip>

H5ToCommonPoseStampedWriter::H5ToCommonPoseStampedWriter(H5::Group& group)
    :   H5ToCommonWriter(group)
{}

void H5ToCommonPoseStampedWriter::writeH5TopicGroup(const std::string path_output)
{
    // Initialize the CSV file
    std::ofstream file_poses;
    file_poses.open(path_output);
    file_poses << std::setprecision(17);
    file_poses << "timestamp, x_position, y_position, z_position, x_quaternion, y_quaternion, z_quaternion, "
        "w_quaternion, \n";

    // Read in the data chunk by chunk
    size_t n_messages_read = 0;
    size_t message_count = 0;
    std::vector<uint64_t> data_timestamps;
    std::vector<double> data_positions;
    std::vector<double> data_quaternions;
    while (message_count < n_messages_)
    {
        readNextChunkFromDataset("timestamps", data_timestamps, n_messages_read);
        readNextChunkFromDataset("positions", data_positions, n_messages_read);
        readNextChunkFromDataset("quaternions", data_quaternions, n_messages_read);

        for (size_t i = 0; i < n_messages_read; i++)
        {
            file_poses << data_timestamps[i] << ", ";
            file_poses << data_positions[i * 3] << ", ";
            file_poses << data_positions[i * 3 + 1] << ", ";
            file_poses << data_positions[i * 3 + 2] << ", ";
            file_poses << data_quaternions[i * 4] << ", ";
            file_poses << data_quaternions[i * 4 + 1] << ", ";
            file_poses << data_quaternions[i * 4 + 2] << ", ";
            file_poses << data_quaternions[i * 4 + 3] << ", ";
            file_poses << "\n";
        }

        message_count += n_messages_read;
        std::cout << "\33[2K\rPercent completion: " << 100.0 * message_count / n_messages_ << std::flush;
    }
    std::cout << "\33[2K\rPercent completion: " << 100.0 << std::flush << std::endl;
}
