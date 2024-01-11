#include "h5_io/h5_to_common/H5ToCommonEventsWriter.h"

#include <cassert>
#include <iostream>
#include <fstream>
#include <iomanip>

H5ToCommonEventsWriter::H5ToCommonEventsWriter(H5::Group& group)
    :   H5ToCommonWriter(group)
{}

void H5ToCommonEventsWriter::writeH5TopicGroup(const std::string path_output)
{
    // Initialize the CSV file
    std::ofstream file_events;
    file_events.open(path_output);
    file_events << std::setprecision(17);
    file_events << "x_coordinate, y_coordinate, timestamp, polarity, \n";

    // Read in the data chunk by chunk
    size_t n_messages_read = 0;
    size_t message_count = 0;
    std::vector<uint16_t> data_x_coordinates;
    std::vector<uint16_t> data_y_coordinates;
    std::vector<uint64_t> data_timestamps;
    std::vector<uint8_t> data_polarities;
    while (message_count < n_messages_)
    {
        readNextChunkFromDataset("x_coordinates", data_x_coordinates, n_messages_read);
        readNextChunkFromDataset("y_coordinates", data_y_coordinates, n_messages_read);
        readNextChunkFromDataset("timestamps", data_timestamps, n_messages_read);
        readNextChunkFromDataset("polarities", data_polarities, n_messages_read);

        for (size_t i = 0; i < n_messages_read; i++)
        {
            file_events << data_x_coordinates[i] << ", ";
            file_events << data_y_coordinates[i] << ", ";
            file_events << data_timestamps[i] << ", ";
            file_events << int(data_polarities[i]) << ", ";
            file_events << "\n";
        }

        message_count += n_messages_read;
        std::cout << "\33[2K\rPercent completion: " << 100.0 * message_count / n_messages_ << std::flush;
    }
    std::cout << "\33[2K\rPercent completion: " << 100.0 << std::flush << std::endl;
}
