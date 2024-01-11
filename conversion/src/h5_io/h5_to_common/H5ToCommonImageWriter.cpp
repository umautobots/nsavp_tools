#include "h5_io/h5_to_common/H5ToCommonImageWriter.h"

#include <cassert>
#include <iostream>
#include <filesystem>

#include <sensor_msgs/image_encodings.h>

H5ToCommonImageWriter::H5ToCommonImageWriter(H5::Group& group)
    :   H5ToCommonWriter(group)
{}

void H5ToCommonImageWriter::writeH5TopicGroup(const std::string path_output)
{
    // Create the output image folder
    std::filesystem::create_directories(path_output);

    // Write according to the encoding
    std::string encoding;
    readAttributeFromGroup("encoding", encoding);
    if (encoding == sensor_msgs::image_encodings::MONO8 || encoding == sensor_msgs::image_encodings::BAYER_RGGB8)
    {
        writeDataByEncoding<uint8_t>(path_output);
    }
    else if (encoding == sensor_msgs::image_encodings::MONO16)
    {
        writeDataByEncoding<uint16_t>(path_output);
    }
    else
    {
        std::cout << "Image message encoding currently not supported\n";
        assert(false);
    }
}