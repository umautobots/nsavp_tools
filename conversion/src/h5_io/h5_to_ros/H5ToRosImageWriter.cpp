#include "h5_io/h5_to_ros/H5ToRosImageWriter.h"

#include <cassert>
#include <iostream>

#include <rosbag/view.h>
#include <sensor_msgs/image_encodings.h>

H5ToRosImageWriter::H5ToRosImageWriter(H5::Group& group)
    :   H5ToRosMessageWriter(group)
{}

void H5ToRosImageWriter::writeH5TopicGroup(
    const std::string topic,
    rosbag::Bag& bag)
{
    // Initialize the image message with constant fields
    sensor_msgs::Image message_image;
    readAttributeFromGroup("height", message_image.height);
    readAttributeFromGroup("width", message_image.width);
    readAttributeFromGroup("encoding", message_image.encoding);
    readAttributeFromGroup("is_bigendian", message_image.is_bigendian);
    readAttributeFromGroup("step", message_image.step);

    // Write according to the encoding
    if (message_image.encoding == sensor_msgs::image_encodings::MONO8 ||
        message_image.encoding == sensor_msgs::image_encodings::BAYER_RGGB8)
    {
        writeDataByEncoding<uint8_t>(topic, bag, message_image);
    }
    else if (message_image.encoding == sensor_msgs::image_encodings::MONO16)
    {
        writeDataByEncoding<uint16_t>(topic, bag, message_image);
    }
    else
    {
        std::cout << "Image message encoding currently not supported\n";
        assert(false);
    }
}