#ifndef H5TOROSIMAGEMETAWRITER_H
#define H5TOROSIMAGEMETAWRITER_H

#include "h5_io/h5_to_ros/H5ToRosMessageWriter.h"

class H5ToRosImageMetaWriter : public H5ToRosMessageWriter
{
public:
    H5ToRosImageMetaWriter(H5::Group& group);

    /************************ public member functions ************************/

    void writeH5TopicGroup(
        const std::string topic,
        rosbag::Bag& bag);
};

#endif //H5TOROSIMAGEMETAWRITER_H