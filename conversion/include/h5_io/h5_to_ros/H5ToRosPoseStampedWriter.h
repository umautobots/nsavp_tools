#ifndef H5TOROSPOSESTAMPEDWRITER_H
#define H5TOROSPOSESTAMPEDWRITER_H

#include "h5_io/h5_to_ros/H5ToRosMessageWriter.h"

class H5ToRosPoseStampedWriter : public H5ToRosMessageWriter
{
public:
    H5ToRosPoseStampedWriter(H5::Group& group);

    /************************ public member functions ************************/

    void writeH5TopicGroup(
        const std::string topic,
        rosbag::Bag& bag);
};

#endif //H5TOROSPOSESTAMPEDWRITER_H