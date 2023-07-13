#ifndef H5TOROSCONFIGURATIONWRITER_H
#define H5TOROSCONFIGURATIONWRITER_H

#include "h5_io/h5_to_ros/H5ToRosMessageWriter.h"

class H5ToRosConfigurationWriter : public H5ToRosMessageWriter
{
public:
    H5ToRosConfigurationWriter(H5::Group& group);

    /************************ public member functions ************************/

    void writeH5TopicGroup(
        const std::string topic,
        rosbag::Bag& bag);
};

#endif //H5TOROSCONFIGURATIONWRITER_H