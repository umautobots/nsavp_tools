#ifndef H5TOROSCAMERADIAGNOSTICSWRITER_H
#define H5TOROSCAMERADIAGNOSTICSWRITER_H

#include "h5_io/h5_to_ros/H5ToRosMessageWriter.h"

class H5ToRosCameraDiagnosticsWriter : public H5ToRosMessageWriter
{
public:
    H5ToRosCameraDiagnosticsWriter(H5::Group& group);

    /************************ public member functions ************************/

    void writeH5TopicGroup(
        const std::string topic,
        rosbag::Bag& bag);
};

#endif //H5TOROSCAMERADIAGNOSTICSWRITER_H