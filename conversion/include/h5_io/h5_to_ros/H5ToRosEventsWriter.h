#ifndef H5TOROSEVENTSWRITER_H
#define H5TOROSEVENTSWRITER_H

#include "h5_io/h5_to_ros/H5ToRosMessageWriter.h"

class H5ToRosEventsWriter : public H5ToRosMessageWriter
{
public:
    H5ToRosEventsWriter(
        H5::Group& group,
        const double rate_message);

    /************************ public member functions ************************/

    void writeH5TopicGroup(
        const std::string topic,
        rosbag::Bag& bag);

private:
    /************************ private member variables ************************/

    const uint64_t period_message_nanoseconds_;
};

#endif //H5TOROSEVENTSWRITER_H