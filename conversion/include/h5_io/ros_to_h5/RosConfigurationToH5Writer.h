#ifndef ROSCONFIGURATOINTOH5WRITER_H
#define ROSCONFIGURATOINTOH5WRITER_H

#include "h5_io/ros_to_h5/RosMessageToH5Writer.h"

namespace rosbag
{
    class View;
}

class RosConfigurationToH5Writer : public RosMessageToH5Writer
{
public:
    RosConfigurationToH5Writer(
        const int chunk_length,
        const int chunk_cache_size,
        const bool shuffle_enable,
        const std::shared_ptr<const CompressionMethod> compression_method,
        const std::shared_ptr<H5::H5File> h5_file);

    /************************ public member functions ************************/

    void writeRosbagTopicView(
        const std::string path_group,
        rosbag::View& view_topic);
};

#endif //ROSCONFIGURATOINTOH5WRITER_H