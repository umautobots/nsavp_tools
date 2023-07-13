#ifndef H5TOROSIMAGEWRITER_H
#define H5TOROSIMAGEWRITER_H

#include "h5_io/h5_to_ros/H5ToRosMessageWriter.h"

#include <sensor_msgs/Image.h>
#include <sensor_msgs/fill_image.h>

class H5ToRosImageWriter : public H5ToRosMessageWriter
{
public:
    H5ToRosImageWriter(H5::Group& group);

    /************************ public member functions ************************/

    void writeH5TopicGroup(
        const std::string topic,
        rosbag::Bag& bag);

private:
    /************************ private member functions ************************/

    template<typename T>
    void writeDataByEncoding(
        const std::string topic,
        rosbag::Bag& bag,
        sensor_msgs::Image& message_image)
    {
        // Read in the data chunk by chunk
        size_t n_messages_read = 0;
        size_t message_count = 0;
        std::vector<uint64_t> data_timestamps;
        std::vector<T> data_images;
        while(message_count < n_messages_)
        {
            readNextChunkFromDataset("timestamps", data_timestamps, n_messages_read);
            readNextChunkFromDataset("images", data_images, n_messages_read);

            // Write out the chunk data as separate image messages
            for (size_t i = 0; i < n_messages_read; i++)
            {
                sensor_msgs::fillImage(message_image, message_image.encoding, message_image.height, message_image.width,
                    message_image.step, &data_images[i * message_image.height * message_image.width]);
                message_image.header.stamp.fromNSec(data_timestamps[i]);
                bag.write(topic, message_image.header.stamp, message_image);
            }

            message_count += n_messages_read;
            std::cout << "\33[2K\rPercent completion: " << 100.0 * message_count / n_messages_ << std::flush;
        }
        std::cout << "\33[2K\rPercent completion: " << 100.0 << std::flush << std::endl;
    }
};

#endif //H5TOROSIMAGEWRITER_H