#ifndef H5TOCOMMONIMAGEWRITER_H
#define H5TOCOMMONIMAGEWRITER_H

#include "h5_io/h5_to_common/H5ToCommonWriter.h"

#include <sstream>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/mat.hpp>

class H5ToCommonImageWriter : public H5ToCommonWriter
{
public:
    H5ToCommonImageWriter(H5::Group& group);

    /************************ public member functions ************************/

    void writeH5TopicGroup(const std::string path_output);

private:
    /************************ private member functions ************************/

    template<typename T>
    void writeDataByEncoding(const std::string path_output)
    {
        // Read in image attributes
        uint32_t height;
        readAttributeFromGroup("height", height);
        uint32_t width;
        readAttributeFromGroup("width", width);
        uint8_t is_bigendian;
        readAttributeFromGroup("is_bigendian", is_bigendian);
        uint32_t step;
        readAttributeFromGroup("step", step);

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
                std::ostringstream timestamp_string_stream;
                timestamp_string_stream << data_timestamps[i];
                std::string timestamp_string(timestamp_string_stream.str());
                std::string filepath_image = path_output + timestamp_string + ".png";

                cv::Mat_<T> image(height, width, &data_images[i * height * width], step);
                cv::imwrite(filepath_image, image);
            }

            message_count += n_messages_read;
            std::cout << "\33[2K\rPercent completion: " << 100.0 * message_count / n_messages_ << std::flush;
        }
        std::cout << "\33[2K\rPercent completion: " << 100.0 << std::flush << std::endl;
    }
};

#endif //H5TOCOMMONIMAGEWRITER_H