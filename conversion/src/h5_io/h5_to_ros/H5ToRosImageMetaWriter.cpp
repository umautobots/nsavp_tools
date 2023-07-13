#include "h5_io/h5_to_ros/H5ToRosImageMetaWriter.h"

#include <cassert>
#include <iostream>

#include <rosbag/view.h>
#include <image_meta_msgs/ImageMeta.h>
#include <image_meta_msgs/NameValueFloat.h>
#include <image_meta_msgs/NameValueInt.h>
#include <image_meta_msgs/NameValueString.h>

H5ToRosImageMetaWriter::H5ToRosImageMetaWriter(H5::Group& group)
    :   H5ToRosMessageWriter(group)
{}

void H5ToRosImageMetaWriter::writeH5TopicGroup(
    const std::string topic,
    rosbag::Bag& bag)
{
    // Find name-value fields
    std::vector<std::string> names_dataset;
    H5Literate(group_.getLocId(), H5_INDEX_NAME, H5_ITER_NATIVE, NULL, H5LIterateCallbackFunction, &names_dataset);
    std::vector<std::string> names_float_dataset, names_int_dataset, names_string_dataset;
    for (const std::string name_dataset : names_dataset)
    {
        H5::DataSet dataset = group_.openDataSet(name_dataset);
        if (dataset.getDataType() == H5::PredType::STD_I64BE || dataset.getDataType() == H5::PredType::STD_I64LE)
        {
            names_int_dataset.push_back(name_dataset);
        }
        else if (dataset.getDataType() == H5::PredType::IEEE_F64BE || dataset.getDataType() == H5::PredType::IEEE_F64LE)
        {
            names_float_dataset.push_back(name_dataset);
        }
        else if (dataset.getDataType() == H5::StrType(H5::PredType::C_S1, H5T_VARIABLE))
        {
            names_string_dataset.push_back(name_dataset);
        }
    }

    // Read in the data one message at a time
    size_t message_count = 0;
    while (message_count < n_messages_)
    {
        image_meta_msgs::ImageMeta message_image_meta;

        uint64_t hardware_stamp;
        readNextValueFrom1DDataset("hardware_timestamps", hardware_stamp);
        message_image_meta.hardware_header.stamp.fromNSec(hardware_stamp);
        readNextValueFrom1DDataset("hardware_indices", message_image_meta.hardware_header.seq);

        uint64_t driver_stamp;
        readNextValueFrom1DDataset("driver_timestamps", driver_stamp);
        message_image_meta.driver_header.stamp.fromNSec(driver_stamp);
        readNextValueFrom1DDataset("driver_indices", message_image_meta.driver_header.seq);

        readNextValueFrom1DDataset("image_crc32_checksums", message_image_meta.crc32);

        for (const std::string name_float_dataset : names_float_dataset)
        {
            double value;
            readNextValueFrom1DDataset(name_float_dataset, value);
            image_meta_msgs::NameValueFloat message_name_value_float;
            message_name_value_float.name = name_float_dataset;
            message_name_value_float.value = value;
            message_image_meta.name_value_floats.push_back(message_name_value_float);
        }

        for (const std::string name_int_dataset : names_int_dataset)
        {
            int64_t value;
            readNextValueFrom1DDataset(name_int_dataset, value);
            image_meta_msgs::NameValueInt message_name_value_int;
            message_name_value_int.name = name_int_dataset;
            message_name_value_int.value = value;
            message_image_meta.name_value_ints.push_back(message_name_value_int);
        }

        for (const std::string name_string_dataset : names_string_dataset)
        {
            std::string value;
            readNextValueFrom1DDataset(name_string_dataset, value);
            image_meta_msgs::NameValueString message_name_value_string;
            message_name_value_string.name = name_string_dataset;
            message_name_value_string.value = value;
            message_image_meta.name_value_strings.push_back(message_name_value_string);
        }

        bag.write(topic, message_image_meta.driver_header.stamp, message_image_meta);

        message_count++;
        std::cout << "\33[2K\rPercent completion: " << 100.0 * message_count / n_messages_ << std::flush;
    }
    std::cout << "\33[2K\rPercent completion: " << 100.0 << std::flush << std::endl;
}
