#include "h5_io/h5_to_ros/H5ToRosCameraDiagnosticsWriter.h"

#include <cassert>
#include <iostream>

#include <rosbag/view.h>
#include <image_meta_msgs/CameraDiagnostics.h>
#include <image_meta_msgs/NameValueFloat.h>
#include <image_meta_msgs/NameValueInt.h>
#include <image_meta_msgs/NameValueString.h>

H5ToRosCameraDiagnosticsWriter::H5ToRosCameraDiagnosticsWriter(H5::Group& group)
    :   H5ToRosMessageWriter(group)
{}

void H5ToRosCameraDiagnosticsWriter::writeH5TopicGroup(
    const std::string topic,
    rosbag::Bag& bag)
{
    // Initialize the camera diagnostic message with the hardware ID
    image_meta_msgs::CameraDiagnostics message_camera_diagnostics;
    readAttributeFromGroup("hardware_id", message_camera_diagnostics.hardware_id);

    // Find name-value fields
    std::vector<std::string> names_dataset;
    H5Literate(group_.getLocId(), H5_INDEX_NAME, H5_ITER_NATIVE, NULL, H5LIterateCallbackFunction, &names_dataset);
    std::vector<std::string> names_float_dataset, names_int_dataset, names_bool_dataset, names_string_dataset,
        names_enum_dataset;
    for (const std::string name_dataset : names_dataset)
    {
        H5::DataSet dataset = group_.openDataSet(name_dataset);
        if (dataset.getDataType() == H5::PredType::STD_I64BE || dataset.getDataType() == H5::PredType::STD_I64LE)
        {
            if (name_dataset.find(" (bool)") != std::string::npos)
            {
                names_bool_dataset.push_back(name_dataset);
            }
            else
            {
                names_int_dataset.push_back(name_dataset);
            }
        }
        else if (dataset.getDataType() == H5::PredType::IEEE_F64BE || dataset.getDataType() == H5::PredType::IEEE_F64LE)
        {
            names_float_dataset.push_back(name_dataset);
        }
        else if (dataset.getDataType() == H5::StrType(H5::PredType::C_S1, H5T_VARIABLE))
        {
            if (name_dataset.find(" (enum)") != std::string::npos)
            {
                names_enum_dataset.push_back(name_dataset);
            }
            else
            {
                names_string_dataset.push_back(name_dataset);
            }
        }
    }

    // Read in the data one message at a time
    size_t message_count = 0;
    while (message_count < n_messages_)
    {
        uint64_t request_stamp;
        readNextValueFrom1DDataset("request_timestamps", request_stamp);
        message_camera_diagnostics.request_stamp.fromNSec(request_stamp);

        uint64_t response_stamp;
        readNextValueFrom1DDataset("response_timestamps", response_stamp);
        message_camera_diagnostics.header.stamp.fromNSec(response_stamp);

        for (const std::string name_float_dataset : names_float_dataset)
        {
            double value;
            readNextValueFrom1DDataset(name_float_dataset, value);
            image_meta_msgs::NameValueFloat message_name_value_float;
            message_name_value_float.name = name_float_dataset;
            message_name_value_float.value = value;
            message_camera_diagnostics.name_value_floats.push_back(message_name_value_float);
        }

        for (const std::string name_int_dataset : names_int_dataset)
        {
            int64_t value;
            readNextValueFrom1DDataset(name_int_dataset, value);
            image_meta_msgs::NameValueInt message_name_value_int;
            message_name_value_int.name = name_int_dataset;
            message_name_value_int.value = value;
            message_camera_diagnostics.name_value_ints.push_back(message_name_value_int);
        }

        for (const std::string name_bool_dataset : names_bool_dataset)
        {
            int64_t value;
            readNextValueFrom1DDataset(name_bool_dataset, value);
            image_meta_msgs::NameValueInt message_name_value_int;
            message_name_value_int.name = name_bool_dataset.substr(0, name_bool_dataset.find(" (bool)"));
            message_name_value_int.value = value;
            message_camera_diagnostics.name_value_bools.push_back(message_name_value_int);
        }

        for (const std::string name_string_dataset : names_string_dataset)
        {
            std::string value;
            readNextValueFrom1DDataset(name_string_dataset, value);
            image_meta_msgs::NameValueString message_name_value_string;
            message_name_value_string.name = name_string_dataset;
            message_name_value_string.value = value;
            message_camera_diagnostics.name_value_strings.push_back(message_name_value_string);
        }

        for (const std::string name_enum_dataset : names_enum_dataset)
        {
            std::string value;
            readNextValueFrom1DDataset(name_enum_dataset, value);
            image_meta_msgs::NameValueString message_name_value_string;
            message_name_value_string.name = name_enum_dataset.substr(0, name_enum_dataset.find(" (enum)"));
            message_name_value_string.value = value;
            message_camera_diagnostics.name_value_enums.push_back(message_name_value_string);
        }

        bag.write(topic, message_camera_diagnostics.header.stamp, message_camera_diagnostics);

        message_camera_diagnostics.name_value_floats.clear();
        message_camera_diagnostics.name_value_ints.clear();
        message_camera_diagnostics.name_value_bools.clear();
        message_camera_diagnostics.name_value_strings.clear();
        message_camera_diagnostics.name_value_enums.clear();

        message_count++;
        std::cout << "\33[2K\rPercent completion: " << 100.0 * message_count / n_messages_ << std::flush;
    }
    std::cout << "\33[2K\rPercent completion: " << 100.0 << std::flush << std::endl;
}
