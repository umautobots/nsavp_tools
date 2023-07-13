#include "h5_io/h5_to_ros/H5ToRosConfigurationWriter.h"

#include <cassert>
#include <iostream>

#include <rosbag/view.h>
#include <image_meta_msgs/CameraDiagnostics.h>
#include <image_meta_msgs/NameValueFloat.h>
#include <image_meta_msgs/NameValueInt.h>
#include <image_meta_msgs/NameValueString.h>

H5ToRosConfigurationWriter::H5ToRosConfigurationWriter(H5::Group& group)
    :   H5ToRosMessageWriter(group)
{}

void H5ToRosConfigurationWriter::writeH5TopicGroup(
    const std::string topic,
    rosbag::Bag& bag)
{
    // Retrieve all attributes
    std::vector<std::string> names_attribute;
    H5Aiterate(group_.getLocId(), H5_INDEX_NAME, H5_ITER_NATIVE, NULL, H5AIterateCallbackFunction, &names_attribute);

    // If there are no attributes, there is no configuration information to process
    if (names_attribute.size() == 0)
    {
        return;
    }
    std::cout << "Writing out configuration information for " << topic << "\n";

    // Read name-value fields
    image_meta_msgs::CameraDiagnostics message_camera_diagnostics;
    for (const std::string name_attribute : names_attribute)
    {
        H5::Attribute attribute = group_.openAttribute(name_attribute);
        if (attribute.getDataType() == H5::PredType::STD_I64BE || attribute.getDataType() == H5::PredType::STD_I64LE)
        {
            if (name_attribute.find(" (bool)") != std::string::npos)
            {
                int64_t value;
                readAttributeFromGroup(name_attribute, value);
                image_meta_msgs::NameValueInt message_name_value_int;
                message_name_value_int.name = name_attribute.substr(0, name_attribute.find(" (bool)"));
                message_name_value_int.value = value;
                message_camera_diagnostics.name_value_bools.push_back(message_name_value_int);
            }
            else
            {
                int64_t value;
                readAttributeFromGroup(name_attribute, value);
                image_meta_msgs::NameValueInt message_name_value_int;
                message_name_value_int.name = name_attribute;
                message_name_value_int.value = value;
                message_camera_diagnostics.name_value_ints.push_back(message_name_value_int);
            }
        }
        else if (attribute.getDataType() == H5::PredType::IEEE_F64BE || attribute.getDataType() == H5::PredType::IEEE_F64LE)
        {
            double value;
            readAttributeFromGroup(name_attribute, value);
            image_meta_msgs::NameValueFloat message_name_value_float;
            message_name_value_float.name = name_attribute;
            message_name_value_float.value = value;
            message_camera_diagnostics.name_value_floats.push_back(message_name_value_float);
        }
        else if (attribute.getDataType() == H5::StrType(H5::PredType::C_S1, H5T_VARIABLE))
        {
            if (name_attribute.find(" (enum)") != std::string::npos)
            {
                std::string value;
                readAttributeFromGroup(name_attribute, value);
                image_meta_msgs::NameValueString message_name_value_string;
                message_name_value_string.name = name_attribute.substr(0, name_attribute.find(" (enum)"));
                message_name_value_string.value = value;
                message_camera_diagnostics.name_value_enums.push_back(message_name_value_string);
            }
            else
            {
                std::string value;
                readAttributeFromGroup(name_attribute, value);
                image_meta_msgs::NameValueString message_name_value_string;
                message_name_value_string.name = name_attribute;
                message_name_value_string.value = value;
                message_camera_diagnostics.name_value_strings.push_back(message_name_value_string);
            }
        }
    }

    // Read fixed attributes
    uint64_t response_stamp;
    readAttributeFromGroup("response_timestamp_nanoseconds", response_stamp);
    message_camera_diagnostics.header.stamp.fromNSec(response_stamp);

    uint64_t request_stamp;
    readAttributeFromGroup("request_timestamp_nanoseconds", request_stamp);
    message_camera_diagnostics.request_stamp.fromNSec(request_stamp);

    readAttributeFromGroup("hardware_id", message_camera_diagnostics.hardware_id);

    uint64_t timestamp_rosbag_int;
    readAttributeFromGroup("rosbag_start_time", timestamp_rosbag_int);
    ros::Time timestamp_rosbag;
    timestamp_rosbag.fromNSec(timestamp_rosbag_int);
    bag.write("/" + topic + "/config", timestamp_rosbag, message_camera_diagnostics);
}
