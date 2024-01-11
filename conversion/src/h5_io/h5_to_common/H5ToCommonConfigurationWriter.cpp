#include "h5_io/h5_to_common/H5ToCommonConfigurationWriter.h"

#include <cassert>
#include <iostream>
#include <fstream>
#include <iomanip>

H5ToCommonConfigurationWriter::H5ToCommonConfigurationWriter(H5::Group& group)
    :   H5ToCommonWriter(group)
{}

void H5ToCommonConfigurationWriter::writeH5TopicGroup(const std::string path_output)
{
    // Retrieve all attributes
    std::vector<std::string> names_attribute;
    H5Aiterate(group_.getLocId(), H5_INDEX_NAME, H5_ITER_NATIVE, NULL, H5AIterateCallbackFunction, &names_attribute);

    // If there are no attributes, there is no configuration information to process
    if (names_attribute.size() == 0)
    {
        return;
    }
    std::cout << "Writing out configuration information to " << path_output << "\n";

    // Initialize the CSV file
    std::ofstream file_configuration;
    file_configuration.open(path_output);
    file_configuration << std::setprecision(17);
    file_configuration << "Name, ";
    file_configuration << "Value, ";
    file_configuration << "\n";

    // Read name-value fields
    for (const std::string name_attribute : names_attribute)
    {
        H5::Attribute attribute = group_.openAttribute(name_attribute);
        if (attribute.getDataType() == H5::PredType::STD_I64BE || attribute.getDataType() == H5::PredType::STD_I64LE)
        {
            if (name_attribute.find(" (bool)") != std::string::npos)
            {
                int64_t value;
                readAttributeFromGroup(name_attribute, value);
                file_configuration << name_attribute << ", ";
                file_configuration << value << ", ";
                file_configuration << "\n";
            }
            else
            {
                int64_t value;
                readAttributeFromGroup(name_attribute, value);
                file_configuration << name_attribute << ", ";
                file_configuration << value << ", ";
                file_configuration << "\n";
            }
        }
        else if (attribute.getDataType() == H5::PredType::IEEE_F64BE || attribute.getDataType() == H5::PredType::IEEE_F64LE)
        {
            double value;
            readAttributeFromGroup(name_attribute, value);
            file_configuration << name_attribute << ", ";
            file_configuration << value << ", ";
            file_configuration << "\n";
        }
        else if (attribute.getDataType() == H5::StrType(H5::PredType::C_S1, H5T_VARIABLE))
        {
            if (name_attribute.find(" (enum)") != std::string::npos)
            {
                std::string value;
                readAttributeFromGroup(name_attribute, value);
                file_configuration << name_attribute << ", ";
                file_configuration << value << ", ";
                file_configuration << "\n";
            }
            else
            {
                std::string value;
                readAttributeFromGroup(name_attribute, value);
                file_configuration << name_attribute << ", ";
                file_configuration << value << ", ";
                file_configuration << "\n";
            }
        }
    }

    // Read fixed attributes
    uint64_t response_stamp;
    readAttributeFromGroup("response_timestamp_nanoseconds", response_stamp);
    file_configuration << "response_timestamp, ";
    file_configuration << response_stamp << ", ";
    file_configuration << "\n";

    uint64_t request_stamp;
    readAttributeFromGroup("request_timestamp_nanoseconds", request_stamp);
    file_configuration << "request_timestamp, ";
    file_configuration << request_stamp << ", ";
    file_configuration << "\n";

    std::string hardware_id;
    readAttributeFromGroup("hardware_id", hardware_id);
    file_configuration << "hardware_id, ";
    file_configuration << hardware_id << ", ";
    file_configuration << "\n";
}
