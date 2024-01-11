#include "h5_io/h5_to_common/H5ToCommonCameraDiagnosticsWriter.h"

#include <cassert>
#include <iostream>
#include <fstream>
#include <iomanip>

H5ToCommonCameraDiagnosticsWriter::H5ToCommonCameraDiagnosticsWriter(H5::Group& group)
    :   H5ToCommonWriter(group)
{}

void H5ToCommonCameraDiagnosticsWriter::writeH5TopicGroup(const std::string path_output)
{
    // Initialize the CSV file
    std::ofstream file_camera_diagnostics;
    file_camera_diagnostics.open(path_output);
    file_camera_diagnostics << std::setprecision(17);

    // Write the hardware ID header and store the hardware ID
    file_camera_diagnostics << "hardware_id" << ", ";
    std::string hardware_id;
    readAttributeFromGroup("hardware_id", hardware_id);

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

    // Write headers for all remaining datasets
    file_camera_diagnostics << "request_timestamp" << ", ";
    file_camera_diagnostics << "response_timestamp" << ", ";
    for (const std::string name_float_dataset : names_float_dataset)
    {
        file_camera_diagnostics << name_float_dataset << ", ";
    }
    for (const std::string name_int_dataset : names_int_dataset)
    {
        file_camera_diagnostics << name_int_dataset << ", ";
    }
    for (const std::string name_bool_dataset : names_bool_dataset)
    {
        file_camera_diagnostics << name_bool_dataset << ", ";
    }
    for (const std::string name_string_dataset : names_string_dataset)
    {
        file_camera_diagnostics << name_string_dataset << ", ";
    }
    for (const std::string name_enum_dataset : names_enum_dataset)
    {
        file_camera_diagnostics << name_enum_dataset << ", ";
    }
    file_camera_diagnostics << "\n";

    // Read in the data one message at a time
    size_t message_count = 0;
    while (message_count < n_messages_)
    {
        file_camera_diagnostics << hardware_id << ", ";

        uint64_t request_stamp;
        readNextValueFrom1DDataset("request_timestamps", request_stamp);
        file_camera_diagnostics << request_stamp << ", ";

        uint64_t response_stamp;
        readNextValueFrom1DDataset("response_timestamps", response_stamp);
        file_camera_diagnostics << response_stamp << ", ";

        for (const std::string name_float_dataset : names_float_dataset)
        {
            double value;
            readNextValueFrom1DDataset(name_float_dataset, value);
            file_camera_diagnostics << value << ", ";
        }

        for (const std::string name_int_dataset : names_int_dataset)
        {
            int64_t value;
            readNextValueFrom1DDataset(name_int_dataset, value);
            file_camera_diagnostics << value << ", ";
        }

        for (const std::string name_bool_dataset : names_bool_dataset)
        {
            int64_t value;
            readNextValueFrom1DDataset(name_bool_dataset, value);
            file_camera_diagnostics << value << ", ";
        }

        for (const std::string name_string_dataset : names_string_dataset)
        {
            std::string value;
            readNextValueFrom1DDataset(name_string_dataset, value);
            file_camera_diagnostics << value << ", ";
        }

        for (const std::string name_enum_dataset : names_enum_dataset)
        {
            std::string value;
            readNextValueFrom1DDataset(name_enum_dataset, value);
            file_camera_diagnostics << value << ", ";
        }
        file_camera_diagnostics << "\n";

        message_count++;
        std::cout << "\33[2K\rPercent completion: " << 100.0 * message_count / n_messages_ << std::flush;
    }
    std::cout << "\33[2K\rPercent completion: " << 100.0 << std::flush << std::endl;
}
