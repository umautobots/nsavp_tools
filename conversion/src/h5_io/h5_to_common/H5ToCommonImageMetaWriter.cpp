#include "h5_io/h5_to_common/H5ToCommonImageMetaWriter.h"

#include <cassert>
#include <iostream>
#include <fstream>
#include <iomanip>

H5ToCommonImageMetaWriter::H5ToCommonImageMetaWriter(H5::Group& group)
    :   H5ToCommonWriter(group)
{}

void H5ToCommonImageMetaWriter::writeH5TopicGroup(const std::string path_output)
{
    // Initialize the CSV file
    std::ofstream file_image_meta;
    file_image_meta.open(path_output);
    file_image_meta << std::setprecision(17);

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

    // Write headers for all remaining datasets
    file_image_meta << "hardware_timestamp" << ", ";
    file_image_meta << "hardware_index" << ", ";
    file_image_meta << "driver_timestamp" << ", ";
    file_image_meta << "driver_index" << ", ";
    file_image_meta << "image_crc32_checksum" << ", ";
    for (const std::string name_float_dataset : names_float_dataset)
    {
        file_image_meta << name_float_dataset << ", ";
    }
    for (const std::string name_int_dataset : names_int_dataset)
    {
        file_image_meta << name_int_dataset << ", ";
    }
    for (const std::string name_string_dataset : names_string_dataset)
    {
        file_image_meta << name_string_dataset << ", ";
    }
    file_image_meta << "\n";

    // Read in the data one message at a time
    size_t message_count = 0;
    while (message_count < n_messages_)
    {
        uint64_t hardware_stamp;
        readNextValueFrom1DDataset("hardware_timestamps", hardware_stamp);
        file_image_meta << hardware_stamp << ", ";
        uint32_t hardware_index;
        readNextValueFrom1DDataset("hardware_indices", hardware_index);
        file_image_meta << hardware_index << ", ";

        uint64_t driver_stamp;
        readNextValueFrom1DDataset("driver_timestamps", driver_stamp);
        file_image_meta << driver_stamp << ", ";
        uint32_t driver_index;
        readNextValueFrom1DDataset("driver_indices", driver_index);
        file_image_meta << driver_index << ", ";

        uint32_t image_crc32_checksum;
        readNextValueFrom1DDataset("image_crc32_checksums", image_crc32_checksum);
        file_image_meta << image_crc32_checksum << ", ";

        for (const std::string name_float_dataset : names_float_dataset)
        {
            double value;
            readNextValueFrom1DDataset(name_float_dataset, value);
            file_image_meta << value << ", ";
        }

        for (const std::string name_int_dataset : names_int_dataset)
        {
            int64_t value;
            readNextValueFrom1DDataset(name_int_dataset, value);
            file_image_meta << value << ", ";
        }

        for (const std::string name_string_dataset : names_string_dataset)
        {
            std::string value;
            readNextValueFrom1DDataset(name_string_dataset, value);
            file_image_meta << value << ", ";
        }
        file_image_meta << "\n";

        message_count++;
        std::cout << "\33[2K\rPercent completion: " << 100.0 * message_count / n_messages_ << std::flush;
    }
    std::cout << "\33[2K\rPercent completion: " << 100.0 << std::flush << std::endl;
}
