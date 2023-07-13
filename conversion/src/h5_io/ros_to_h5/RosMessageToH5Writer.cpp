#include "h5_io/ros_to_h5/RosMessageToH5Writer.h"

RosMessageToH5Writer::RosMessageToH5Writer(
    const int chunk_length,
    const int chunk_cache_size,
    const bool shuffle_enable,
    const std::shared_ptr<const CompressionMethod> compression_method,
    const std::shared_ptr<H5::H5File> h5_file)
    :   chunk_length_(chunk_length),
        chunk_cache_size_(chunk_cache_size),
        shuffle_enable_(shuffle_enable),
        compression_method_(compression_method),
        h5_file_(h5_file)
{}

void RosMessageToH5Writer::addGroup(const std::string& path_group)
{
    if (h5_file_->nameExists(path_group))
    {
        open_groups_.insert({path_group, h5_file_->openGroup(path_group)});
    }
    else
    {
        open_groups_.insert({path_group, h5_file_->createGroup(path_group)});
    }
}

void RosMessageToH5Writer::writeDataToDataset(
    const std::string& path_dataset,
    const void* data,
    const H5::DataType datatype,
    const size_t index_message,
    const int n_messages)
{
    // Check that the dataset has been opened
    auto element_iter = open_datasets_.find(path_dataset);
    if (element_iter == open_datasets_.end())
    {
        std::cout << "Attempted to write data to dataset that has not yet been opened: " << path_dataset << "\n";
        assert(false);
    }
    H5::DataSpace dataspace = element_iter->second.first;
    H5::DataSet dataset = element_iter->second.second;

    // Determine the dimensions of the dataspace subset to write to
    int rank = dataspace.getSimpleExtentNdims();
    hsize_t count[rank];
    dataspace.getSimpleExtentDims(count);
    count[0] = n_messages;

    // Select the dataspace subset to write to
    hsize_t offset[rank] = {};
    offset[0] = index_message;
    dataspace.selectHyperslab(H5S_SELECT_SET, count, offset);

    // Write the data
    H5::DataSpace memory_dataspace(rank, count);
    dataset.write(data, datatype, memory_dataspace, dataspace);
}

size_t RosMessageToH5Writer::getPrimeGreaterThanOrEqual(size_t start)
{
    if (start <= 2) return 2;

    // Check only odd numbers
    size_t prime = (start % 2 == 0) ? ++start : start;

    // Loop until prime number is found
    for (bool is_prime = false; !is_prime; prime += 2)
    {
        is_prime = true;
        for (size_t i = 3; i <= prime / 2; ++i)
        {
            if (prime % i == 0)
            {
                is_prime = false;
                break;
            }
        }
    }

    return prime - 2;
}
