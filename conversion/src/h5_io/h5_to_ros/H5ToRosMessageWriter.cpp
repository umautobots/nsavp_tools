#include "h5_io/h5_to_ros/H5ToRosMessageWriter.h"

H5ToRosMessageWriter::H5ToRosMessageWriter(H5::Group& group)
    :   group_(group),
        chunk_length_(1),
        chunked_(false),
        n_messages_(-1) // Note: hsize_t is uint64_t, this is setting to the maximum value
{}

void H5ToRosMessageWriter::openDataset(const std::string name)
{
    if (!group_.nameExists(name))
    {
        std::cout << "Tried to open dataset " << name << " but it does not exist\n";
        assert(false);
    }

    // Open and store the dataset and dataspace, and initialize the message count on this dataset
    H5::DataSet dataset = group_.openDataSet(name);
    H5::DataSpace dataspace = dataset.getSpace();
    open_datasets_.insert({name, std::make_pair(dataspace, dataset)});
    indices_message_.insert({name, 0});

    // Determine the number of messages and ensure it matches across all opened datasets
    int rank = dataspace.getSimpleExtentNdims();
    hsize_t dimensions_dataspace[rank];
    dataspace.getSimpleExtentDims(dimensions_dataspace);
    hsize_t n_messages = dimensions_dataspace[0];
    if (n_messages_ != -1 && n_messages_ != n_messages)
    {
        std::cout << "Dataset " << name << " has a different number of messages than a previously opened dataset\n";
        assert(false);
    }
    n_messages_ = n_messages;

    // Get the chunk length, if chunking is present
    H5::DSetCreatPropList dataset_create_property_list = dataset.getCreatePlist();
    hsize_t dimensions_chunk[rank];
    if (H5D_CHUNKED == dataset_create_property_list.getLayout())
    {
        dataset_create_property_list.getChunk(rank, dimensions_chunk);
        int chunk_length = dimensions_chunk[0];
        if (chunked_ == true && chunk_length_ != chunk_length)
        {
            std::cout << "Dataset " << name << " has a different chunk length than a previously opened dataset\n";
            assert(false);
        }
        chunk_length_ = chunk_length;
        chunked_ = true;

        // Disable the chunk cache, this class only reads in data chunk by chunk so it is not needed
        H5::DSetAccPropList dataset_access_property_list = dataset.getAccessPlist();
        dataset_access_property_list.setChunkCache(0, 0, 1.0);
    }
}
