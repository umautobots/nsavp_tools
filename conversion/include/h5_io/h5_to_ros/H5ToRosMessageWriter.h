#ifndef H5TOROSMESSAGEWRITER_H
#define H5TOROSMESSAGEWRITER_H

#include <string>
#include <map>
#include <vector>
#include <cassert>
#include <iostream>

#include <rosbag/bag.h>

#include "h5_io/H5UtilityFunctions.h"

/**
 * Base class for writing ROS messages from an H5 group.
 */
class H5ToRosMessageWriter
{
public:

    /**
     * Creates a H5 to ROS message writer.
     *
     * @param[out] group A single-topic H5 group.
     *
     * @pre The group must be open
     */
    H5ToRosMessageWriter(H5::Group& group);

    /************************ public member functions ************************/

    /**
     * Pure virtual function implemented by message-specific derived classes to write single-topic h5 groups to the
     * rosbag.
     *
     * @param[in] topic The topic to write messages on.
     * @param[in] bag The rosbag to write messages to.
     *
     * @pre The bag must be open and writable.
     */
    virtual void writeH5TopicGroup(
        const std::string topic,
        rosbag::Bag& bag) = 0;

protected:
    /************************ protected member functions ************************/

    /**
     * Opens a dataset with the specified name in the group.
     *
     * The dataspace and dataset objects are stored along with chunking information, if chunking is present.
     *
     * @param[in] name The name of the dataset.
     */
    void openDataset(const std::string name);

    /**
     * Reads the next chunk from the specified dataset to a vector.
     *
     * This class keeps an internal count of the next chunk to read. If the remaining number of messages do not fill out
     * a full chunk, the remaining number of messages is read.
     *
     * @param[in] name The name of the dataset.
     * @param[out] data A vector for the data to be read into in row-major order.
     * @param[out] n_messages_read The number of messages read. Equal to the chunk length unless the number of remaining
     *                             messages is less than the chunk length.
     */
    template<typename T>
    void readNextChunkFromDataset(
        const std::string name,
        std::vector<T>& data,
        size_t& n_messages_read)
    {
        // Check that the dataset has been opened
        auto element_iter = open_datasets_.find(name);
        if (element_iter == open_datasets_.end())
        {
            openDataset(name);
        }
        H5::DataSpace dataspace = open_datasets_.at(name).first;
        H5::DataSet dataset = open_datasets_.at(name).second;
        hsize_t& index_message = indices_message_.at(name);

        // Determine the dimensions of the dataspace subset to read from
        int rank = dataspace.getSimpleExtentNdims();
        hsize_t count[rank];
        dataspace.getSimpleExtentDims(count);
        hsize_t n_remaining_messages = n_messages_ - index_message;
        count[0] = (n_remaining_messages < chunk_length_) ? n_remaining_messages : chunk_length_;
        n_messages_read = count[0];

        // Select the dataspace subset to read from
        hsize_t offset[rank] = {};
        offset[0] = index_message;
        dataspace.selectHyperslab(H5S_SELECT_SET, count, offset);

        // Resize the vector to fit the data to be read
        hsize_t n_elements_to_read = 1;
        for (int i = 0; i < rank; i++)
        {
            n_elements_to_read *= count[i];
        }
        data.resize(n_elements_to_read);

        // Read in the chunk
        H5::DataSpace memory_dataspace_chunk(rank, count);
        dataset.read(data.data(), getH5DataType<T>(), memory_dataspace_chunk, dataspace);

        // Increment the message index by the chunk length
        index_message += chunk_length_;
    }

    /**
     * Reads the next value from the specified one-dimensional dataset.
     *
     * The next value to read is tracked internally.
     *
     * @param[in] name The name of the dataset.
     * @param[out] data A reference to the variable the value will be read into.
     *
     * @pre The dataset to be read from must be one-dimensional
     */
    template<typename T>
    void readNextValueFrom1DDataset(
        const std::string name,
        T& data)
    {
        // Check that the dataset has been opened
        auto element_iter = open_datasets_.find(name);
        if (element_iter == open_datasets_.end())
        {
            openDataset(name);
        }
        H5::DataSpace dataspace = open_datasets_.at(name).first;
        H5::DataSet dataset = open_datasets_.at(name).second;
        hsize_t& index_message = indices_message_.at(name);

        // Check that the dataset is one-dimensional
        int rank = dataspace.getSimpleExtentNdims();
        if (rank != 1)
        {
            std::cout << "The dataset " << name << " is not one-dimensional\n";
            assert(false);
        }

        // Read in the value
        hsize_t count = 1;
        dataspace.selectHyperslab(H5S_SELECT_SET, &count, &index_message);
        H5::DataSpace memory_dataspace(rank, &count);
        if constexpr (std::is_same_v<T, std::string>)
        {
            dataset.read(data, getH5DataType<T>(), memory_dataspace, dataspace);
        }
        else
        {
            dataset.read(&data, getH5DataType<T>(), memory_dataspace, dataspace);
        }

        // Increment the message index
        index_message++;
    }

    /**
     * Reads a scalar (single element) attribute from the group.
     *
     * The attributes's H5 datatype is inferred from the template type T.
     *
     * @param[in] name The attribute name.
     * @param[out] value The attribute value to set.
     */
    template<typename T>
    void readAttributeFromGroup(
        const std::string& name,
        T& value)
    {
        if (!group_.attrExists(name))
        {
            std::cout << "Tried to read attribute " << name << " but it does not exist\n";
            assert(false);
        }

        // Get the attribute type
        H5::DataType datatype = getH5DataType<T>();

        // Open the attribute
        H5::Attribute attribute = group_.openAttribute(name);

        // Read the attribute value
        if constexpr (std::is_same_v<T, std::string>)
        {
            attribute.read(datatype, value);
        }
        else
        {
            attribute.read(datatype, &value);
        }
    }

    /************************ protected member variables ************************/

    // Total number of messages
    hsize_t n_messages_;

    // H5 group
    H5::Group group_;

private:
    /************************ private member variables ************************/

    // Opened H5 file objects
    std::map<std::string, std::pair<H5::DataSpace, H5::DataSet>> open_datasets_;

    // Chunk information
    int chunk_length_;
    bool chunked_;

    // Message counts
    std::map<std::string, hsize_t> indices_message_;
};

#endif //ROSMESSAGETOH5WRITER_H