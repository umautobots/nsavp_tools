#ifndef ROSMESSAGETOH5WRITER_H
#define ROSMESSAGETOH5WRITER_H

#include <string>
#include <map>
#include <vector>
#include <memory>
#include <cassert>
#include <iostream>

#include "h5_io/H5UtilityFunctions.h"
#include "h5_io/ros_to_h5/compression/CompressionMethod.h"

#include "H5Cpp.h"

namespace rosbag
{
    class View;
}

/**
 * Base class for writing ROS messages to a H5 file.
 */
class RosMessageToH5Writer
{
public:

    /**
     * Creates a ROS message to H5 writer with the specified chunking and compression settings.
     *
     * @param[in] chunk_length The size of the chunks in terms of number of messages. If zero no chunks will be used
     *                         (contiguous storage).
     * @param[in] chunk_cache_size The size of the chunk cache in terms of the number of chunks. If zero the chunk cache
     *                             will be disabled. The chunk cache can be disabled if data is always written as full
     *                             chunks. Otherwise, larger is better assuming it fits in memory.
     * @param[in] shuffle_enable Whether to enable the shuffle filter before compression. Ignored if chunks are
     *                           disabled.
     * @param[in] compression_method The compression method to apply to each chunk. Ignored if chunks are disabled.
     * @param[out] h5_file The H5 file to write to.
     *
     * @pre The h5_file must be open and writable
     */
    RosMessageToH5Writer(
        const int chunk_length,
        const int chunk_cache_size,
        const bool shuffle_enable,
        const std::shared_ptr<const CompressionMethod> compression_method,
        const std::shared_ptr<H5::H5File> h5_file);

    /************************ public member functions ************************/

    /**
     * Pure virtual function implemented by message-specific derived classes to write single-topic rosbag views to the
     * H5 file.
     *
     * @param[in] path_group The group to write datasets within and/or attributes to
     * @param[in] topic_view Topic specific rosbag view to be written to the H5 file.
     */
    virtual void writeRosbagTopicView(
        const std::string path_group,
        rosbag::View& view_topic) = 0;

protected:
    /************************ protected member functions ************************/

    /**
     * Creates a group with the specified absolute path if it does not already exist.
     *
     * The group object is stored to avoid repeatedly opening it.
     *
     * @param[in] path_group
     */
    void addGroup(const std::string& path_group);

    /**
     * Creates a dataset with the specified absolute path.
     *
     * The dataspace dimensions are set to (n_messages_total, dimensions[0], dimensions[1], ...) and the chunk size is
     * set to (chunk_length_, dimensions[0], dimensions[1], ...), unless chunking is disabled with
     * chunk_disable_override.
     *
     * The dataset's H5 datatype is inferred from the template type T, which should match that of the data to be
     * written.
     *
     * The dataspace and dataset objects are stored to avoid repeatedly opening them.
     *
     * @param[in] path_dataset The dataset absolute path.
     * @param[in] n_messages_total The total number of messages.
     * @param[in] dimensions The dimensions of the data contained in a single message. An empty vector implies the data
     *                       in a single message is a scalar.
     * @param[in] chunk_disable_override If set to true this dataset will not be chunked or compressed, overriding
     *                                   the settings passed to the constructor. This is to handle instances where one
     *                                   message field contains much less data than another and compressing the smaller
     *                                   field would be inefficient. An example is image timestamps vs image data.
     *
     * @pre The dataset's group must have already been opened and the dataset must not exist yet.
     */
    template<typename T>
    void addDataset(
        const std::string& path_dataset,
        const size_t n_messages,
        const std::vector<hsize_t>& dimensions = {},
        const bool chunk_disable_override = false)
    {
        // Check that the dataset's group has been opened
        std::string path_group = path_dataset.substr(0, path_dataset.find_last_of("/"));
        auto element_iter = open_groups_.find(path_group);
        if (element_iter == open_groups_.end())
        {
            std::cout << "Attempted to create dataset in group that has not yet been opened: " << path_group << "\n";
            assert(false);
        }

        // Create the dataspace
        std::vector<hsize_t> full_dimensions{n_messages};
        full_dimensions.insert(full_dimensions.end(), dimensions.begin(), dimensions.end());
        int rank = full_dimensions.size();
        H5::DataSpace dataspace(rank, full_dimensions.data());

        // Create the property lists
        H5::DSetCreatPropList dataset_create_property_list;
        H5::DSetAccPropList dataset_access_property_list;

        // Enable chunking if the chunk_length is great than zero, note that chunking is required for compression
        if (chunk_length_ > 0 && !chunk_disable_override)
        {
            // Set the chunk dimensions
            std::vector<hsize_t> chunk_dimensions(full_dimensions);
            chunk_dimensions[0] = chunk_length_;
            dataset_create_property_list.setChunk(rank, chunk_dimensions.data());

            // Set the chunk cache
            if (chunk_cache_size_ > 0)
            {
                // Determine the number of bytes per chunk
                size_t chunk_size_bytes = sizeof(T);
                for (const hsize_t chunk_dimension : chunk_dimensions)
                {
                    chunk_size_bytes *= chunk_dimension;
                }

                // Set the chunk cache size in bytes as a multiple of the size of a single chunk
                size_t n_chunk_cache_bytes = chunk_cache_size_ * chunk_size_bytes;

                // Paraphrasing H5 docs: this should be a prime number and approximately 100 times greater than
                // chunk_cache_size (the number of chunks that can fit in the cache)
                size_t n_chunk_cache_slots = getPrimeGreaterThanOrEqual(100 * chunk_cache_size_);

                // From the H5 docs: If your application only reads or writes data once, this can be safely set to 1
                double chunk_preemption_policy = 1.0;

                dataset_access_property_list.setChunkCache(n_chunk_cache_slots, n_chunk_cache_bytes,
                    chunk_preemption_policy);
            }
            else
            {
                // Disable the chunk cache
                dataset_access_property_list.setChunkCache(0, 0, 1.0);
            }

            // Add a shuffle filter before the compression filter if enabled
            if (shuffle_enable_)
            {
                dataset_create_property_list.setShuffle();
            }

            // Add the compression filter
            compression_method_->addCompressionToPropertyList(dataset_create_property_list);
        }

        // Get the H5 datatype
        H5::DataType datatype = getH5DataType<T>();

        // Create the dataset
        H5::DataSet dataset(h5_file_->createDataSet(path_dataset, datatype, dataspace, dataset_create_property_list,
            dataset_access_property_list));
        open_datasets_.insert({path_dataset, std::make_pair(dataspace, dataset)});
    }

    /**
     * Writes data from a vector into the specified dataset.
     *
     * The subset of the dataset written to has offset (index_message, 0, 0) and count
     * (n_messages, dimensions[0], dimensions[1], ...) where dimensions is the vector passed to the addDataset call
     * which created this dataset.
     *
     * Note that the data vector is assumed to be in row-major order.
     *
     * For efficiency, data should be written in chunks, i.e., n_messages should always equal chunk_length unless the
     * number of remaining messages is less than chunk_length.
     *
     * @param[in] path_dataset The dataset absolute path.
     * @param[in] data A vector containing the data to be written in row-major order.
     * @param[in] index_message The index of the next message to be written.
     * @param[in] n_messages The number of messages stored in the data vector.
     *
     * @pre The dataset to be written to must have already been opened.
     */
    template<typename T>
    void writeDataToDataset(
        const std::string& path_dataset,
        const std::vector<T>& data,
        const size_t index_message,
        const int n_messages)
    {
        writeDataToDataset(path_dataset, data.data(), getH5DataType<T>(), index_message, n_messages);
    }

    /**
     * Overload that writes a scalar value to a one dimensional dataset.
     *
     * @param[in] path_dataset The dataset absolute path.
     * @param[in] data A scalar value to be written
     * @param[in] index_message The index of the next message to be written.
     *
     * @pre The dataset to be written to must have already been opened.
     */
    template<typename T>
    void writeDataToDataset(
        const std::string& path_dataset,
        const T& data,
        const size_t index_message)
    {
        writeDataToDataset(path_dataset, &data, getH5DataType<T>(), index_message, 1);
    }

    /**
     * Overload that takes in a void* buffer and H5 datatype. Needed in cases where the H5 datatype should not be
     * inferred from the vector datatype. For example, ROS images are stored as uint8_t vectors regardless of the actual
     * image encoding.
     *
     * @param[in] path_dataset The dataset absolute path.
     * @param[in] data A buffer containing the data to be written in row-major order.
     * @param[in] datatype The H5 datatype of the data to be written.
     * @param[in] index_message The index of the next message to be written.
     * @param[in] n_messages The number of messages stored in the data vector.
     *
     * @pre The dataset to be written to must have already been opened.
     */
    void writeDataToDataset(
        const std::string& path_dataset,
        const void* data,
        const H5::DataType datatype,
        const size_t index_message,
        const int n_messages);

    /**
     * Writes a scalar (single element) attribute to the specified object (group or dataset).
     *
     * The attributes's H5 datatype is inferred from the template type T.
     *
     * @param[in] path_object The absolute path of the object (group or dataset) to write the attribute to.
     * @param[in] name The attribute name.
     * @param[in] value The attribute value.
     *
     * @pre The object must have already been opened and the attribute must not exist yet.
     */
    template<typename T>
    void writeAttributeToObject(
        const std::string& path_object,
        const std::string& name,
        const T& value)
    {
        // Get the attribute type
        H5::DataType datatype = getH5DataType<T>();

        // Create the attribute dataspace, note that all attributes are assumed to be scalars
        H5::DataSpace dataspace = H5::DataSpace(H5S_SCALAR);

        // Write the attribute to the specified object if it has been opened
        auto element_iter_group = open_groups_.find(path_object);
        if (element_iter_group != open_groups_.end())
        {
            H5::Group group = element_iter_group->second;
            H5::Attribute attribute = group.createAttribute(name, datatype, dataspace);
            attribute.write(datatype, &value);
            return;
        }

        auto element_iter_dataset = open_datasets_.find(path_object);
        if (element_iter_dataset != open_datasets_.end())
        {
            H5::DataSet dataset = element_iter_dataset->second.second;
            H5::Attribute attribute = dataset.createAttribute(name, datatype, dataspace);
            attribute.write(datatype, &value);
            return;
        }

        std::cout << "Attempted to write attribute to object that has not yet been opened: " << path_object << "\n";
        assert(false);
    }

    /************************ protected member variables ************************/

    // Chunk length along the image dimension
    const int chunk_length_;

private:
    /************************ private member variables ************************/

    /**
     * Gets the smallest prime number greater than or equal to start.
     *
     * @param[in] start The initial number to check.
     * @return The smallest prime number greater than or equal to start.
     */
    size_t getPrimeGreaterThanOrEqual(size_t start);

    const int chunk_cache_size_;

    // Compression parameters
    const bool shuffle_enable_;
    const std::shared_ptr<const CompressionMethod> compression_method_;

    // H5 file
    const std::shared_ptr<H5::H5File> h5_file_;

    // Opened H5 file objects
    std::map<std::string, H5::Group> open_groups_;
    std::map<std::string, std::pair<H5::DataSpace, H5::DataSet>> open_datasets_;
};

#endif //ROSMESSAGETOH5WRITER_H