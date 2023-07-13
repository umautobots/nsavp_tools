#ifndef H5UTILITYFUNCTIONS_H
#define H5UTILITYFUNCTIONS_H

#include <vector>
#include <string>
#include <cassert>
#include <iostream>
#include <type_traits>

#include "H5Cpp.h"

/**
 * Returns the H5 datatype that corresponds to the C++ template datatype.
 *
 * @return The H5 datatype corresponding to the template type T.
 */
template <typename T>
H5::DataType getH5DataType()
{
    if (std::is_same<T, std::int8_t>::value)
    {
        return H5::PredType::NATIVE_INT8;
    }
    else if (std::is_same<T, std::uint8_t>::value)
    {
        return H5::PredType::NATIVE_UINT8;
    }
    else if (std::is_same<T, std::int16_t>::value)
    {
        return H5::PredType::NATIVE_INT16;
    }
    else if (std::is_same<T, std::uint16_t>::value)
    {
        return H5::PredType::NATIVE_UINT16;
    }
    else if (std::is_same<T, std::int32_t>::value)
    {
        return H5::PredType::NATIVE_INT32;
    }
    else if (std::is_same<T, std::uint32_t>::value)
    {
        return H5::PredType::NATIVE_UINT32;
    }
    else if (std::is_same<T, std::int64_t>::value)
    {
        return H5::PredType::NATIVE_INT64;
    }
    else if (std::is_same<T, std::uint64_t>::value)
    {
        return H5::PredType::NATIVE_UINT64;
    }
    else if (std::is_same<T, float>::value)
    {
        return H5::PredType::NATIVE_FLOAT;
    }
    else if (std::is_same<T, double>::value)
    {
        return H5::PredType::NATIVE_DOUBLE;
    }
    else if (std::is_same<T, std::string>::value)
    {
        return H5::StrType(H5::PredType::C_S1, H5T_VARIABLE);
    }
    else
    {
        std::cout << "Requested datatype currently unsupported.\n";
        assert(false);
    }
}

/**
 * Callback function called by H5LIterate for each link it iterates over within a group.
 *
 * Stores the names of all links/objects iterated over within the group.
 *
 * @param[in] id_object The location identifier of the group iterated in (not used here).
 * @param[in] name_object The name of the link/object at the current iteration, i.e. path of the current link/object
 *                        relative to the group iterated in.
 * @param[in] info A struct containing information about the current link (not used here).
 * @param[out] operator_data A buffer passed into H5LIterate where output data can be cumulatively stored at each
 *                           iteration. Here, we store the names of all objects iterated over.
 *
 * @return A value indicating one of three things: (1) 0 indicates that iteration should continue, (2) positive
 *         indicates that a condition has been satisfied and iteration should stop and (3) negative indicates that an
 *         error has occurred and iteration should stop. Here, we always return 0 in order to iterate over all links
 *         within the group.
 */
herr_t H5LIterateCallbackFunction(
    hid_t id_object,
    const char* name_object,
    const H5L_info_t* info,
    void* operator_data);

/**
 * Callback function called by H5Aiterate for each attribute it iterates over within an object.
 *
 * Stores the names of all attributes iterated over within the object.
 *
 * @param[in] id_object The location identifier of the object iterated in (not used here).
 * @param[in] name_attribute The name of the attribute at the current iteration
 * @param[in] info A struct containing information about the current attribute (not used here).
 * @param[out] operator_data A buffer passed into H5Aiterate where output data can be cumulatively stored at each
 *                           iteration. Here, we store the names of all attributes iterated over.
 *
 * @return A value indicating one of three things: (1) 0 indicates that iteration should continue, (2) positive
 *         indicates that a condition has been satisfied and iteration should stop and (3) negative indicates that an
 *         error has occurred and iteration should stop. Here, we always return 0 in order to iterate over all
 *         attributes within the object.
 */
herr_t H5AIterateCallbackFunction(
    hid_t id_object,
    const char* name_attribute,
    const H5A_info_t* info,
    void* operator_data);

#endif //H5UTILITYFUNCTIONS_H