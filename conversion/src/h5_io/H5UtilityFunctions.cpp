#include "h5_io/H5UtilityFunctions.h"

herr_t H5LIterateCallbackFunction(
    hid_t id_object,
    const char* name_object,
    const H5L_info_t* info,
    void* operator_data)
{
    std::vector<std::string>* names_out = static_cast<std::vector<std::string>*>(operator_data);
    names_out->push_back(std::string(name_object));
    return 0;
}

herr_t H5AIterateCallbackFunction(
    hid_t id_object,
    const char* name_attribute,
    const H5A_info_t* info,
    void* operator_data)
{
    std::vector<std::string>* names_out = static_cast<std::vector<std::string>*>(operator_data);
    names_out->push_back(std::string(name_attribute));
    return 0;
}