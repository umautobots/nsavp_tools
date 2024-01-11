#ifndef H5TOCOMMONCONFIGURATIONWRITER_H
#define H5TOCOMMONCONFIGURATIONWRITER_H

#include "h5_io/h5_to_common/H5ToCommonWriter.h"

class H5ToCommonConfigurationWriter : public H5ToCommonWriter
{
public:
    H5ToCommonConfigurationWriter(H5::Group& group);

    /************************ public member functions ************************/

    void writeH5TopicGroup(const std::string path_output);
};

#endif //H5TOCOMMONCONFIGURATIONWRITER_H