#ifndef H5TOCOMMONIMAGEMETAWRITER_H
#define H5TOCOMMONIMAGEMETAWRITER_H

#include "h5_io/h5_to_common/H5ToCommonWriter.h"

class H5ToCommonImageMetaWriter : public H5ToCommonWriter
{
public:
    H5ToCommonImageMetaWriter(H5::Group& group);

    /************************ public member functions ************************/

    void writeH5TopicGroup(const std::string path_output);
};

#endif //H5TOCOMMONIMAGEMETAWRITER_H