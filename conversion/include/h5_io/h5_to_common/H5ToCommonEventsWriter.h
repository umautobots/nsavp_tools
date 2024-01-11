#ifndef H5TOCOMMONEVENTSWRITER_H
#define H5TOCOMMONEVENTSWRITER_H

#include "h5_io/h5_to_common/H5ToCommonWriter.h"

class H5ToCommonEventsWriter : public H5ToCommonWriter
{
public:
    H5ToCommonEventsWriter(H5::Group& group);

    /************************ public member functions ************************/

    void writeH5TopicGroup(const std::string path_output);
};

#endif //H5TOCOMMONEVENTSWRITER_H