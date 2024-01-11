#ifndef H5TOCOMMONCAMERADIAGNOSTICSWRITER_H
#define H5TOCOMMONCAMERADIAGNOSTICSWRITER_H

#include "h5_io/h5_to_common/H5ToCommonWriter.h"

class H5ToCommonCameraDiagnosticsWriter : public H5ToCommonWriter
{
public:
    H5ToCommonCameraDiagnosticsWriter(H5::Group& group);

    /************************ public member functions ************************/

    void writeH5TopicGroup(const std::string path_output);
};

#endif //H5TOCOMMONCAMERADIAGNOSTICSWRITER_H