#ifndef H5TOCOMMONPOSESTAMPEDWRITER_H
#define H5TOCOMMONPOSESTAMPEDWRITER_H

#include "h5_io/h5_to_common/H5ToCommonWriter.h"

class H5ToCommonPoseStampedWriter : public H5ToCommonWriter
{
public:
    H5ToCommonPoseStampedWriter(H5::Group& group);

    /************************ public member functions ************************/

    void writeH5TopicGroup(const std::string path_output);
};

#endif //H5TOCOMMONPOSESTAMPEDWRITER_H