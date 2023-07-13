#ifndef GZIPCOMPRESSIONMETHOD_H
#define GZIPCOMPRESSIONMETHOD_H

#include "h5_io/ros_to_h5/compression/CompressionMethod.h"

class GzipCompressionMethod : public CompressionMethod
{
public:
    GzipCompressionMethod(const int compression_level);

    /************************ public member functions ************************/

    void addCompressionToPropertyList(H5::DSetCreatPropList& property_list) const;

private:
    /************************ private member variables ************************/

    int compression_level_;
};

#endif //GZIPCOMPRESSIONMETHOD_H