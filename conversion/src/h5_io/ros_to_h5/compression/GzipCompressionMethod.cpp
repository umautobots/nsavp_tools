#include "h5_io/ros_to_h5/compression/GzipCompressionMethod.h"

#include "H5Cpp.h"

GzipCompressionMethod::GzipCompressionMethod(const int compression_level)
    :   compression_level_(compression_level)
{}

void GzipCompressionMethod::addCompressionToPropertyList(H5::DSetCreatPropList& property_list) const
{
    property_list.setDeflate(compression_level_);
}
