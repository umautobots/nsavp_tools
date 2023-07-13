#ifndef COMPRESSIONMETHOD_H
#define COMPRESSIONMETHOD_H

namespace H5
{
    class DSetCreatPropList;
}

class CompressionMethod
{
public:
    /************************ public member functions ************************/

    virtual void addCompressionToPropertyList(H5::DSetCreatPropList& property_list) const = 0;
};

#endif //COMPRESSIONMETHOD_H