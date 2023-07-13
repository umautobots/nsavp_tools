#ifndef ADD_VAL_HH
#define ADD_VAL_HH

#include <image_meta_msgs/NameValueFloat.h>
#include <image_meta_msgs/NameValueInt.h>
#include <image_meta_msgs/NameValueString.h>
#include <image_meta_msgs/ImageMeta.h>
#include <image_meta_msgs/CameraDiagnostics.h>

namespace image_meta_msgs
{
    static inline void addVal(ImageMeta& image_meta_msg,
                            const std::string& name,
                            int64_t value)
    {
        NameValueInt nv_int;
        nv_int.name = name;
        nv_int.value = value;
        image_meta_msg.name_value_ints.push_back(nv_int);
    }

    static inline void addVal(ImageMeta& image_meta_msg,
                            const std::string& name,
                            double value)
    {
        NameValueFloat nv_float;
        nv_float.name = name;
        nv_float.value = value;
        image_meta_msg.name_value_floats.push_back(nv_float);
    }

    static inline void addVal(ImageMeta& image_meta_msg,
                            const std::string& name,
                            const std::string& value)
    {
        NameValueString nv_string;
        nv_string.name = name;
        nv_string.value = value;
        image_meta_msg.name_value_strings.push_back(nv_string);
    }

    static inline void addVal(CameraDiagnostics& cam_diag_msg,
                            const std::string& name,
                            int64_t value)
    {
        NameValueInt nv_int;
        nv_int.name = name;
        nv_int.value = value;
        cam_diag_msg.name_value_ints.push_back(nv_int);
    }

    static inline void addVal(CameraDiagnostics& cam_diag_msg,
                            const std::string& name,
                            bool value)
    {
        NameValueInt nv_int;
        nv_int.name = name;
        nv_int.value = value ? 1 : 0;
        cam_diag_msg.name_value_bools.push_back(nv_int);
    }

    static inline void addVal(CameraDiagnostics& cam_diag_msg,
                            const std::string& name,
                            double value)
    {
        NameValueFloat nv_float;
        nv_float.name = name;
        nv_float.value = value;
        cam_diag_msg.name_value_floats.push_back(nv_float);
    }

    static inline void addVal(CameraDiagnostics& cam_diag_msg,
                            const std::string& name,
                            const std::string& value,
                            bool is_enum = false)
    {
        NameValueString nv_string;
        nv_string.name = name;
        nv_string.value = value;
        if(is_enum)
        {
            cam_diag_msg.name_value_enums.push_back(nv_string);
        }
        else
        {
            cam_diag_msg.name_value_strings.push_back(nv_string);
        }
    }
}

#endif