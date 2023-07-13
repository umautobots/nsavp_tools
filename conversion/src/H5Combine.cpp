#include <filesystem>
#include <vector>
#include <string>
#include <iostream>
#include <stdexcept>

#include <ros/ros.h>

#include "H5Cpp.h"

using fspath = std::filesystem::path;

int main(int argc, char* argv[])
{
    // Initialize ROS node
    ros::init(argc, argv, "h5_combine");
    ros::NodeHandle nh_private("~");

    // Process arguments
    std::string path_base_folder, filepath_h5_out;
    std::vector<std::string> filepaths_h5_in;
    bool remove_prefix, symbolic, relative;
    nh_private.param<std::string>("path_base_folder", path_base_folder, "");
    nh_private.param<std::string>("filepath_h5_out", filepath_h5_out, "");
    nh_private.param<std::vector<std::string>>("filepaths_h5_in", filepaths_h5_in, std::vector<std::string>{});
    nh_private.param<bool>("remove_prefix", remove_prefix, true);
    nh_private.param<bool>("symbolic", symbolic, true);
    nh_private.param<bool>("relative", relative, true);

    // Determine H5 files to combine
    std::vector<fspath> paths_h5_in;
    if (filepaths_h5_in.empty())
    {
        if (path_base_folder.empty())
        {
            std::cout << "No input folder or filepaths were specified.\n";
            throw std::invalid_argument("No input folder or filepaths were specified.");
        }
        for (std::filesystem::directory_entry directory_entry : std::filesystem::directory_iterator(path_base_folder))
        {
            if (directory_entry.path().extension() == ".h5")
            {
                paths_h5_in.push_back(directory_entry.path());
            }
        }
    }
    else
    {
        for (std::string filepath_h5_in : filepaths_h5_in)
        {
            paths_h5_in.push_back(fspath(path_base_folder) / fspath(filepath_h5_in));
        }
    }

    // Create the combined H5 file
    if (filepath_h5_out.empty())
    {
        std::cout << "No output filepath was specified.\n";
        throw std::invalid_argument("No output filepath was specified.");
    }
    fspath path_h5_out = fspath(path_base_folder) / fspath(filepath_h5_out);
    std::filesystem::create_directories(path_h5_out.parent_path());
    std::cout << "Creating file " << path_h5_out << "\n";
    H5::H5File h5_file_combined(path_h5_out, H5F_ACC_TRUNC);

    // Combine input H5 files into the output file
    for (fspath path_h5_in : paths_h5_in)
    {
        // Avoid creating self-referential links
        if (path_h5_in == path_h5_out)
        {
            continue;
        }

        std::string name_target = path_h5_in.stem();
        if (remove_prefix)
        {
            size_t index_prefix_end = name_target.find("sample_");
            index_prefix_end = (index_prefix_end != std::string::npos) ? index_prefix_end + 7 :
                ((name_target.at(0) == 'C') ? 3 : 7);
            name_target = name_target.substr(index_prefix_end);
        }

        std::cout << "Processing filepath " << path_h5_in << "\n";
        if (!std::filesystem::exists(path_h5_in))
        {
            std::cout << "Skipping, no file exists with this filepath\n";
            continue;
        }
        if (symbolic)
        {
            std::string path_h5_target = relative ? std::filesystem::relative(path_h5_in, path_h5_out.parent_path()) :
                path_h5_in;

            std::cout << "Creating a symbolic link " << name_target << " with the path " << path_h5_target << "\n";

            H5Lcreate_external(path_h5_target.c_str(), "/", h5_file_combined.getLocId(),
                name_target.c_str(), H5Pcreate(H5P_LINK_CREATE), H5P_DEFAULT);
        }
        else
        {
            std::cout << "Copying data to group " << name_target << " from " << path_h5_in << "\n";

            H5::H5File h5_file_to_copy(path_h5_in, H5F_ACC_RDONLY);
            H5Ocopy(h5_file_to_copy.getLocId(), "/", h5_file_combined.getLocId(),
                name_target.c_str(), H5P_DEFAULT, H5P_DEFAULT);
        }
    }

    return 0;
}