// This script is used to validate the rosbag->h5->rosbag conversion by comparing the input and output rosbags

#include <iostream>
#include <string>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>

#include <image_meta_msgs/CameraDiagnostics.h>
#include <image_meta_msgs/ImageMeta.h>
#include <image_meta_msgs/NameValueFloat.h>
#include <image_meta_msgs/NameValueInt.h>
#include <image_meta_msgs/NameValueString.h>

#include <sensor_msgs/Image.h>

#include <geometry_msgs/PoseStamped.h>

std::string removeForwardSlashesFromNamespace(const std::string& topic)
{
    // Initialize string as the namespace
    std::string namespace_without_forward_slashes = topic.substr(1, topic.find_last_of("/") - 1);

    // Replace all forward slashes in the namespace with underscores
    size_t index_forward_slash = namespace_without_forward_slashes.find("/");
    while (index_forward_slash != std::string::npos)
    {
        namespace_without_forward_slashes[index_forward_slash] = '_';
        index_forward_slash = namespace_without_forward_slashes.find("/");
    }

    return namespace_without_forward_slashes + topic.substr(topic.find_last_of("/"));
}

int main(int argc, char* argv[])
{
    // Initialize ROS node
    ros::init(argc, argv, "rosbag_validator");
    ros::NodeHandle nh_private("~");

    // Process arguments
    std::string path_bag_1, path_bag_2;
    nh_private.param<std::string>("path_bag_1", path_bag_1, "");
    nh_private.param<std::string>("path_bag_2", path_bag_2, "");

    // Open rosbag 1
    std::cout << "Opening rosbag 1: " << path_bag_1.c_str() << "\n";
    rosbag::Bag bag_1;
    bag_1.open(path_bag_1.c_str(), rosbag::bagmode::Read);

    // Open rosbag 2
    std::cout << "Opening rosbag 2: " << path_bag_2.c_str() << "\n";
    rosbag::Bag bag_2;
    bag_2.open(path_bag_2.c_str(), rosbag::bagmode::Read);

    // Loop over topics
    rosbag::View view_full_1(bag_1);
    rosbag::View view_full_2(bag_2);

    std::vector<const rosbag::ConnectionInfo*> connections_1 = view_full_1.getConnections();
    std::cout << "\n----------------------------------------\n";
    for (const rosbag::ConnectionInfo* connection_1 : connections_1)
    {
        std::string topic_1 = connection_1->topic;
        std::string message_type_1 = connection_1->datatype;
        std::string topic_1_standardized = removeForwardSlashesFromNamespace(topic_1);
        std::cout << "Found topic " << topic_1 << " in rosbag 1 with message type " << message_type_1 << "\n";

        std::vector<const rosbag::ConnectionInfo*> connections_2 = view_full_2.getConnections();
        bool match_found = false;
        std::string topic_2;
        std::string message_type_2;
        for (const rosbag::ConnectionInfo* connection_2 : connections_2)
        {
            topic_2 = connection_2->topic;
            message_type_2 = connection_2->datatype;
            std::string topic_2_standardized = removeForwardSlashesFromNamespace(topic_2);
            if (topic_2_standardized == topic_1_standardized)
            {
                std::cout << "Found matching topic " << topic_2 << " in rosbag 2\n";
                match_found = true;
                break;
            }
        }

        if (match_found == false)
        {
            std::cout << "Found no matching topic in rosbag 2\n";
            continue;
        }

        if (message_type_2 != message_type_1)
        {
            std::cout << "Message type mismatch\n";
            continue;
        }

        rosbag::View view_topic_1(bag_1, rosbag::TopicQuery(topic_1));
        rosbag::View view_topic_2(bag_2, rosbag::TopicQuery(topic_2));
        size_t n_messages_1 = view_topic_1.size();
        size_t n_messages_2 = view_topic_2.size();
        auto iterator_1 = view_topic_1.begin();
        auto iterator_2 = view_topic_2.begin();
        bool no_differences = true;
        std::cout << "Comparing messages...\n";
        if (message_type_1 == "dvs_msgs/EventArray")
        {
            size_t message_count_1 = 0;
            dvs_msgs::EventArray::ConstPtr message_event_array_1 = iterator_1->instantiate<dvs_msgs::EventArray>();
            dvs_msgs::EventArray::ConstPtr message_event_array_2 = iterator_2->instantiate<dvs_msgs::EventArray>();
            auto iterator_event_array_1 = message_event_array_1->events.begin();
            auto iterator_event_array_2 = message_event_array_2->events.begin();
            while(true)
            {
                if (!(*iterator_event_array_1 == *iterator_event_array_2))
                {
                    std::cout << std::endl;
                    std::cout << "Event message mismatch\n";
                    no_differences = false;
                    break;
                }

                iterator_event_array_1++;
                iterator_event_array_2++;

                if (iterator_event_array_1 == message_event_array_1->events.end())
                {
                    iterator_1++;

                    message_count_1++;
                    std::cout << "\33[2K\rPercent completion: " << 100.0 * message_count_1 / n_messages_1 << std::flush;

                    if (iterator_1 == view_topic_1.end())
                    {
                        iterator_2++;
                        break;
                    }
                    else
                    {
                        message_event_array_1 = iterator_1->instantiate<dvs_msgs::EventArray>();
                        iterator_event_array_1 = message_event_array_1->events.begin();
                    }
                }

                if (iterator_event_array_2 == message_event_array_2->events.end())
                {
                    iterator_2++;
                    if (iterator_2 == view_topic_2.end())
                    {
                        iterator_1++;
                        break;
                    }
                    else
                    {
                        message_event_array_2 = iterator_2->instantiate<dvs_msgs::EventArray>();
                        iterator_event_array_2 = message_event_array_2->events.begin();
                    }
                }
            }
            std::cout << std::endl;

            if (!(iterator_event_array_1 == message_event_array_1->events.end() && iterator_1 == view_topic_1.end() &&
                iterator_event_array_2 == message_event_array_2->events.end() && iterator_2 == view_topic_2.end()) &&
                no_differences)
            {
                std::cout << "Number of event messages mismatch\n";
                no_differences = false;
            }
        }
        else
        {
            if (n_messages_1 != n_messages_2)
            {
                std::cout << "Number of messages mismatch\n";
                no_differences = false;
            }
            else
            {
                size_t message_count_1 = 0;
                while(iterator_1 != view_topic_1.end())
                {
                    if (message_type_1 == "image_meta_msgs/CameraDiagnostics")
                    {
                        image_meta_msgs::CameraDiagnostics::Ptr message_diag_1 =
                            iterator_1->instantiate<image_meta_msgs::CameraDiagnostics>();
                        image_meta_msgs::CameraDiagnostics::Ptr message_diag_2 =
                            iterator_2->instantiate<image_meta_msgs::CameraDiagnostics>();

                        // Sequence and frame IDs not set in H5 to rosbag conversion, so we don't compare them here
                        message_diag_1->header.seq = message_diag_2->header.seq;
                        message_diag_1->header.frame_id = message_diag_2->header.frame_id;

                        no_differences = no_differences && (message_diag_1->header == message_diag_2->header);
                        no_differences = no_differences &&
                            (message_diag_1->request_stamp == message_diag_2->request_stamp);
                        no_differences = no_differences && (message_diag_1->hardware_id == message_diag_2->hardware_id);

                        // Note: the order of name-value pairs in not maintained through conversion
                        bool match_found = false;
                        for (const image_meta_msgs::NameValueFloat& message_name_value_float_1 :
                            message_diag_1->name_value_floats)
                        {
                            match_found = false;
                            for (const image_meta_msgs::NameValueFloat& message_name_value_float_2 :
                                message_diag_2->name_value_floats)
                            {
                                if (message_name_value_float_2.name == message_name_value_float_1.name &&
                                    message_name_value_float_2.value == message_name_value_float_1.value)
                                {
                                    match_found = true;
                                    break;
                                }
                            }
                            no_differences = no_differences && match_found;
                        }

                        for (const image_meta_msgs::NameValueInt& message_name_value_int_1 :
                            message_diag_1->name_value_ints)
                        {
                            match_found = false;
                            for (const image_meta_msgs::NameValueInt& message_name_value_int_2 :
                                message_diag_2->name_value_ints)
                            {
                                if (message_name_value_int_2.name == message_name_value_int_1.name &&
                                    message_name_value_int_2.value == message_name_value_int_1.value)
                                {
                                    match_found = true;
                                    break;
                                }
                            }
                            no_differences = no_differences && match_found;
                        }

                        for (const image_meta_msgs::NameValueInt& message_name_value_bool_1 :
                            message_diag_1->name_value_bools)
                        {
                            match_found = false;
                            for (const image_meta_msgs::NameValueInt& message_name_value_bool_2 :
                                message_diag_2->name_value_bools)
                            {
                                if (message_name_value_bool_2.name == message_name_value_bool_1.name &&
                                    message_name_value_bool_2.value == message_name_value_bool_1.value)
                                {
                                    match_found = true;
                                    break;
                                }
                            }
                            no_differences = no_differences && match_found;
                        }

                        for (const image_meta_msgs::NameValueString& message_name_value_string_1 :
                            message_diag_1->name_value_strings)
                        {
                            match_found = false;
                            for (const image_meta_msgs::NameValueString& message_name_value_string_2 :
                                message_diag_2->name_value_strings)
                            {
                                if (message_name_value_string_2.name == message_name_value_string_1.name &&
                                    message_name_value_string_2.value == message_name_value_string_1.value)
                                {
                                    match_found = true;
                                    break;
                                }
                            }
                            no_differences = no_differences && match_found;
                        }

                        for (const image_meta_msgs::NameValueString& message_name_value_enum_1 :
                            message_diag_1->name_value_enums)
                        {
                            match_found = false;
                            for (const image_meta_msgs::NameValueString& message_name_value_enum_2 :
                                message_diag_2->name_value_enums)
                            {
                                if (message_name_value_enum_2.name == message_name_value_enum_1.name &&
                                    message_name_value_enum_2.value == message_name_value_enum_1.value)
                                {
                                    match_found = true;
                                    break;
                                }
                            }
                            no_differences = no_differences && match_found;
                        }

                        if (!no_differences)
                        {
                            std::cout << std::endl;
                            std::cout << "Message mismatch\n";
                            break;
                        }
                    }
                    else if (message_type_1 == "image_meta_msgs/ImageMeta")
                    {
                        image_meta_msgs::ImageMeta::Ptr message_meta_1 =
                            iterator_1->instantiate<image_meta_msgs::ImageMeta>();
                        image_meta_msgs::ImageMeta::Ptr message_meta_2 =
                            iterator_2->instantiate<image_meta_msgs::ImageMeta>();

                        // Sequence and frame IDs not set in H5 to rosbag conversion, so we don't compare them here
                        message_meta_1->hardware_header.seq = message_meta_2->hardware_header.seq;
                        message_meta_1->hardware_header.frame_id = message_meta_2->hardware_header.frame_id;
                        message_meta_1->driver_header.seq = message_meta_2->driver_header.seq;
                        message_meta_1->driver_header.frame_id = message_meta_2->driver_header.frame_id;

                        no_differences = no_differences &&
                            (message_meta_1->hardware_header == message_meta_2->hardware_header);
                        no_differences = no_differences &&
                            (message_meta_1->driver_header == message_meta_2->driver_header);
                        no_differences = no_differences && (message_meta_1->crc32 == message_meta_2->crc32);

                        // Note: the order of name-value pairs in not maintained through conversion
                        bool match_found = false;
                        for (const image_meta_msgs::NameValueFloat& message_name_value_float_1 :
                            message_meta_1->name_value_floats)
                        {
                            match_found = false;
                            for (const image_meta_msgs::NameValueFloat& message_name_value_float_2 :
                                message_meta_2->name_value_floats)
                            {
                                if (message_name_value_float_2.name == message_name_value_float_1.name &&
                                    message_name_value_float_2.value == message_name_value_float_1.value)
                                {
                                    match_found = true;
                                    break;
                                }
                            }
                            no_differences = no_differences && match_found;
                        }

                        for (const image_meta_msgs::NameValueInt& message_name_value_int_1 :
                            message_meta_1->name_value_ints)
                        {
                            match_found = false;
                            for (const image_meta_msgs::NameValueInt& message_name_value_int_2 :
                                message_meta_2->name_value_ints)
                            {
                                if (message_name_value_int_2.name == message_name_value_int_1.name &&
                                    message_name_value_int_2.value == message_name_value_int_1.value)
                                {
                                    match_found = true;
                                    break;
                                }
                            }
                            no_differences = no_differences && match_found;
                        }

                        for (const image_meta_msgs::NameValueString& message_name_value_string_1 :
                            message_meta_1->name_value_strings)
                        {
                            match_found = false;
                            for (const image_meta_msgs::NameValueString& message_name_value_string_2 :
                                message_meta_2->name_value_strings)
                            {
                                if (message_name_value_string_2.name == message_name_value_string_1.name &&
                                    message_name_value_string_2.value == message_name_value_string_1.value)
                                {
                                    match_found = true;
                                    break;
                                }
                            }
                            no_differences = no_differences && match_found;
                        }

                        if (!no_differences)
                        {
                            std::cout << std::endl;
                            std::cout << "Message mismatch\n";
                            break;
                        }
                    }
                    else if (message_type_1 == "sensor_msgs/Image")
                    {
                        sensor_msgs::Image::Ptr message_image_1 =
                            iterator_1->instantiate<sensor_msgs::Image>();
                        sensor_msgs::Image::Ptr message_image_2 =
                            iterator_2->instantiate<sensor_msgs::Image>();

                        // Sequence and frame IDs not set in H5 to rosbag conversion, so we don't compare them here
                        message_image_1->header.seq = message_image_2->header.seq;
                        message_image_1->header.frame_id = message_image_2->header.frame_id;

                        if (!(*message_image_1 == *message_image_2))
                        {
                            std::cout << std::endl;
                            std::cout << "Message mismatch\n";
                            no_differences = false;
                            break;
                        }
                    }
                    else if (message_type_1 == "geometry_msgs/PoseStamped")
                    {
                        geometry_msgs::PoseStamped::Ptr message_pose_1 =
                            iterator_1->instantiate<geometry_msgs::PoseStamped>();
                        geometry_msgs::PoseStamped::Ptr message_pose_2 =
                            iterator_2->instantiate<geometry_msgs::PoseStamped>();

                        // Sequence and frame IDs not set in H5 to rosbag conversion, so we don't compare them here
                        message_pose_1->header.seq = message_pose_2->header.seq;
                        message_pose_1->header.frame_id = message_pose_2->header.frame_id;

                        if (!(*message_pose_1 == *message_pose_2))
                        {
                            std::cout << std::endl;
                            std::cout << "Message mismatch\n";
                            no_differences = false;
                            break;
                        }
                    }
                    else
                    {
                        std::cout << "Skipping, the message type is not supported\n";
                        break;
                    }

                    iterator_1++;
                    iterator_2++;

                    message_count_1++;
                    std::cout << "\33[2K\rPercent completion: " << 100.0 * message_count_1 / n_messages_1 << std::flush;
                }
                std::cout << std::endl;
            }
        }

        if (no_differences)
        {
            std::cout << "No differences found!\n";
        }

        std::cout << "\n----------------------------------------\n";
    }

    // Close the rosbags
    bag_1.close();
    bag_2.close();

    return 0;
}