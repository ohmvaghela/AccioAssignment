#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <unordered_map>
// #include <utility>  // needed for std::pair
// #include <iostream>

typedef std::unordered_map<int, std::pair<int, int>> map_type;

std::vector<float> ranges(720, 0.0);

std::vector<float> prev_timestep;
std::vector<float> curr_timestep;
map_type curr_map;
map_type prev_map;
map_type final_map;
std::vector<map_type> object_trajectory;

void PrintMap()
{
    for (auto &TB : curr_map)
    {
        int curr_start = TB.second.first;
        int curr_end = TB.second.second;
        ROS_INFO("%i, %i", curr_start, curr_end);
    }
}

void updateMap()
{
    map_type object_map;
    std::vector<float> input_vec = curr_timestep;
    std::vector<bool> visited(input_vec.size(), false);
    for (int i = 0; i < input_vec.size(); ++i)
    {
        if (visited[i] || input_vec[i] == std::numeric_limits<float>::infinity())
        {
            continue;
        }
        if (input_vec[i] > 0 && input_vec[i] <= 2)
        {
            // Object detected at this angle, assign object number
            int object_num = object_map.size() + 1;
            int start_angle = i;
            int end_angle = i;
            // Expand object range to adjacent angles
            while (end_angle < input_vec.size() && (input_vec[end_angle] > 0 && input_vec[end_angle] <= 2))
            {
                visited[end_angle] = true;
                ++end_angle;
            }
            while ((input_vec[start_angle] > 0 && input_vec[start_angle] <= 2))
            {
                visited[start_angle] = true;
                --start_angle;
                if (start_angle < 0)
                    start_angle = input_vec.size() - 1;
            }
            ++start_angle;
            --end_angle;
            // Add object to unordered map
            ROS_INFO("%i: %i,%i",object_num,start_angle,end_angle);
            object_map[object_num] = std::make_pair(start_angle, end_angle);
        }
    }
    curr_map = object_map;
}

void updateCurrentMap()
{
    int cur_size = curr_map.size();
    ROS_INFO("size : %i",cur_size); 
    for (int i = 1; cur_size; ++i)
    {
        int max_id = 0;
        int loopTime = 0;
        for (auto &curr_obj : curr_map)
        {
            ROS_INFO("times : %i | size : %i",loopTime,cur_size);
            loopTime++;
            int curr_start = curr_obj.second.first;
            int curr_end = curr_obj.second.second;
            ROS_INFO("2");
            std::pair<int, int> temp_range = {curr_start, curr_end};
            bool found_intersect = false;
            for (auto &prev_obj : prev_map)
            {
                // ROS_INFO("3");
                int prev_start = prev_obj.second.first;
                int prev_end = prev_obj.second.second;
                // ROS_INFO("4");
                // if curcular end case occur in curr map
                if (curr_end < curr_start)
                {
                    // ROS_INFO("5");
                    if (prev_end < prev_start)
                    {
                        // Objects intersect, assign same object ID as previous object
                        // curr_map.erase(curr_obj.first);
                        final_map[prev_obj.first] = temp_range;
                        // curr_map[prev_obj.first] = temp_range;
                        // curr_obj.first = prev_obj.first;
                        found_intersect = true;
                        break;
                    }
                    if (curr_start <= prev_start || curr_end >= prev_start)
                    {
                        // curr_map.erase(curr_obj.first);
                        final_map[prev_obj.first] = temp_range;
                        // curr_map[prev_obj.first] = temp_range;
                        // curr_obj.first = prev_obj.first;
                        found_intersect = true;
                        break;
                    }
                }
                else // for normal case
                {
                    // ROS_INFO("6");
                    // if circular case in prev map
                    if (prev_end < prev_start)
                    {
                        if (prev_start <= curr_end || prev_end >= curr_start)
                        {
                            // Objects intersect, assign same object ID as previous object
                            // curr_map.erase(curr_obj.first);
                            final_map[prev_obj.first] = temp_range;
                            // curr_map[prev_obj.first] = temp_range;
                            // curr_obj.first = prev_obj.first;
                            found_intersect = true;
                            break;
                        }
                    }
                    if (prev_start <= curr_end && prev_end >= curr_start)
                    {
                        // Objects intersect, assign same object ID as previous object
                        // curr_map.erase(curr_obj.first);
                        final_map[prev_obj.first] = temp_range;
                        // curr_map[prev_obj.first] = temp_range;
                        // curr_obj.first = prev_obj.first;
                        found_intersect = true;
                        break;
                    }
                }
                // ROS_INFO("7");
            }
            if (!found_intersect)
            {
                // ROS_INFO("8");
                // Object does not intersect with any old object, assign new object ID
                max_id = std::max((int)curr_map.size(), max_id) + 1;
                // ROS_INFO("9");
                // curr_map.erase(curr_obj.first);
                curr_map[max_id] = temp_range;
                // ROS_INFO("11");
            }
        }
    }
    object_trajectory.push_back(final_map);
    prev_map = final_map;
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    // Extract the laser scan data
    ranges = scan->ranges;
    // Find the minimum range value
    float min_range = *std::min_element(ranges.begin(), ranges.end());
    // Calculate the depth
    float max_range = scan->range_max;
    float depth = max_range - min_range;

    // Publish the depth value to a ROS topic or use it for further processing
    // ROS_INFO("Depth: %f", depth);
    // ROS_INFO("Depth: %li", ranges.size());
}

int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "depth_finder");
    ros::NodeHandle nh;

    // Subscribe to the laser scan topic
    ros::Subscriber sub = nh.subscribe<sensor_msgs::LaserScan>("/box/scan", 10, scanCallback);
    ros::Rate loop_rate(5);

    while (ros::ok())
    {
        curr_timestep = ranges;
        updateMap();
        // if ((int)object_trajectory.size() < 1)
        // {
        //     prev_map = final_map;
        //     object_trajectory.push_back(curr_map);
        //     ROS_INFO("stuck here");
        //     continue;
        // }
        // updateCurrentMap();
        // PrintMap();
        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}
