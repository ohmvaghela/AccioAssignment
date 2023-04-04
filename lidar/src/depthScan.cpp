#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <unordered_map>
#include <lidar/Obstacles.h>
#include <vector>
// {obstacle number,{start angle, end angle, distance}}
typedef std::unordered_map<int, std::vector<float>> map_type;// 0 - start angle, 1 - end angle, 2 - distance

// Initially collect lidar feedback
std::vector<float> ranges(720, 0.0);
// Lidar Feedback for current timestep
std::vector<float> curr_timestep;

// Obstacles detected in current timestep
map_type curr_map;

/*
lidar::Obstacles 

float32[] distance
float32[] start_angle
float32[] end_angle
*/
lidar::Obstacles obstacles;

// Process data and indentify objects
void updateMap()
{
    // To store obstacles
    map_type object_map;

    // Temperory LIDAR feedback storage 
    std::vector<float> input_vec = curr_timestep;
    
    // For not revisting same index of vector
    std::vector<bool> visited(input_vec.size(), false);
    for (int i = 0; i < input_vec.size(); ++i)
    {
        // If index visited of index out of lidar range so skip
        if (visited[i] || input_vec[i] == std::numeric_limits<float>::infinity())
        {
            continue;
        }
        // If range in 2 meter
        // Though it is defined in urdf file but as an extra measure  
        if (input_vec[i] > 0 && input_vec[i] <= 2)
        {
            // Object detected at this angle, assign object number
            int object_num = object_map.size() + 1;

            // Temperory variables for iteration
            float start_angle = i;
            float end_angle = i;
            float temp_dist = 3; // 2 meter is max range 
            
            // Expand object range to adjacent angles for each start and end angles
            while (end_angle < input_vec.size() && (input_vec[end_angle] > 0 && input_vec[end_angle] <= 2))
            {
                visited[end_angle] = true;
                temp_dist = std::min(ranges[end_angle], temp_dist);
                ++end_angle;
            }
            while ((input_vec[start_angle] > 0 && input_vec[start_angle] <= 2))
            {
                visited[start_angle] = true;
                temp_dist = std::min(ranges[end_angle], temp_dist);
                --start_angle;
                if (start_angle < 0)
                    start_angle = input_vec.size() - 1;
            }
            ++start_angle;
            --end_angle;

            // Add object to unordered map
            ROS_INFO("bot %d: %f,%f", object_num, start_angle / 2, end_angle / 2);
            object_map[object_num] ={start_angle, end_angle,temp_dist};
        }
    }
    curr_map = object_map;
}

// Foramt data to be published
void UpdateObstacles()
{
    
    int sa, ea, dist;
    std::vector<float> start_angle;
    std::vector<float> end_angle;
    std::vector<float> distance;

    ROS_INFO("%i", (int)curr_map.size());
    for (auto &obstacle : curr_map)
    {
        std::vector<float> obj = obstacle.second;
        start_angle.push_back(obj[0]/2);
        end_angle.push_back(obj[1]/2);
        distance.push_back(obj[2]);
    }
    obstacles.start_angle = start_angle;
    obstacles.end_angle = end_angle;
    obstacles.distance = distance;
}

// Callback to recieve data from sensor
void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    // Extract the laser scan data
    ranges = scan->ranges;
}

int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "depth_finder");
    ros::NodeHandle nh;

    // Publihser for obstacles 
    ros::Publisher Obstacle_pub = nh.advertise<lidar::Obstacles>("PublishObstaclesTopic", 1000);

    // Subscribe to the laser scan topic
    ros::Subscriber sub = nh.subscribe<sensor_msgs::LaserScan>("/box/scan", 10, scanCallback);
    ros::Rate loop_rate(1);

    while (ros::ok())
    {
        // update input data 
        curr_timestep = ranges;
        // process data
        updateMap();
        // format data
        UpdateObstacles();
        // Publish data
        Obstacle_pub.publish(obstacles);
        
        
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
