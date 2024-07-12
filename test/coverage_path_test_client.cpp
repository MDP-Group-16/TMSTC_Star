#include "ros/ros.h"
#include "TMSTC_Star/CoveragePath.h"
#include "nav_msgs/GetMap.h"
#include "std_msgs/Header.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include <cstdlib>
#include <string>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "coverage_path_client");
    ros::NodeHandle n;

    std::string map_file_name;

    std::vector<double> robot1_pos = {.8, .9};
    //std::vector<double> robot2_pos = {-1.7, 1.2};

    ROS_INFO("Starting coverage path planner test client.");

    ros::ServiceClient map_client = n.serviceClient<nav_msgs::GetMap>("static_map");
    ros::ServiceClient client = n.serviceClient<TMSTC_Star::CoveragePath>("get_coverage_path");
    ros::Publisher coverage_map_pub = n.advertise<nav_msgs::OccupancyGrid>("coverage_map", 1);
    ros::Publisher rob_pos_pub = n.advertise<geometry_msgs::PoseArray>("test_init_poses", 1);

    ros::Publisher path1_pub = n.advertise<nav_msgs::Path>("test_path1", 1);
    ros::Publisher path2_pub = n.advertise<nav_msgs::Path>("test_path2", 1);

    
    ros::Duration(.5).sleep();

    ROS_INFO("Getting Map from map server.");

    nav_msgs::GetMap map_srv;

    if (map_client.call(map_srv))
    {
        ROS_INFO("Map service called successfully");
    }
    else
    {
        ROS_ERROR("Failed to call map service");
        return 0;
    }

    //create pose array

    geometry_msgs::PoseArray inital_poses;

    std_msgs::Header header;
    header.frame_id = "map";
    inital_poses.header = header;

    geometry_msgs::Pose pos1;
    geometry_msgs::Pose pos2;

    pos1.position.x = robot1_pos[0];
    pos1.position.y = robot1_pos[1];
    pos1.position.z = 0;
    inital_poses.poses.push_back(pos1);

    // pos2.position.x = robot2_pos[0];
    // pos2.position.y = robot2_pos[1];
    // pos2.position.z = 0;
    // inital_poses.poses.push_back(pos2);

    //create service request

    TMSTC_Star::CoveragePath srv;

    srv.request.tool_width = 0.2;
    srv.request.map = map_srv.response.map;
    srv.request.initial_poses = inital_poses;

    ROS_INFO("Sending coverage map request to get_coverage_path service server.");
    nav_msgs::OccupancyGrid coverage_map;
    coverage_map = srv.response.coverage_map;

    if (client.call(srv))
    {
        ROS_INFO("Received coverage map");

        ROS_INFO("Continually publishing result until shutdown");
        while (ros::ok())
        {
            path1_pub.publish(srv.response.coverage_paths[0]);
            // path2_pub.publish(srv.response.coverage_paths[1]);
            rob_pos_pub.publish(inital_poses);
            coverage_map_pub.publish(srv.response.coverage_map);
            ros::Duration(.5).sleep();
        }
    }
    else
    {
        ROS_ERROR("Failed to call service get_coverage_path");
        return 1;
    }

    return 0;
}

