#include "ros/ros.h"
#include "TMSTC_Star/CoveragePath.h"
#include "nav_msgs/GetMap.h"
#include "geometry_msgs/Pose.h"
#include <cstdlib>
#include <string>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "coverage_path_client");
    ros::NodeHandle n;

    std::string map_file_name;

    std::vector<double> robot1_pos = {2.5, 2.5};
    std::vector<double> robot2_pos = {0, 0};
    std::vector<double> robot3_pos = {-2.5, -2.5};
    

    ROS_INFO("Starting coverage path planner test client.");

    ros::ServiceClient map_client = n.serviceClient<nav_msgs::GetMap>("map");
    ros::ServiceClient client = n.serviceClient<TMSTC_Star::CoveragePath>("get_coverage_path");


    ROS_INFO("Getting Map from yaml file.");

    nav_msgs::GetMap map_srv;
    map_client.call(map_srv);


    std::vector<geometry_msgs::Pose> robot_positions;
    geometry_msgs::Pose pos1;
    geometry_msgs::Pose pos2;
    geometry_msgs::Pose pos3;

    pos1.position.x = robot1_pos[0];
    pos1.position.y = robot1_pos[1];
    pos1.position.z = 0;
    robot_positions.push_back(pos1);

    pos2.position.x = robot2_pos[0];
    pos2.position.y = robot2_pos[1];
    pos2.position.z = 0;
    robot_positions.push_back(pos2);

    pos3.position.x = robot3_pos[0];
    pos3.position.y = robot3_pos[1];
    pos3.position.z = 0;
    robot_positions.push_back(pos3);

    TMSTC_Star::CoveragePath srv;

    srv.request.num_robots = 3;
    srv.request.tool_width = 0.220;
    srv.request.map = map_srv.response.map;
    srv.request.initial_poses = robot_positions;

    ROS_INFO("Sending coverage map requestto server.");

    if (client.call(srv))
    {
        ROS_INFO("Call succeeded ");
    }
    else
    {
        ROS_ERROR("Failed to call service get_coverage_path");
        return 1;
    }

    while (ros::ok())
    {
        ROS_INFO("done");
        ros::Duration(.5).sleep();
    }

    return 0;
}
