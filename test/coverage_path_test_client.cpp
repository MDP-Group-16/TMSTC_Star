#include "ros/ros.h"
#include "TMSTC_Star/CoveragePath.h"
#include "nav_msgs/GetMap.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include <cstdlib>
#include <string>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "coverage_path_client");
    ros::NodeHandle n;

    std::string map_file_name;

    std::vector<double> robot1_pos = {2.5, 2.5};
    std::vector<double> robot2_pos = {10, 10};
    std::vector<double> robot3_pos = {17.5, 17.5};

    ROS_INFO("Starting coverage path planner test client.");

    ros::ServiceClient map_client = n.serviceClient<nav_msgs::GetMap>("static_map");
    ros::ServiceClient client = n.serviceClient<TMSTC_Star::CoveragePath>("get_coverage_path");
    ros::Publisher coverage_map_pub = n.advertise<nav_msgs::OccupancyGrid>("coverage_map", 1);
    ros::Publisher rob_pos_pub = n.advertise<geometry_msgs::PoseArray>("test_init_poses", 1);
    
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
    geometry_msgs::Pose pos3;

    pos1.position.x = robot1_pos[0];
    pos1.position.y = robot1_pos[1];
    pos1.position.z = 0;
    inital_poses.poses.push_back(pos1);

    pos2.position.x = robot2_pos[0];
    pos2.position.y = robot2_pos[1];
    pos2.position.z = 0;
    inital_poses.poses.push_back(pos2);

    pos3.position.x = robot3_pos[0];
    pos3.position.y = robot3_pos[1];
    pos3.position.z = 0;
    inital_poses.poses.push_back(pos3);

    //create service request

    TMSTC_Star::CoveragePath srv;

    srv.request.num_robots = 3;
    srv.request.tool_width = 0.220;
    srv.request.map = map_srv.response.map;
    srv.request.initial_poses = inital_poses;

    ROS_INFO("Sending coverage map request to get_coverage_path service server.");

    if (client.call(srv))
    {
        ROS_INFO("Received coverage map");

        ROS_INFO("Continually publishing result until shutdown");
        while (ros::ok())
        {
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

