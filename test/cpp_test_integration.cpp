#include "ros/ros.h"
#include "TMSTC_Star/CoveragePath.h"
#include "nav_msgs/GetMap.h"
#include "std_msgs/Header.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose.h"
#include <geometry_msgs/PoseStamped.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseArray.h"
#include <cstdlib>
#include <string>
#include <tf/transform_listener.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "coverage_path_client");
    ros::NodeHandle n;
    ROS_INFO("Starting coverage path planner test client.");
    
    ROS_INFO("Getting robot pose.");
    tf::TransformListener listener;

    // Allow the listener to receive some transforms
    ros::Duration(2.0).sleep();

    // Example pose in base_link frame
    geometry_msgs::PoseStamped pose_in_base_link;
    pose_in_base_link.header.frame_id = "base_link";
    pose_in_base_link.header.stamp = ros::Time::now();
    pose_in_base_link.pose.orientation = tf::createQuaternionMsgFromYaw(0);

    geometry_msgs::PoseStamped pose_in_map;
    try{
        listener.waitForTransform("map", "base_link", ros::Time::now(), ros::Duration(4.0));
        listener.transformPose("map", pose_in_base_link, pose_in_map);

        ROS_INFO("Pose in map frame: x: %f, y: %f, z: %f", 
                pose_in_map.pose.position.x, 
                pose_in_map.pose.position.y, 
                pose_in_map.pose.position.z);
    }
    catch(tf::TransformException& ex){
        ROS_ERROR("Received an exception trying to transform a pose: %s", ex.what());
    }
    geometry_msgs::Pose pos1 = pose_in_map.pose;


    ROS_INFO("Getting map.");
    boost::shared_ptr<const nav_msgs::OccupancyGrid> map_ptr = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("map");

    ROS_INFO("Creating request.");
    ros::ServiceClient client = n.serviceClient<TMSTC_Star::CoveragePath>("get_coverage_path");
    ros::Publisher coverage_map_pub = n.advertise<nav_msgs::OccupancyGrid>("coverage_map", 1);
    ros::Publisher rob_pos_pub = n.advertise<geometry_msgs::PoseArray>("test_init_poses", 1);

    ros::Publisher path1_pub = n.advertise<nav_msgs::Path>("cpp_path1", 1);
    //ros::Publisher path2_pub = n.advertise<nav_msgs::Path>("test_path2", 1);
    //ros::Publisher path3_pub = n.advertise<nav_msgs::Path>("test_path3", 1);

    nav_msgs::OccupancyGrid map = *map_ptr;
    ros::Duration(.5).sleep();

    //create pose array
    geometry_msgs::PoseArray inital_poses;

    std_msgs::Header header;
    header.frame_id = "map";
    inital_poses.header = header;
    inital_poses.poses.push_back(pos1);

    //create service request

    TMSTC_Star::CoveragePath srv;

    srv.request.tool_width = 0.18;
    srv.request.map = map;
    srv.request.initial_poses = inital_poses;

    ROS_INFO("Sending coverage map request to get_coverage_path service server.");
    nav_msgs::OccupancyGrid coverage_map;

    if (client.call(srv))
    {
        ROS_INFO("Received coverage map");

        ROS_INFO("Continually publishing result until shutdown");
        while (ros::ok())
        {
            path1_pub.publish(srv.response.coverage_paths[0]);
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

