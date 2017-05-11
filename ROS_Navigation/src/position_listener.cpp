#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"

void posCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    ROS_INFO_STREAM("Received pose: " << msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argc, "position_listener");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/mavros/local_position/pose", 1000, posCa;;back); 
    ros::spin();
    return EXIT_SUCCESS;
}
