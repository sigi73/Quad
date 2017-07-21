#include "ros/ros.h"
#include "mavros_msgs/OverrideRCIn.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "movement_testing");
    ros::NodeHandle n;

    ros::Publisher movement_pub = n.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 1000);

    int currentState = 0; 
    mavros_msgs::OverrideRCIn msg;
    ros::Rate loop_rate(100);
    for (int i = 0; i < 8; i++)
    {
        msg.channels[i] = 65535;
    }

    double segmentStartTime = ros::Time::now().toSec();
    while (ros::ok())
    {
        switch(currentState)
        {
            case 0:
            {
                msg.channels[2] = 1500;
                if (ros::Time::now().toSec() > segmentStartTime + 1)
                {
                    currentState++;
                    segmentStartTime = ros::Time::now().toSec();
                }
                break;
            }
            case 1:
            {
                msg.channels[2] = 1200;
                if (ros::Time::now().toSec() > segmentStartTime + 1)
                {
                    currentState++;
                    segmentStartTime = ros::Time::now().toSec();
                }
                break;
            }
            case 2:
            {
                msg.channels[2] = 65535;
                if (ros::Time::now().toSec() > segmentStartTime + 1)
                {
                    currentState++;
                    segmentStartTime = ros::Time::now().toSec();
                }
                break;
            }
            default:
            {
                ros::shutdown();
            }
        }

        movement_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
}