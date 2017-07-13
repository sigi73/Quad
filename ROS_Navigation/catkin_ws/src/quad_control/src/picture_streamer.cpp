#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>

#include "opencv2/opencv.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "picture_streamer");
  ros::NodeHandle n;
  
  ros::Publisher image_pub = n.advertise<sensor_msgs::Image>("picture_streamer", 1000);

  ros::Rate loop_rate(10);

  cv::VideoCapture cap(0);
  if (!cap.isOpened())
  {
    ROS_ERROR("No camera\n");
    return -1;
  }

  cv::Mat src;
  cv_bridge::CvImage out_msg;
  std_msgs::Header header;
  int counter = 0;
  while (ros::ok())
  {
    header.seq = counter;
    header.stamp = ros::Time::now();

    cap >> src;

    out_msg.header = header;
    out_msg.encoding = sensor_msgs::image_encodings::BGR8;
    out_msg.image = src;

    counter++;

    image_pub.publish(out_msg);

    ros::spinOnce();

    loop_rate.sleep();
    ROS_INFO("Streaming");
  }

}
