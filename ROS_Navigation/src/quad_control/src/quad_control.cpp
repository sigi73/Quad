#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"

#include <sstream>

#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cmath>
#include <iostream>

static double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0)
{
  double dx1 = pt1.x - pt0.x;
  double dy1 = pt1.y - pt0.y;
  double dx2 = pt2.x - pt0.x;
  double dy2 = pt2.y - pt0.y;
  return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

cv::Mat updated;
bool imageFound = false;

void cameraCallback(const sensor_msgs::ImageConstPtr &msg)
{
  ROS_INFO("Camera_callback called");
  cv_bridge::CvImagePtr cv_ptr; 
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch(cv_bridge::Exception &e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what()); 
    return;
  }
  updated = cv_ptr->image;
  imageFound = true;
}



/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "quad_control");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("quad_movement", 1000);
  //ros::Subscriber sub = n.subscribe("picture_streamer", 1000, cameraCallback);
  ros::Subscriber sub = n.subscribe("/cv_camera/image_raw", 1000, cameraCallback);

  ros::Rate loop_rate(10);



  /*
  cv::VideoCapture cap(0);
  if (!cap.isOpened())
  {
    printf("No camera\n");
    return -1;
  }
  */

    
  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;

  while (ros::ok())
  {

    cv::Mat src;
    /*
    cap >> src;
    if (src.empty()) return -1;
    */
    //ROS_INFO("Control loop");
    ros::spinOnce();
    if (!imageFound) continue;
    src = updated;


    cv::Mat gray;
    cv::cvtColor(src, gray, CV_BGR2GRAY);

    cv::Mat bw;
    cv::Canny(gray, bw, 0, 50, 5);

    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(bw.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    std::vector<cv::Point> approx;
    cv::Mat dst = src.clone();

    std::vector<cv::Point> triangleShape;
    std::vector<cv::Point> circleShape;

    for (int i = 0; i < contours.size(); i++)
    {
      cv::approxPolyDP(cv::Mat(contours[i]), approx, cv::arcLength(cv::Mat(contours[i]), true) * 0.02, true);  

      if (std::fabs(cv::contourArea(contours[i])) < 2000 || !cv::isContourConvex(approx))
        continue;
      
      if (approx.size() == 3)
      {
        triangleShape = contours[i];
      }
      else if (approx.size() >= 4 && approx.size() <= 6)
      {
      }
      else
      {
        double area = cv::contourArea(contours[i]);
        cv::Rect r = cv::boundingRect(contours[i]);
        int radius = r.width / 2;

        if (std::abs(1 - ((double)r.width / r.height)) < 0.2 &&
            std::abs(1 - (area / (CV_PI * std::pow(radius, 2)))) <= 0.2)
        {
          circleShape = contours[i];
        }
      }
    }

    cv::Moments circleMoment = cv::moments(circleShape, false);
    cv::Moments triangleMoment = cv::moments(triangleShape, false);
    cv::Point2f circleCenter = cv::Point2f(circleMoment.m10 / circleMoment.m00, circleMoment.m01 / circleMoment.m00);
    cv::Point2f triangleCenter = cv::Point2f(triangleMoment.m10 / triangleMoment.m00, triangleMoment.m01 / triangleMoment.m00);

    cv::Point2f paperCenter = cv::Point2f((circleCenter.x + triangleCenter.x) / 2.0f, (circleCenter.y + triangleCenter.y) / 2.0f);

    float horizontalSpacing = 200;
    float xTarget = (float)dst.cols / 2.0f;
    float yTarget = (float)dst.rows / 2.0f;

    std_msgs::String msg;
    std::stringstream ss;

    if (paperCenter.y > yTarget)
    {    
      ss << "-Move up-";
    }
    else if (paperCenter.y < yTarget)
    {
      ss << "-Move down-";
    }

    if (paperCenter.x > xTarget)
    {
      ss << "Move right";
    }
    else if (paperCenter.x < xTarget)
    {
      ss << "Move left";
    }

    float dist = cv::norm(triangleCenter - circleCenter);
    if (dist < horizontalSpacing)
    {
      ss << "Move forward";
    }
    else if (dist > horizontalSpacing)
    {
      ss << "Move backward";
    }

    msg.data = ss.str();

    //ROS_INFO("%s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);


    loop_rate.sleep();
    ++count;
  }


  return 0;
}
