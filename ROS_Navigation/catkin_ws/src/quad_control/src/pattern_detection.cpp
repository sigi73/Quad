#include "ros/ros.h"
#include "AprilTags/TagDetector.h"
#include "AprilTags/Tag36h11.h"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <cmath>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


const char *windowName = "pattern_test";

cv::Mat updated;
bool imageFound = false;

void cameraCallback(const sensor_msgs::ImageConstPtr &msg)
{
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


#ifndef PI
const double PI = 3.14159265358979323846;
#endif
const double TWOPI = 2.0*PI;

inline double standardRad(double t) {
  if (t >= 0.) {
    t = fmod(t+PI, TWOPI) - PI;
  } else {
    t = fmod(t-PI, -TWOPI) + PI;
  }
  return t;
}

void wRo_to_euler(const Eigen::Matrix3d& wRo, double& yaw, double& pitch, double& roll) {
    yaw = standardRad(atan2(wRo(1,0), wRo(0,0)));
    double c = cos(yaw);
    double s = sin(yaw);
    pitch = standardRad(atan2(-wRo(2,0), wRo(0,0)*c + wRo(1,0)*s));
    roll  = standardRad(atan2(wRo(0,2)*s - wRo(1,2)*c, -wRo(0,1)*s + wRo(1,1)*c));
}


class PatternIdentifier
{
public:
    AprilTags::TagDetector *tagDetector;
    AprilTags::TagCodes tagCodes;

    int width;
    int height;
    double tagSize;
    double fx;
    double fy;
    double px;
    double py;
    bool drawWindow;

    PatternIdentifier() :
        tagDetector(NULL),
        tagCodes(AprilTags::tagCodes36h11),
        width(640 / 4),
        height(480 / 4),
        tagSize(0.166),
        /*
        //Pi values
        fx(528.695652174 / 4.0),
        fy(528.695652174 / 4.0),
        */
        fx(425.0 / 4.0),
        fy(425.0 / 4.0),
        px(width / 2.0),
        py(height / 2.0),
        drawWindow(true)
        {}
    
    void setup()
    {
        tagDetector = new AprilTags::TagDetector(tagCodes);
        if (drawWindow)
        {
            cv::namedWindow(windowName, 1);
        }
    }    

    void print_detection(AprilTags::TagDetection& detection) const {
    cout << "  Id: " << detection.id
         << " (Hamming: " << detection.hammingDistance << ")";

    // recovering the relative pose of a tag:

    // NOTE: for this to be accurate, it is necessary to use the
    // actual camera parameters here as well as the actual tag size
    // (m_fx, m_fy, m_px, m_py, m_tagSize)

    Eigen::Vector3d translation;
    Eigen::Matrix3d rotation;
    detection.getRelativeTranslationRotation(tagSize, fx, fy, px, py,
                                             translation, rotation);

    Eigen::Matrix3d F;
    F <<
      1, 0,  0,
      0,  -1,  0,
      0,  0,  1;
    Eigen::Matrix3d fixed_rot = F*rotation;
    double yaw, pitch, roll;
    wRo_to_euler(fixed_rot, yaw, pitch, roll);

    cout << "  distance=" << translation.norm()
         << "m, x=" << translation(0)
         << ", y=" << translation(1)
         << ", z=" << translation(2)
         << ", yaw=" << yaw
         << ", pitch=" << pitch
         << ", roll=" << roll
         << endl;
  }

  void processImage(cv::Mat& image, cv::Mat& image_gray) {
    // alternative way is to grab, then retrieve; allows for
    // multiple grab when processing below frame rate - v4l keeps a
    // number of frames buffered, which can lead to significant lag
    //      m_cap.grab();
    //      m_cap.retrieve(image);

    // detect April tags (requires a gray scale image)
    cv::cvtColor(image, image_gray, CV_BGR2GRAY);
    double t0;
    vector<AprilTags::TagDetection> detections = tagDetector->extractTags(image_gray);

    // print out each detection
    cout << detections.size() << " tags detected:" << endl;
    for (int i=0; i<detections.size(); i++) {
      print_detection(detections[i]);
    }

    // show the current image including any detections
    if (drawWindow) {
      for (int i=0; i<detections.size(); i++) {
        // also highlight in the image
        detections[i].draw(image);
      }
      imshow(windowName, image); // OpenCV call
    }
  }


};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pattern_detection");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/cv_camera/image_raw", 1000, cameraCallback);


    cv::Mat image;
    cv::Mat image_gray;

    PatternIdentifier patternIdentifier;
    patternIdentifier.setup();
    while (ros::ok())
    {
        ros::spinOnce();
        if (!imageFound) continue;

        image = updated;
        patternIdentifier.processImage(image, image_gray);

        if(patternIdentifier.drawWindow)
        {
            cv::waitKey(1);
        }
    }
}
