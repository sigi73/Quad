#include "ros/ros.h"
#include "AprilTags/TagDetector.h"
#include "AprilTags/Tag36h11.h"
#include "sensor_msgs/Image.h"
#include "mavros_msgs/OverrideRCIn.h"
#include "mavros_msgs/RCIn.h"
#include <cv_bridge/cv_bridge.h>
#include <cmath>
#include <vector>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#ifdef USE_VIZ
#include "visualization_msgs/Marker.h"
#endif


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

int ch6Val = 0;
void rcCallback(const mavros_msgs::RCIn &msg)
{
   ch6Val = msg.channels[5];
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

class Glyph
{
public:
    Glyph(Eigen::Vector3d location, int glyphId):
        realLocation(location),
        id(glyphId)
    {}
    void updateCameraTranslation(Eigen::Vector3d translation)
    {
        glyphTranslation = translation;
        cameraLocation = realLocation - translation;
        //printf("adjusted translation: x: %f, y: %f, z: %f\n", translation(2), translation(1), translation(0));
    }

    int getId()
    {
        return id;
    }
    Eigen::Vector3d getWorldLocation()
    {
        return realLocation;
    }
    Eigen::Vector3d getCameraLocation()
    {
        return cameraLocation;
    }

    Eigen::Vector3d realLocation;
    Eigen::Vector3d cameraLocation;
    Eigen::Vector3d glyphTranslation;
    int id;
};


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
    std::vector<Glyph*> glyphs;

    PatternIdentifier() :
        tagDetector(NULL),
        tagCodes(AprilTags::tagCodes36h11),
        width(640 / 4),
        height(480 / 4),
        tagSize(0.166),
        //Pi values
        fx(528.695652174 / 4.0),
        fy(528.695652174 / 4.0),
        /*
        fx(425.0 / 4.0),
        fy(425.0 / 4.0),
        */
        px(width / 2.0),
        py(height / 2.0),
        drawWindow(true)
        {
            Glyph *glyph0 = new Glyph(Eigen::Vector3d(0, 0, 0), 0);
            glyphs.push_back(glyph0);
        }
    
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

    for (int i = 0; i < glyphs.size(); i++)
    {
        if (glyphs[i]->getId() == detection.id)
        {
            glyphs[i]->updateCameraTranslation(translation);
        }
    }
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
    ros::Subscriber rc_sub = n.subscribe("/mavros/rc/in", 1000, rcCallback);

    #ifdef USE_VIZ
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    uint32_t glyphShape = visualization_msgs::Marker::CUBE;
    uint32_t cameraShape = visualization_msgs::Marker::SPHERE;
    #endif

    ros::Publisher movement_pub = n.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 1);

    cv::Mat image;
    cv::Mat image_gray;

    PatternIdentifier patternIdentifier;
    patternIdentifier.setup();
    while (ros::ok())
    {
        ros::spinOnce();
        if (!imageFound) continue;

        cv::resize(updated, image, cv::Size(640 / 4, 480 / 4));
        patternIdentifier.processImage(image, image_gray);

        if(patternIdentifier.drawWindow)
        {
            cv::waitKey(1);
        }

        mavros_msgs::OverrideRCIn msg;
        for (int i = 0; i < 8; i++)
        {
            msg.channels[i] = 0;
        }
        Eigen::Vector3d translation = patternIdentifier.glyphs[0]->glyphTranslation;
        if (translation(0) != 0 && ch6Val < 1700)
        {
            msg.channels[3] = 1500;
            if (translation(0) < 0.9)
            {
                //Move up
                msg.channels[2] = 1500;
            }
            else if (translation(0) > 1.1)
            {
                //Move down
                msg.channels[2] = 1200;
            }
            else
            {
                msg.channels[2] = 1350;
            }

            if (translation(1) < -0.05)
            {
                //Move right
                msg.channels[0] = 1600;
            }
            else if (translation(1) > 0.05)
            {
                //Move left
                msg.channels[0] = 1400;
            }
            else
            {
                msg.channels[0] = 1500;
            }

            if (translation(2) < -0.05)
            {
                //Move backward
                msg.channels[1] = 1400;
            }
            else if (translation(2) > 0.05)
            {
                //Move forward
                msg.channels[1] = 1600;
            }
            else
            {
                msg.channels[1] = 1500;
            }
        }

        movement_pub.publish(msg);

        #ifdef USE_VIZ
        if (marker_pub.getNumSubscribers() < 1)
        {
            continue;
        }
        
        for(int i = 0; i < patternIdentifier.glyphs.size(); i++)
        {
            Glyph glyph = *patternIdentifier.glyphs[i];
            visualization_msgs::Marker glyphMarker;
            visualization_msgs::Marker cameraMarker;

            glyphMarker.header.frame_id = "/quad_frame";
            cameraMarker.header.frame_id = "/quad_frame";

            glyphMarker.ns = "glyph" + glyph.getId();
            glyphMarker.id = glyph.getId();
            cameraMarker.ns = "camera" + glyph.getId();
            cameraMarker.id = glyph.getId();
            
            glyphMarker.type = glyphShape;
            glyphMarker.action = visualization_msgs::Marker::ADD;
            cameraMarker.type = cameraShape;
            cameraMarker.action = visualization_msgs::Marker::ADD;

            glyphMarker.pose.position.x = glyph.getWorldLocation()(0);
            glyphMarker.pose.position.y = glyph.getWorldLocation()(1);
            glyphMarker.pose.position.z = glyph.getWorldLocation()(2);
            cameraMarker.pose.position.x = glyph.getCameraLocation()(0);
            cameraMarker.pose.position.y = glyph.getCameraLocation()(1);
            cameraMarker.pose.position.z = glyph.getCameraLocation()(2);
            
            glyphMarker.pose.orientation.x = 0;
            glyphMarker.pose.orientation.y = 0;
            glyphMarker.pose.orientation.z = 0;
            glyphMarker.pose.orientation.w = 1;
            cameraMarker.pose.orientation.x = 0;
            cameraMarker.pose.orientation.y = 0;
            cameraMarker.pose.orientation.z = 0;
            cameraMarker.pose.orientation.w = 1;

            glyphMarker.scale.x = .1;
            glyphMarker.scale.y = .1;
            glyphMarker.scale.z = .1;
            cameraMarker.scale.x = .1;
            cameraMarker.scale.y = .1;
            cameraMarker.scale.z = .1;

            glyphMarker.color.r = 0;
            glyphMarker.color.g = 0;
            glyphMarker.color.b = 1;
            glyphMarker.color.a = 1;
            cameraMarker.color.r = 0;
            cameraMarker.color.g = 1;
            cameraMarker.color.b = 0;
            cameraMarker.color.a = 1;

            glyphMarker.lifetime = ros::Duration();
            cameraMarker.lifetime = ros::Duration();

            marker_pub.publish(glyphMarker);
            marker_pub.publish(cameraMarker);
        }
        #endif
    }
}
