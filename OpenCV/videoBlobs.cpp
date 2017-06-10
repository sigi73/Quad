#include "opencv2/opencv.hpp"
#include <vector>

using namespace cv;
using namespace std;

int main(int, char **argv) 
{
    VideoCapture cap(0);
    if (!cap.isOpened())
        return -1;

    SimpleBlobDetector::Params params;
    //params.minThreshold = 10;
    //params.maxThreshold = 200;

    params.filterByArea = false;
    params.minArea = 100;

    params.filterByCircularity = true;
    params.minCircularity = 0.85;

    params.filterByConvexity = false;
    params.minConvexity = 0.87;

    params.filterByInertia = false;
    Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);

    namedWindow("circles", 1);
    for (;;)
    {
        Mat src_gray;
        Mat src;
        cap >> src;
        /*
        cvtColor(src, src_gray, COLOR_BGR2GRAY);
        GaussianBlur(src_gray, src_gray, Size(9, 9), 2, 2);

        vector<Vec3f> circles;
        HoughCircles(src_gray, circles, HOUGH_GRADIENT, 1, src_gray.rows / 8, 200, 25, 0, 0);

        printf("Circle num: %d\n", circles.size());

        for (size_t i = 0; i < circles.size(); i++)
        {
            Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
            int radius = cvRound(circles[i][2]);

            circle(src, center, 3, Scalar(0, 255, 0), -1, LineTypes::LINE_8, 0);
            circle(src, center, radius, Scalar(255, 0, 0), 3, LineTypes::LINE_8, 0);
        }

        */
        
        cvtColor(src, src, COLOR_BGR2GRAY);
        GaussianBlur(src, src, Size(9, 9), 2, 2);
        std::vector<KeyPoint> keypoints;
        detector->detect(src, keypoints);

        drawKeypoints(src, keypoints, src_gray, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

        imshow("circles", src_gray);
        if ((char)waitKey(30) >= 0) break;
    }

    return 0;
}
