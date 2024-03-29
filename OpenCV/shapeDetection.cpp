#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cmath>
#include <iostream>

/**
 * Helper function to find a cosine of angle between vectors
 * from pt0->pt1 and pt0->pt2
 */
static double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0)
{
	double dx1 = pt1.x - pt0.x;
	double dy1 = pt1.y - pt0.y;
	double dx2 = pt2.x - pt0.x;
	double dy2 = pt2.y - pt0.y;
	return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

/**
 * Helper function to display text in the center of a contour
 */
void setLabel(cv::Mat& im, const std::string type, std::vector<cv::Point>& contour)
{
	int fontface = cv::FONT_HERSHEY_SIMPLEX;
	double scale = 0.4;
	int thickness = 1;
	int baseline = 0;
    std::string label = type + " " + std::to_string(cv::contourArea(contour));

	cv::Size text = cv::getTextSize(label, fontface, scale, thickness, &baseline);
	cv::Rect r = cv::boundingRect(contour);

	cv::Point pt(r.x + ((r.width - text.width) / 2), r.y + ((r.height + text.height) / 2));
	cv::rectangle(im, pt + cv::Point(0, baseline), pt + cv::Point(text.width, -text.height), CV_RGB(255,255,255), CV_FILLED);
	cv::putText(im, label, pt, fontface, scale, CV_RGB(0,0,0), thickness, 8);
}

int main()
{
    cv::VideoCapture cap(0);
    if (!cap.isOpened())
    {
        printf("No camera\n");
        return -1;
    }

    //cv::namedWindow("src", CV_WINDOW_AUTOSIZE);
    //cv::namedWindow("dst", CV_WINDOW_AUTOSIZE);
    //cv::namedWindow("debug", CV_WINDOW_AUTOSIZE);
    
    for (;;)
    {
        cv::Mat src;
        cap >> src;
        if (src.empty()) {
            return -1;
        }


        // Convert to grayscale
        cv::Mat gray;
        cv::cvtColor(src, gray, CV_BGR2GRAY);

        // Use Canny instead of threshold to catch squares with gradient shading
        cv::Mat bw;
        cv::Canny(gray, bw, 0, 50, 5);

        // Find contours
        std::vector<std::vector<cv::Point> > contours;
        cv::findContours(bw.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

        std::vector<cv::Point> approx;
        cv::Mat dst = src.clone();

        //std::vector<std::vector<cv::Point>> targetShapes;
        std::vector<cv::Point> triangleShape;
        std::vector<cv::Point> circleShape;

        int j = 0;

        for (int i = 0; i < contours.size(); i++)
        {
            // Approximate contour with accuracy proportional
            // to the contour perimeter
            cv::approxPolyDP(cv::Mat(contours[i]), approx, cv::arcLength(cv::Mat(contours[i]), true)*0.02, true);

            // Skip small or non-convex objects 
            if (std::fabs(cv::contourArea(contours[i])) < 2000 || !cv::isContourConvex(approx))
                continue;

            j++;
            if (approx.size() == 3)
            {
                //setLabel(dst, "TRI", contours[i]);    // Triangles
                //targetShapes.push_back(contours[i]);
                triangleShape = contours[i];
            }
            else if (approx.size() >= 4 && approx.size() <= 6)
            {
                // Number of vertices of polygonal curve
                int vtc = approx.size();

                // Get the cosines of all corners
                std::vector<double> cos;
                for (int j = 2; j < vtc+1; j++)
                    cos.push_back(angle(approx[j%vtc], approx[j-2], approx[j-1]));

                // Sort ascending the cosine values
                std::sort(cos.begin(), cos.end());

                // Get the lowest and the highest cosine
                double mincos = cos.front();
                double maxcos = cos.back();

                // Use the degrees obtained above and the number of vertices
                // to determine the shape of the contour
                /*
                if (vtc == 4 && mincos >= -0.1 && maxcos <= 0.3)
                    setLabel(dst, "RECT", contours[i]);
                else if (vtc == 5 && mincos >= -0.34 && maxcos <= -0.27)
                    setLabel(dst, "PENTA", contours[i]);
                else if (vtc == 6 && mincos >= -0.55 && maxcos <= -0.45)
                    setLabel(dst, "HEXA", contours[i]);
                */
            }
            else
            {
                // Detect and label circles
                double area = cv::contourArea(contours[i]);
                cv::Rect r = cv::boundingRect(contours[i]);
                int radius = r.width / 2;

                if (std::abs(1 - ((double)r.width / r.height)) <= 0.2 &&
                    std::abs(1 - (area / (CV_PI * std::pow(radius, 2)))) <= 0.2)
                {
                    //setLabel(dst, "CIR", contours[i]);
                    //targetShapes.push_back(contours[i]);
                    circleShape = contours[i];
                }
            }
        }


        cv::Moments circleMoment = cv::moments(circleShape, false);
        cv::Moments triangleMoment = cv::moments(triangleShape, false);
        cv::Point2f circleCenter = cv::Point2f(circleMoment.m10 / circleMoment.m00, circleMoment.m01 / circleMoment.m00);
        cv::Point2f triangleCenter = cv::Point2f(triangleMoment.m10 / triangleMoment.m00, triangleMoment.m01 / triangleMoment.m00);
        cv::circle(dst, circleCenter, 4, cv::Scalar(0, 0, 255), -1, 8, 0);
        cv::circle(dst, triangleCenter, 4, cv::Scalar(0, 255, 0), -1, 8, 0);


        cv::Point2f paperCenter = cv::Point2f((circleCenter.x + triangleCenter.x) / 2.0f, (circleCenter.y + triangleCenter.y) / 2.0f);
        cv::circle(dst, paperCenter, 4, cv::Scalar(255, 0, 0), -1, 8, 0);

        //Draw target horizontal line
        //cv::line(dst, cv::Point2f(0.0f, (float)dst.rows / 2.0f), cv::Point2f((float)dst.cols, (float)dst.rows / 2.0f), 1, 8, 0);
        float horizontalSpacing = 200;
        float leftXTarget = ((float)dst.cols - horizontalSpacing) / 2.0f;
        float rightXTarget = ((float)dst.cols + horizontalSpacing) / 2.0f;
        float xTarget = (float)dst.cols / 2.0f;
        float yTarget = (float)dst.rows / 2.0f;
        cv::Point2f leftTarget = cv::Point2f(leftXTarget, yTarget);
        cv::Point2f rightTarget = cv::Point2f(rightXTarget, yTarget);
        cv::circle(dst, leftTarget, 8, cv::Scalar(255, 255, 0));
        cv::circle(dst, rightTarget, 8, cv::Scalar(255, 255, 0));

        bool printingSomething = false;
        if (paperCenter.y > yTarget)
        {
            //Move up
            printf("-Move up-");
            printingSomething = true;
        }
        else if (paperCenter.y < yTarget)
        {
            //Move down
            printf("-Move down-");
            printingSomething = true;
        }
        if (paperCenter.x > xTarget)
        {
            //Move right
            printf("-Move right-");
            printingSomething = true;
        }
        else if (paperCenter.x < xTarget)
        {
            //Move left
            printf("-Move left-");
            printingSomething = true;
        }
        float dist = cv::norm(triangleCenter - circleCenter);
        if (dist < horizontalSpacing)
        {
            printf("Move forward");
            printingSomething = true;
        }
        else if (dist > horizontalSpacing)
        {
            printf("Move backward");
            printingSomething = true;
        }
        if (printingSomething)
        {
            printf("\n");
        }





        //cv::imshow("src", src);
        //cv::imshow("dst", dst);
        //cv::imshow("debug", bw);
        //if ((char)cv::waitKey(30) >= 0) break;
    }
	return 0;
}

