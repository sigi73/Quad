#include "opencv2/opencv.hpp"
#include <vector>
#include <iostream>

using namespace cv;
using namespace std;

Mat src;
Mat result;
Mat img_template;

int match_method;
int max_trackbar = 5;

void MatchingMethod(int, void*)
{
}

int main(int, char **argv) 
{
    img_template = imread(argv[1], CV_LOAD_IMAGE_COLOR);

    VideoCapture cap(0);
    if (!cap.isOpened())
        return -1;

    namedWindow("result", CV_WINDOW_AUTOSIZE);
    namedWindow("src", CV_WINDOW_AUTOSIZE);

    char* trackbar_label = (char*)"Method: \n 0: SQDIFF \n 1: SQDIFF NORMED \n 2: TM CCORR \n 3: TM CCORR NORMED \n 4: TM COEFF \n 5: TM COEFF NORMED";

    createTrackbar(trackbar_label, "result", &match_method, max_trackbar, MatchingMethod);

    for (;;)
    {
        cap >> src;
        int result_cols = src.cols - img_template.cols + 1;
        int result_rows = src.rows - img_template.rows + 1;

        result.create(result_rows, result_cols, CV_32FC1);

        matchTemplate(src, img_template, result, match_method);
        normalize(result, result, 0, 1, NORM_MINMAX, -1, Mat());

        double minVal; double maxVal; Point minLoc; Point maxLoc;
        Point matchLoc;

        minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, Mat());

        if (match_method == CV_TM_SQDIFF || match_method == CV_TM_SQDIFF_NORMED) 
            matchLoc = minLoc;
        else matchLoc = maxLoc;

        rectangle(src, matchLoc, Point(matchLoc.x + img_template.cols, matchLoc.y + img_template.rows), Scalar::all(0), 2, 8, 0);
        rectangle(result, matchLoc, Point(matchLoc.x + img_template.cols, matchLoc.y + img_template.rows), Scalar::all(0), 2, 8, 0);

        imshow("result", result);
        imshow("src", src);


        if ((char)waitKey(30) >= 0) break;
    }

    return 0;
}

