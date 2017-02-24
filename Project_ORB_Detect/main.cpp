#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include "RobustMatcher.h"
#include <Utils.h>

using namespace cv;
using namespace std;

// Some basic colors
Scalar red(0, 0, 255);
Scalar green(0,255,0);
Scalar blue(255,0,0);
Scalar yellow(0,255,255);

int main()
{
    Mat img=imread("/home/nimo/Pictures/Koala.jpg");
    Mat img_in = img.clone();

    // set parameters
    int numKeyPoints = 10000;
    vector<KeyPoint> keypoints_model;
    Mat descriptors;

    RobustMatcher rmatcher;
    Ptr<FeatureDetector> detector = ORB::create(numKeyPoints);
    rmatcher.setFeatureDetector(detector);


    rmatcher.computeKeyPoints(img_in, keypoints_model);
    rmatcher.computeDescriptors(img_in, keypoints_model, descriptors);

    vector<Point2f> list_points_in;
    for (unsigned int i = 0; i < keypoints_model.size(); ++i) {
        Point2f point2d(keypoints_model[i].pt);
        list_points_in.push_back(point2d);
    }

    draw2DPoints(img, list_points_in, green);


    imshow("show",img);
    // 等待6000 ms后窗口自动关闭
    waitKey(0);
    return 0;
}
