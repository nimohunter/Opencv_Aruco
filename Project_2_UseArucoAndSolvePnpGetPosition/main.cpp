#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <map>
#include <fstream>
#include <iostream>
using namespace cv;
using namespace std;

cv::Ptr<aruco::DetectorParameters> parameters;
cv::Ptr<aruco::Dictionary> dictionary=aruco::getPredefinedDictionary(aruco::DICT_6X6_250);

// Use for AurcoDetect
std::vector<int> ids;
std::vector<std::vector<cv::Point2f> > corners, rejectedCandidates;

// Use for SolvePnp input data
std::vector<cv::Point2f> imagePoints;
std::vector<cv::Point3f> objectPoints;
std::map<int ,std::vector<cv::Point3f> >  AurcoObjectPositionMap;

// Use for Calibration
cv::Mat cameraMatrix(3,3,cv::DataType<double>::type);
cv::Mat distCoeffs(5,1,cv::DataType<double>::type);

// use for Solvepnp result
cv::Mat rvec(3,1,cv::DataType<double>::type);
cv::Mat tvec(3,1,cv::DataType<double>::type);

ofstream oout;

void readCameraParameters()
{
    cv::setIdentity(cameraMatrix);
    cameraMatrix.at<double>(0,0) = 712.12;
    cameraMatrix.at<double>(0,2) = 367.371;
    cameraMatrix.at<double>(1,1) = 736.132;
    cameraMatrix.at<double>(1,2) = 236.552;

    distCoeffs.at<double>(0) = -0.531157;
    distCoeffs.at<double>(1) = 0.515348;
    distCoeffs.at<double>(2) = -0.0166326;
    distCoeffs.at<double>(3) = -0.00256654;
    distCoeffs.at<double>(4) = -0.536911;

}

void setObjectPoints()
{
    AurcoObjectPositionMap[31].push_back(cv::Point3f(2.7, 6.9, 6.2));
    AurcoObjectPositionMap[31].push_back(cv::Point3f(7.9, 6.9, 6.2));
    AurcoObjectPositionMap[31].push_back(cv::Point3f(7.9, 6.9, 1.0));
    AurcoObjectPositionMap[31].push_back(cv::Point3f(2.7, 6.9, 1.0));

    AurcoObjectPositionMap[114].push_back(cv::Point3f(0.0, 1.2, 1.4));
    AurcoObjectPositionMap[114].push_back(cv::Point3f(0.0, 1.2, 6.6));
    AurcoObjectPositionMap[114].push_back(cv::Point3f(0.0, 6.4, 6.6));
    AurcoObjectPositionMap[114].push_back(cv::Point3f(0.0, 6.4, 1.4));

    AurcoObjectPositionMap[131].push_back(cv::Point3f(2.1, 6.0, 0.0));
    AurcoObjectPositionMap[131].push_back(cv::Point3f(7.3, 6.0, 0.0));
    AurcoObjectPositionMap[131].push_back(cv::Point3f(7.3, 0.8, 0.0));
    AurcoObjectPositionMap[131].push_back(cv::Point3f(2.1, 0.8, 0.0));

    AurcoObjectPositionMap[230].push_back(cv::Point3f(9.2, 6.9, 6.6));
    AurcoObjectPositionMap[230].push_back(cv::Point3f(14.4, 6.9, 6.6));
    AurcoObjectPositionMap[230].push_back(cv::Point3f(14.4, 6.9, 1.4));
    AurcoObjectPositionMap[230].push_back(cv::Point3f(9.2, 6.9, 1.4));

    AurcoObjectPositionMap[56].push_back(cv::Point3f(9.5, 6.1, 0.0));
    AurcoObjectPositionMap[56].push_back(cv::Point3f(14.7, 6.1, 0.0));
    AurcoObjectPositionMap[56].push_back(cv::Point3f(14.7, 0.9, 0.0));
    AurcoObjectPositionMap[56].push_back(cv::Point3f(9.5, 0.9, 0.0));

}

void checkSolvePnPData()
{
    int num = 12;
    cout << "imagePoint | objectPoint" << endl;
    for (int i = 0; i < num; i++)
    {
        cout << imagePoints[i] << "|" << objectPoints[i] << endl;
    }

    std::cout << "Initial cameraMatrix: " << cameraMatrix << std::endl;
}

void ArucoDetect(Mat frame){
    cv::aruco::detectMarkers(frame, dictionary, corners, ids);
    int idsSize = ids.size();

    imagePoints.clear();
    objectPoints.clear();

    for (int i = 0; i < idsSize; i++)
    {
        int thisId = ids[i];
        for (int j = 0; j < 4; j++)
        {
            imagePoints.push_back(corners[i][j]);
            objectPoints.push_back(AurcoObjectPositionMap[thisId][j]);
        }

    }

    if (idsSize > 2)
    {
        //checkSolvePnPData();
        cv::solvePnPRansac(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);

        oout << rvec.at<double>(0, 0) << " " << rvec.at<double>(1, 0) << " "<< rvec.at<double>(2, 0) << endl;
        oout << tvec.at<double>(0, 0) << " " << tvec.at<double>(1, 0) << " "<< tvec.at<double>(2, 0) << endl;
    }

    if (ids.size() > 0){
        cv::aruco::drawDetectedMarkers(frame, corners, ids);
    }
}

int main()
{
    try {
    Mat frame;
    cv::VideoCapture cap;
    readCameraParameters();
    setObjectPoints();

    cap.open("/home/nimo/Videos/screencast_0209.avi");
    if (!cap.isOpened()) {
        cerr << "ERROR! Unable to open camera\n";
        return -1;
    }
    oout.open("screencast_0209.txt");

    for (;;)
    {
        // wait for a new frame from camera and store it into 'frame'
        cap.read(frame);
        // check if we succeeded
        if (frame.empty()) {
            cout << "ERROR! blank frame grabbed\n";
            break;
        }
        // show live and wait for a key with timeout long enough to show images
        ArucoDetect(frame);
        imshow("Live", frame);
        if (waitKey(100) >= 0)
        {
            //cout <<"break"<<endl;
        }
    }
    oout.close();
    } catch (std::exception &ex){
        cout<<"Nimo Exception :"<<ex.what()<<endl;
    }
    return 0;
}
