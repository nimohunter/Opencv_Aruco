


#include "PnPThread.h"
#include "COperatingSystemFactory.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <assert.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/video/tracking.hpp>
#include <map>
#include <fstream>
#include <iostream>
using namespace cv;
using namespace std;

void CameraInit();
void setObjectPoints();
void set_P_matrix( const cv::Mat &R_matrix, const cv::Mat &t_matrix);
void initKalmanFilter(cv::KalmanFilter &KF, int nStates, int nMeasurements, int nInputs, double dt);
void fillMeasurements( cv::Mat &measurements,
                       const cv::Mat &translation_measured, const cv::Mat &rotation_measured);
cv::Mat rot2euler(const cv::Mat & rotationMatrix);

// Use for AurcoDetect
cv::Ptr<aruco::DetectorParameters> parameters;
cv::Ptr<aruco::Dictionary> dictionary=aruco::getPredefinedDictionary(aruco::DICT_6X6_250);
std::vector<int> ids;
std::vector<std::vector<cv::Point2f> > corners, rejectedCandidates;

// Use for SolvePnp input data
std::vector<cv::Point2f> imagePoints;
std::vector<cv::Point3f> objectPoints;
std::map<int,std::vector<cv::Point3f> >  AurcoObjectPositionMap;

// Use for Calibration
cv::Mat cameraMatrix(3,3,cv::DataType<double>::type);
cv::Mat distCoeffs(5,1,cv::DataType<double>::type);

// use for Solvepnp result
cv::Mat _R_matrix;   // rotation matrix
cv::Mat _t_matrix;   // translation matrix
cv::Mat _P_matrix;   // rotation-translation matrix

ofstream oout;

//Kalman
cv::KalmanFilter KF;         // instantiate Kalman Filter
int nStates = 18;            // the number of states
int nMeasurements = 6;       // the number of measured states
int nInputs = 0;             // the number of action control
double dt = 0.125;           // time between measurements (1/FPS)


// RANSAC parameters
int iterationsCount = 100;      // number of Ransac iterations.
float reprojectionError = 8.0;  // maximum allowed distance to consider it an inlier.
double confidence = 0.95;        // ransac successful confidence.
int pnpMethod = SOLVEPNP_ITERATIVE;

// Kalman Filter parameters
int minInliersKalman = 6;    // Kalman threshold updating
Mat inliers;
Mat measurements(nMeasurements, 1, CV_64F);

unsigned int code=0;



void PnPThread::setMsgQueue(CMsgQueue *q)
{
    p_msg_queue = q;
}

void ArucoDetect(Mat frame) {
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
        cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);          // output rotation vector
        cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);          // output translation vector


        cv::solvePnPRansac(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec,
                           false, iterationsCount, reprojectionError, confidence,
                           inliers, pnpMethod);
        //cout << inliers.size() << endl;

        cv::Rodrigues(rvec, _R_matrix);
        _t_matrix = tvec;
        set_P_matrix(_R_matrix, _t_matrix);

    }

    if (ids.size() > 0) {
        cv::aruco::drawDetectedMarkers(frame, corners, ids);
    }
}

void fillMeasurements( cv::Mat &measurements,
                       const cv::Mat &translation_measured, const cv::Mat &rotation_measured)
{
    // Convert rotation matrix to euler angles
    cv::Mat measured_eulers(3, 1, CV_64F);
    measured_eulers = rot2euler(rotation_measured);
    // Set measurement to predict
    measurements.at<double>(0) = translation_measured.at<double>(0); // x
    measurements.at<double>(1) = translation_measured.at<double>(1); // y
    measurements.at<double>(2) = translation_measured.at<double>(2); // z
    measurements.at<double>(3) = measured_eulers.at<double>(0);      // roll
    measurements.at<double>(4) = measured_eulers.at<double>(1);      // pitch
    measurements.at<double>(5) = measured_eulers.at<double>(2);      // yaw
}

void updateKalmanFilter( cv::KalmanFilter &KF, cv::Mat &measurement,
                         cv::Mat &translation_estimated, cv::Mat &rotation_estimated, CMsgQueue *this_queue )
{
    // First predict, to update the internal statePre variable
    cv::Mat prediction = KF.predict();
    // The "correct" phase that is going to use the predicted value and our measurement
    cv::Mat estimated = KF.correct(measurement);
    // Estimated translation
    translation_estimated.at<double>(0) = estimated.at<double>(0);
    translation_estimated.at<double>(1) = estimated.at<double>(1);
    translation_estimated.at<double>(2) = estimated.at<double>(2);
    // Estimated euler angles
    cv::Mat eulers_estimated(3, 1, CV_64F);
    eulers_estimated.at<double>(0) = estimated.at<double>(9);
    eulers_estimated.at<double>(1) = estimated.at<double>(10);
    eulers_estimated.at<double>(2) = estimated.at<double>(11);
    // Convert estimated quaternion to rotation matrix
    //rotation_estimated = euler2rot(eulers_estimated);
//    oout << estimated.at<double>(0) << " " << estimated.at<double>(1) << " " << estimated.at<double>(2) << endl;
//    oout << estimated.at<double>(9) << " " << estimated.at<double>(10) << " " << estimated.at<double>(11) << endl;

    try {
//        oout <<  code << " " << estimated.at<double>(0) << " " << estimated.at<double>(1) << " " << estimated.at<double>(2) << endl;
//        oout <<  code << " " << estimated.at<double>(9) << " " << estimated.at<double>(10) << " " << estimated.at<double>(11) << endl;
        char PnPResult[256] ;
        sprintf(PnPResult, "%lf_%lf_%lf_%lf_%lf_%lf", estimated.at<double>(0), estimated.at<double>(1), estimated.at<double>(2),
                estimated.at<double>(9), estimated.at<double>(10), estimated.at<double>(11));
        cout << code << " " << PnPResult << endl;
        printf("[%d] %s\n", code, PnPResult);
        cout << code << "==== " << endl;
    }
    catch (std::exception &ex) {
        cout<<"Nimo Exception :"<<ex.what()<<endl;
    }
//    this_queue->sendMsg(code, (void *)PnPResult);
    code++;



}

void TheKalmanFilterWork(CMsgQueue *this_queue)
{
    // -- Step 5: Kalman Filter
    // GOOD MEASUREMENT
    if( inliers.rows >= minInliersKalman )
    {
        fillMeasurements(measurements, _t_matrix, _R_matrix);
    }
    // Instantiate estimated translation and rotation
    cv::Mat translation_estimated(3, 1, CV_64F);
    cv::Mat rotation_estimated(3, 3, CV_64F);
    // update the Kalman filter with good measurements
    updateKalmanFilter( KF, measurements,
                        translation_estimated, rotation_estimated, this_queue);
}



PnPThread::PnPThread(const char *m_name):
    CThread(m_name)
{

    //add your code here
}

PnPThread::~PnPThread()
{

}


void PnPThread::mainLoop()
{
//    unsigned int code=0;
//    //TODO
//    char  *p_msg="PnP Result";
//    while(true)
//    {
//        printf(">>>>>>>%s is Running....send data to message queue...\n",p_thread_name);
//        p_msg_queue->sendMsg(code, (void *)p_msg);
//        code++;
//        p_opration_system->sleepSec(1);
//    }
    code=0;
    try {
        CameraInit();
        setObjectPoints();
        initKalmanFilter(KF, nStates, nMeasurements, nInputs, dt);    // init function

        Mat frame;
        cv::VideoCapture cap;
        oout.open("screencast_0224.txt");

        cap.open("/home/nimo/Videos/screencast_mature.avi");
        if (!cap.isOpened()) {
            throw "Camera does not open";
        }

        for (;;)
        {
            cap.read(frame);

            if (frame.empty()) {
                cout << "ERROR! blank frame grabbed\n";
                break;
            }

            ArucoDetect(frame);
            TheKalmanFilterWork(p_msg_queue);

            imshow("Live", frame);
            if (waitKey(100) >= 0)
            {
                //cout <<"break"<<endl;
            }
        }
        oout.close();
    }
    catch (std::exception &ex) {
        cout<<"Nimo Exception :"<<ex.what()<<endl;
    }

    cout <<"end" << endl;



}



void initKalmanFilter(cv::KalmanFilter &KF, int nStates, int nMeasurements, int nInputs, double dt)
{
    measurements.setTo(Scalar(0));
    KF.init(nStates, nMeasurements, nInputs, CV_64F);                 // init Kalman Filter
    cv::setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-5));       // set process noise
    cv::setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1e-4));   // set measurement noise
    cv::setIdentity(KF.errorCovPost, cv::Scalar::all(1));             // error covariance
    /* DYNAMIC MODEL */
    //  [1 0 0 dt  0  0 dt2   0   0 0 0 0  0  0  0   0   0   0]
    //  [0 1 0  0 dt  0   0 dt2   0 0 0 0  0  0  0   0   0   0]
    //  [0 0 1  0  0 dt   0   0 dt2 0 0 0  0  0  0   0   0   0]
    //  [0 0 0  1  0  0  dt   0   0 0 0 0  0  0  0   0   0   0]
    //  [0 0 0  0  1  0   0  dt   0 0 0 0  0  0  0   0   0   0]
    //  [0 0 0  0  0  1   0   0  dt 0 0 0  0  0  0   0   0   0]
    //  [0 0 0  0  0  0   1   0   0 0 0 0  0  0  0   0   0   0]
    //  [0 0 0  0  0  0   0   1   0 0 0 0  0  0  0   0   0   0]
    //  [0 0 0  0  0  0   0   0   1 0 0 0  0  0  0   0   0   0]
    //  [0 0 0  0  0  0   0   0   0 1 0 0 dt  0  0 dt2   0   0]
    //  [0 0 0  0  0  0   0   0   0 0 1 0  0 dt  0   0 dt2   0]
    //  [0 0 0  0  0  0   0   0   0 0 0 1  0  0 dt   0   0 dt2]
    //  [0 0 0  0  0  0   0   0   0 0 0 0  1  0  0  dt   0   0]
    //  [0 0 0  0  0  0   0   0   0 0 0 0  0  1  0   0  dt   0]
    //  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  1   0   0  dt]
    //  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   1   0   0]
    //  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   0   1   0]
    //  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   0   0   1]
    // position
    KF.transitionMatrix.at<double>(0,3) = dt;
    KF.transitionMatrix.at<double>(1,4) = dt;
    KF.transitionMatrix.at<double>(2,5) = dt;
    KF.transitionMatrix.at<double>(3,6) = dt;
    KF.transitionMatrix.at<double>(4,7) = dt;
    KF.transitionMatrix.at<double>(5,8) = dt;
    KF.transitionMatrix.at<double>(0,6) = 0.5*pow(dt,2);
    KF.transitionMatrix.at<double>(1,7) = 0.5*pow(dt,2);
    KF.transitionMatrix.at<double>(2,8) = 0.5*pow(dt,2);
    // orientation
    KF.transitionMatrix.at<double>(9,12) = dt;
    KF.transitionMatrix.at<double>(10,13) = dt;
    KF.transitionMatrix.at<double>(11,14) = dt;
    KF.transitionMatrix.at<double>(12,15) = dt;
    KF.transitionMatrix.at<double>(13,16) = dt;
    KF.transitionMatrix.at<double>(14,17) = dt;
    KF.transitionMatrix.at<double>(9,15) = 0.5*pow(dt,2);
    KF.transitionMatrix.at<double>(10,16) = 0.5*pow(dt,2);
    KF.transitionMatrix.at<double>(11,17) = 0.5*pow(dt,2);
    /* MEASUREMENT MODEL */
    //  [1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
    //  [0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
    //  [0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
    //  [0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0]
    //  [0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0]
    //  [0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0]
    KF.measurementMatrix.at<double>(0,0) = 1;  // x
    KF.measurementMatrix.at<double>(1,1) = 1;  // y
    KF.measurementMatrix.at<double>(2,2) = 1;  // z
    KF.measurementMatrix.at<double>(3,9) = 1;  // roll
    KF.measurementMatrix.at<double>(4,10) = 1; // pitch
    KF.measurementMatrix.at<double>(5,11) = 1; // yaw
}

void set_P_matrix( const cv::Mat &R_matrix, const cv::Mat &t_matrix)
{
    // Rotation-Translation Matrix Definition
    _P_matrix.at<double>(0,0) = R_matrix.at<double>(0,0);
    _P_matrix.at<double>(0,1) = R_matrix.at<double>(0,1);
    _P_matrix.at<double>(0,2) = R_matrix.at<double>(0,2);
    _P_matrix.at<double>(1,0) = R_matrix.at<double>(1,0);
    _P_matrix.at<double>(1,1) = R_matrix.at<double>(1,1);
    _P_matrix.at<double>(1,2) = R_matrix.at<double>(1,2);
    _P_matrix.at<double>(2,0) = R_matrix.at<double>(2,0);
    _P_matrix.at<double>(2,1) = R_matrix.at<double>(2,1);
    _P_matrix.at<double>(2,2) = R_matrix.at<double>(2,2);
    _P_matrix.at<double>(0,3) = t_matrix.at<double>(0);
    _P_matrix.at<double>(1,3) = t_matrix.at<double>(1);
    _P_matrix.at<double>(2,3) = t_matrix.at<double>(2);
}

void CameraInit()
{
    //    cameraMatrix = (Mat_<double>(3, 3) << 712.12, 0, 367.371, 0, 736.132, 236.552, 0, 0, 1);
    //    distCoeffs = (Mat_<double>(1, 5) << -0.531157, 0.515348, -0.0166326, -0.00256654, -0.536911);

    //    cameraMatrix.resize(3, std::vector<double>(3));
    //    cameraMatrix[0][0] = 712.12;
    //    cameraMatrix[0][1] = 0;
    //    cameraMatrix[0][2] = 367.371;
    //    cameraMatrix[1][0] = 0;
    //    cameraMatrix[1][1] = 736.132;
    //    cameraMatrix[1][2] = 236.552;
    //    cameraMatrix[2][0] = 0;
    //    cameraMatrix[2][1] = 0;
    //    cameraMatrix[2][2] = 1;

    //    distCoeffs.resize(5);
    //    distCoeffs[0] = -0.531157;
    //    distCoeffs[1] =  0.515348;
    //    distCoeffs[2] =   -0.0166326;
    //    distCoeffs[3] =    -0.00256654;
    //    distCoeffs[4] =    -0.536911;

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

    _R_matrix = cv::Mat::zeros(3, 3, CV_64FC1);   // rotation matrix
    _t_matrix = cv::Mat::zeros(3, 1, CV_64FC1);   // translation matrix
    _P_matrix = cv::Mat::zeros(3, 4, CV_64FC1);   // rotation-translation matrix

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

cv::Mat rot2euler(const cv::Mat & rotationMatrix)
{
    cv::Mat euler(3,1,CV_64F);

    double m00 = rotationMatrix.at<double>(0,0);
    double m02 = rotationMatrix.at<double>(0,2);
    double m10 = rotationMatrix.at<double>(1,0);
    double m11 = rotationMatrix.at<double>(1,1);
    double m12 = rotationMatrix.at<double>(1,2);
    double m20 = rotationMatrix.at<double>(2,0);
    double m22 = rotationMatrix.at<double>(2,2);

    double x, y, z;

    // Assuming the angles are in radians.
    if (m10 > 0.998) { // singularity at north pole
        x = 0;
        y = CV_PI/2;
        z = atan2(m02,m22);
    }
    else if (m10 < -0.998) { // singularity at south pole
        x = 0;
        y = -CV_PI/2;
        z = atan2(m02,m22);
    }
    else
    {
        x = atan2(-m12,m11);
        y = asin(m10);
        z = atan2(-m20,m00);
    }

    euler.at<double>(0) = x;
    euler.at<double>(1) = y;
    euler.at<double>(2) = z;

    return euler;
}
