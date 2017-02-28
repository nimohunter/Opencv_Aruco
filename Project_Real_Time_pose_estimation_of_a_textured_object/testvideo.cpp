// C++
#include <iostream>
#include <time.h>
// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/video/tracking.hpp>

using namespace std;
using namespace cv;

int main()
{
    try {
        Mat frame;
        cv::VideoCapture cap;

        cap.open("/home/nimo/Videos/0228.avi");
        if (!cap.isOpened()) {
            cout << "Camera does not open";
            return -1;
        }

        for (;;)
        {
            cap.read(frame);

            if (frame.empty()) {
                cout << "ERROR! blank frame grabbed\n";
                continue;
            }

            imshow("Live", frame);
            if (waitKey(20) >= 0)
            {
                //cout <<"break"<<endl;
            }
        }

    }
    catch (std::exception &ex) {
        cout<<"Nimo Exception :"<<ex.what()<<endl;
    }
    return 0;
}
