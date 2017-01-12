#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/videoio/videoio.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/aruco.hpp"
#include <vector>
using namespace cv;
using namespace std;

int main()
{
    try {
    cv::Mat inputImage = imread("/home/nimo/Pictures/test/chessboard01.jpg");
    cv::Ptr<aruco::DetectorParameters> parameters;
    cv::Ptr<aruco::Dictionary> dictionary=aruco::getPredefinedDictionary(aruco::DICT_6X6_250);

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f> > corners, rejectedCandidates;
    cv::aruco::detectMarkers(inputImage, dictionary, corners, ids);

    cout << ids.size() << endl;
    if (ids.size() > 0){
        cv::aruco::drawDetectedMarkers(inputImage, corners, ids);
    }

    int idsSize = ids.size();
    for (int i = 0; i < idsSize; i++)
    {
        cout << "id:" << ids[i] << endl;
        cout << "corner:" << corners[i] << endl;
    }

    imshow("show", inputImage);
    waitKey();
    destroyAllWindows();
    } catch (std::exception &ex){
        cout<<"Exception :"<<ex.what()<<endl;
    }
}
