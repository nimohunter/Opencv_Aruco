#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <strstream>
using namespace cv;
using namespace std;

string myIntToString (int x){
    string temp;
    strstream ss;
    ss << x;
    ss >> temp;
    return temp;
}
int main()
{
    int id = 146;
    cv::Mat markerImage;
    cv::Ptr<aruco::Dictionary> dictionary=aruco::getPredefinedDictionary(aruco::DICT_6X6_250);
    cv::aruco::drawMarker(dictionary, id, 200, markerImage, 1);
    std::string filename = "/home/nimo/Pictures/Create/Marker" + myIntToString(id) + ".png";
    imwrite(filename, markerImage);

    imshow("show", markerImage);
    waitKey();
    destroyAllWindows();
}
