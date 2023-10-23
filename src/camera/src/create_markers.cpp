#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>

using namespace cv;

namespace {
const char* about = "Create an ArUco marker image";
const char* keys  =
        "{@outfile |QR_code.jpg | Output image }"
        "{d        | 10      | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
        "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
        "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
        "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}"
        "{id       | 10      | Marker id in the dictionary }"
        "{ms       | 200   | Marker size in pixels }"
        "{bb       | 1     | Number of bits in marker borders }"
        "{si       | true | show generated image }";
}


int main(int argc, char *argv[]) {

    int dictionaryId = 10;
    int markerId = 30;
    int borderBits = 1;
    int markerSize = 200;
    int bb =1;


    Ptr<cv::aruco::Dictionary> dictionary =
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

    cv::Mat markerImg;
    cv::aruco::drawMarker(dictionary, markerId, markerSize, markerImg, bb);

    cv::imwrite("QR_code2.png", markerImg);

    return 0;
}