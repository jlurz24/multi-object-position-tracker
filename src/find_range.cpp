#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
    Mat bgrImage = imread(argv[1]);
    Mat src;
    cv::cvtColor(bgrImage, src, CV_BGR2HSV);
   

    int hh = 255, hl = 0, sh = 255, sl = 0, vh = 255, vl = 0;

    string windowName = "background";
    namedWindow(windowName);

    createTrackbar("hh", windowName, &hh, 255);
    createTrackbar("hl", windowName, &hl, 255);
    createTrackbar("sh", windowName, &sh, 255);
    createTrackbar("sl", windowName, &sl, 255);
    createTrackbar("vh", windowName, &vh, 255);
    createTrackbar("vl", windowName, &vl, 255);

    // for dilation
    Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));

    Mat bgIsolation;
    int key = 0;
    do
    {
        inRange(src, Scalar(hl, sl, vl), Scalar(hh, sh, vh), bgIsolation);

        bitwise_not(bgIsolation, bgIsolation);

        erode(bgIsolation, bgIsolation, Mat());
        dilate(bgIsolation, bgIsolation, element);

        imshow(windowName, bgIsolation);
        key = waitKey(33);
    } while((char)key != 27);

    waitKey();

    return 0;
}
