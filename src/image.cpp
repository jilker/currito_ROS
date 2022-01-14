#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

 int main( int argc, char** argv )
 {
    VideoCapture cap("/dev/video0"); //capture the video from webcam
    Mat output;
    
    if ( !cap.isOpened() )  // if not success, exit program
    {
         cout << "Cannot open the web cam" << endl;
         return -1;
    }
    while(1)
    {
    
    cap >> output;

    imshow("webcam input", output);
    char c = (char)waitKey(10);
    if( c == 27 ) break;
    }
 }