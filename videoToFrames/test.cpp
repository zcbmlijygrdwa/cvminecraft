#include "opencv2/opencv.hpp"
using namespace cv;
int main(int, char**)
{
    //VideoCapture cap(0); // open the default camera
    //VideoCapture cap(1); // open the second camera
    VideoCapture cap("IMG_9890.MOV"); // open the second camera
    if(!cap.isOpened())  // check if we succeeded
        return -1;
    Mat edges;
    namedWindow("edges",1);
    int count = 0;
    for(;;)
    {
        count++;
        Mat frame;
        cap >> frame; // get a new frame from camera

//write to file
// Save the frame into a file
std::string filename = ".jpg";
filename = std::to_string(count) + filename;
imwrite(filename, frame); // A JPG FILE IS BEING SAVED

        cvtColor(frame, edges, COLOR_BGR2GRAY);
        GaussianBlur(edges, edges, Size(7,7), 1.5, 1.5);
        Canny(edges, edges, 0, 30, 3);
        imshow("edges", edges);
        if(waitKey(30) >= 0) break;
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}
