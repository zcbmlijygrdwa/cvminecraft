#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{

    const char* default_file = "../../data/image/apple.jpg";
    const char* filename = argc >=2 ? argv[1] : default_file;

    Mat src, gray;
    src = imread(filename, 1 );
    resize(src,src, cv::Size(),0.5,0.5);
    cvtColor( src, gray, CV_BGR2GRAY );

    // Reduce the noise so we avoid false circle detection
    GaussianBlur( gray, gray, Size(9, 9), 2, 2 );

    vector<Vec3f> circles;

    int minDist = 20;   //Minimum distance between the centers of the detected circles. 

    // Apply the Hough Transform to find the circles
    HoughCircles( gray, circles, CV_HOUGH_GRADIENT, 1, minDist, 200, 55, 0, 0 );

    // Draw the circles detected
    Mat circles_only(Size(src.cols,src.rows),src.type());
    circles_only.setTo(Scalar(255,255,255));// fill with color
    
    for( size_t i = 0; i < circles.size(); i++ )
    {
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);     
        circle( src, center, 3, Scalar(89,206,88), -1, 8, 0 );// circle center     
        circle( src, center, radius, Scalar(26,125,233), 3, 8, 0 );// circle outline
        
        circle( circles_only, center, radius, Scalar(0,0,0), 3, 8, 0 );// circle outline
        cout << "center : " << center << "\nradius : " << radius << endl;
    }

    // Show your results
    namedWindow( "Hough Circle Transform Demo", CV_WINDOW_AUTOSIZE );
    imshow( "Hough Circle Transform Demo", src );


    imshow("circles_only",circles_only);

    waitKey(0);
    return 0;
}

