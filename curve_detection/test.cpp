
// https://docs.opencv.org/3.4/da/d97/tutorial_threshold_inRange.html

#include <opencv2/opencv.hpp>
#include <ceres/ceres.h>
#include "CurveDetection.hpp"

using namespace cv;
using namespace std;
using namespace ceres;
int main(int argc, char** argv)
{
    const char* default_file = "../../data/image/lane1.jpg";
    const char* filename = argc >=2 ? argv[1] : default_file;

    //Input and Output Image;
    Mat input, input2,input3;

    //Load the image
    input = imread(filename, 1 );

    // first convert the image to grayscale
    cvtColor(input, input2, CV_RGB2GRAY);

    // then adjust the threshold to actually make it binary
    threshold(input2, input3, 100, 255, CV_THRESH_BINARY);


    CurveDetection cd;

    //cd.setColorInput(input);
    //cd.setGrayscaleInput(input2);
    cd.setBinaryInput(input3);
    cd.solve();
    vector<Point2d> result = cd.getResult();

    Mat output = Mat::zeros(Size(input.cols,input.rows),CV_8UC1);


    ////Mat.at<type>(row,col):
    //// -------------> col
    //// |
    //// |
    //// |
    //// |
    //// |
    //// |
    //// V
    //// row

    int trow = 0;
    int tcol = 0;

    for(int i = 0; i<result.size(); i++)
    {
        trow = result[i].y;
        tcol = result[i].x;
        //cout<<"tcol = "<<tcol<<", trow = "<<trow<<" ,input.cols = "<<input.cols<<", input.rows = "<<input.rows<<endl;
        //input.at<uchar>(trow,tcol) = 255;
        input.at<Vec3b>(trow,tcol) = Vec3b(255,0,0);
        output.at<uchar>(trow,tcol) = 255;
    }

    //Display input and output
    imshow("Input",input);
    imshow("Output",output);

    //imwrite("output.jpg",output);

    waitKey(0);
    return 0;
}
