
// https://docs.opencv.org/3.4/da/d97/tutorial_threshold_inRange.html

#include<opencv2/opencv.hpp>

using namespace cv;

int main(int argc, char** argv)
{

    

    const char* default_file = "../../data/image/lane1.jpg";
    const char* filename = argc >=2 ? argv[1] : default_file;

    //Input and Output Image;
    Mat input, input2,output;

    //Load the image
    input = imread(filename, 1 );

    const int max_value = 255;

    int low_H = (35.0/100)*max_value;
    int high_H = (43.0/100)*max_value;
    
    int low_S = (40.0/100)*max_value;
    int high_S = (72.0/100)*max_value;
    
    int low_V = (84.0/100)*max_value;
    int high_V = (100.0/100)*max_value;
    

    // Convert from BGR to HSV colorspace
    cvtColor(input, input2, COLOR_RGB2HSV);
    // Detect the object based on HSV Range Values
    inRange(input2, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), output);

    //Display input and output
    imshow("Input",input);
    imshow("Input2",input2);
    imshow("Output",output);


    imwrite("output.jpg",output);

    waitKey(0);
    return 0;
}
