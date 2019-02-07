
// https://docs.opencv.org/3.4/da/d97/tutorial_threshold_inRange.html

#include<opencv2/opencv.hpp>

using namespace cv;

int main(int argc, char** argv)
{

    

    const char* default_file1 = "../data/ground.png";
    const char* filename1 = argc >=3 ? argv[1] : default_file1;

    const char* default_file2 = "../data/with_parts.png";
    const char* filename2 = argc >=3 ? argv[2] : default_file2;

    //Input and Output Image;
    Mat input1, input2,diff;

    //Load the image
    input1 = imread(filename1,1);
    input2 = imread(filename2,1);


    diff = input1-input2;

    //Display input and output
    imshow("input1",input1);
    imshow("input2",input2);
    imshow("diff",diff);
    //imshow("Output",output);

    

    //imwrite("output.jpg",output);

    waitKey(0);
    return 0;
}
