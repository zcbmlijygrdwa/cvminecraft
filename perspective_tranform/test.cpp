#include<opencv2/opencv.hpp>
 
using namespace cv;
 
int main(int argc, char** argv)
{

const char* default_file = "../../data/image/lane1.jpg";
    const char* filename = argc >=2 ? argv[1] : default_file;


    // Input Quadilateral or Image plane coordinates
    Point2f inputQuad[4]; 
    // Output Quadilateral or World plane coordinates
    Point2f outputQuad[4];
         
    // Lambda Matrix
    Mat lambda( 2, 4, CV_32FC1 );
    //Input and Output Image;
    Mat input, output;
     
    //Load the image
    input = imread(filename, 1 );
    // Set the lambda matrix the same type and size as input
    lambda = Mat::zeros( input.rows, input.cols, input.type() );
 
    // The 4 points that select quadilateral on the input , from top-left in clockwise order
    // These four pts are the sides of the rect box used as input 
    //inputQuad[0] = Point2f( -30,-60 );
    //inputQuad[1] = Point2f( input.cols+50,-50);
    //inputQuad[2] = Point2f( input.cols+100,input.rows+50);
    //inputQuad[3] = Point2f( -50,input.rows+50  );  
    
    inputQuad[0] = Point2f(547,455);
    inputQuad[1] = Point2f(776,455);
    inputQuad[2] = Point2f(1276,568);
    inputQuad[3] = Point2f(177, 568);  
    // The 4 points where the mapping is to be done , from top-left in clockwise order
    outputQuad[0] = Point2f( 0,0 );
    outputQuad[1] = Point2f( input.cols-1,0);
    outputQuad[2] = Point2f( input.cols-1,input.rows-1);
    outputQuad[3] = Point2f( 0,input.rows-1  );
 
    // Get the Perspective Transform Matrix i.e. lambda 
    lambda = getPerspectiveTransform( inputQuad, outputQuad );
    // Apply the Perspective Transform just found to the src image
    warpPerspective(input,output,lambda,output.size() );
 
    //Display input and output
    imshow("Input",input);
    imshow("Output",output);
 
    waitKey(0);
    return 0;
}
