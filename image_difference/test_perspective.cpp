
// https://docs.opencv.org/3.4/da/d97/tutorial_threshold_inRange.html

#include <opencv2/opencv.hpp>
#include "PerspectiveDifference.hpp"
using namespace cv;

int main(int argc, char** argv)
{



    const char* default_file1 = "../data/ground.png";
    const char* filename1 = argc >=3 ? argv[1] : default_file1;

    const char* default_file2 = "../data/with_parts.png";
    const char* filename2 = argc >=3 ? argv[2] : default_file2;

    //Input and Output Image;
    Mat input1, input2,diff;

    Mat proj1, proj2;



    //Load the image
    input1 = imread(filename1,1);
    input2 = imread(filename2,1);

    vector<Point2f> points1,points2;

    points1.push_back(Point2f( 104,116 ));
    points1.push_back(Point2f( 567,121));
    points1.push_back(Point2f( 571,459));
    points1.push_back(Point2f( 91,451));


    points2.push_back(Point2f( 84,420 ));
    points2.push_back(Point2f( 183,51));
    points2.push_back(Point2f( 464,71));
    points2.push_back(Point2f( 556,438));

    PerspectiveDifference pd(points1,points2); 

    pd.setInputs(input1,input2);
    pd.compute();
    pd.getDiff(&diff);
    pd.getProj1(&proj1);
    pd.getProj2(&proj2);

    //// Input Quadilateral or Image plane coordinates
    //Point2f inputQuad[4]; 
    //// Output Quadilateral or World plane coordinates
    //Point2f outputQuad[4];
    //// Lambda Matrix
    //Mat lambda( 2, 4, CV_32FC1 );


    //// The 4 points that select quadilateral on the input , from top-left in clockwise order
    //// These four pts are the sides of the rect box used as input 
    //inputQuad[0] = Point2f( 104,116 );
    //inputQuad[1] = Point2f( 567,121);
    //inputQuad[2] = Point2f( 571,459);
    //inputQuad[3] = Point2f( 91,451);  
    //// The 4 points where the mapping is to be done , from top-left in clockwise order
    //outputQuad[0] = Point2f( 0,0 );
    //outputQuad[1] = Point2f( input1.cols-1,0);
    //outputQuad[2] = Point2f( input1.cols-1,input1.rows-1);
    //outputQuad[3] = Point2f( 0,input1.rows-1  );
    //// Get the Perspective Transform Matrix i.e. lambda 
    //lambda = getPerspectiveTransform( inputQuad, outputQuad );
    //// Apply the Perspective Transform just found to the src image
    //warpPerspective(input1,proj1,lambda,proj1.size() );



    //// The 4 points that select quadilateral on the input , from top-left in clockwise order
    //// These four pts are the sides of the rect box used as input 
    //inputQuad[0] = Point2f( 84,420 );
    //inputQuad[1] = Point2f( 183,51);
    //inputQuad[2] = Point2f( 464,71);
    //inputQuad[3] = Point2f( 556,438);  
    //// The 4 points where the mapping is to be done , from top-left in clockwise order
    //outputQuad[0] = Point2f( 0,0 );
    //outputQuad[1] = Point2f( input2.cols-1,0);
    //outputQuad[2] = Point2f( input2.cols-1,input2.rows-1);
    //outputQuad[3] = Point2f( 0,input2.rows-1  );
    //// Get the Perspective Transform Matrix i.e. lambda 
    //lambda = getPerspectiveTransform( inputQuad, outputQuad );
    //// Apply the Perspective Transform just found to the src image
    //warpPerspective(input2,proj2,lambda,proj2.size() );

    //diff = proj1-proj2;

    //Display input and output
    imshow("input1",input1);
    imshow("input2",input2);
    imshow("proj1",proj1);
    imshow("proj2",proj2);
    imshow("diff",diff);
    //imshow("Output",output);



    //imwrite("output.jpg",output);

    waitKey(0);
    return 0;
}
