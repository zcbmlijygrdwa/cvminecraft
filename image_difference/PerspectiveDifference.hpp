#ifndef PerspectiveDifference_h
#define PerspectiveDifference_h

#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

class PerspectiveDifference
{
    public:
        //Input and Output Image;
        Mat input1, input2,diff;

        Mat proj1, proj2;

        // Input Quadilateral or Image plane coordinates
        Point2f inputQuad1[4];
        Point2f inputQuad2[4];
        // Output Quadilateral or World plane coordinates
        Point2f outputQuad[4];



        PerspectiveDifference()
        {

        }


        PerspectiveDifference(vector<Point2f> points1, vector<Point2f> points2)
        {
            inputQuad1[0] = points1[0]; 
            inputQuad1[1] = points1[1]; 
            inputQuad1[2] = points1[2]; 
            inputQuad1[3] = points1[3]; 

            inputQuad2[0] = points2[0]; 
            inputQuad2[1] = points2[1]; 
            inputQuad2[2] = points2[2]; 
            inputQuad2[3] = points2[3]; 
        }

        void setQuadilaterals(vector<Point2f> points1, vector<Point2f> points2)
        {
            inputQuad1[0] = points1[0];
            inputQuad1[1] = points1[1];
            inputQuad1[2] = points1[2];
            inputQuad1[3] = points1[3];

            inputQuad2[0] = points2[0];
            inputQuad2[1] = points2[1];
            inputQuad2[2] = points2[2];
            inputQuad2[3] = points2[3];
        }


        void setInputs(Mat& img1, Mat& img2)
        {
            input1 = img1;
            input2 = img2;

            // The 4 points where the mapping is to be done , from top-left in clockwise order
            outputQuad[0] = Point2f( 0,0 );
            outputQuad[1] = Point2f( input1.cols-1,0);
            outputQuad[2] = Point2f( input1.cols-1,input1.rows-1);
            outputQuad[3] = Point2f( 0,input1.rows-1  );


        }

        void compute()
        {
            // Lambda Matrix
            Mat lambda(2,4,CV_32FC1 );

            // Get the Perspective Transform Matrix i.e. lambda 
            lambda = getPerspectiveTransform( inputQuad1, outputQuad );
            // Apply the Perspective Transform just found to the src image
            warpPerspective(input1,proj1,lambda,proj1.size() );

            // Get the Perspective Transform Matrix i.e. lambda 
            lambda = getPerspectiveTransform( inputQuad2, outputQuad );
            // Apply the Perspective Transform just found to the src image
            warpPerspective(input2,proj2,lambda,proj2.size() );

            diff = proj1-proj2;
        }

        void getDiff(Mat* m)
        {
            *m = diff;
        }

        void getProj1(Mat* m)
        {
            *m = proj1;
        }

        void getProj2(Mat* m)
        {
            *m = proj2;
        }
};


#endif
