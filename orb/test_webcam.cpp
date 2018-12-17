#include "opencv2/features2d/features2d.hpp"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
    char* wrongFormat = "Usage: test_webcam cameraIndex";

    if(argc==2)
    {
        int cameraIndex = 0;


        cameraIndex = atoi(argv[1]);

        for(int i = 0 ;i<strlen(argv[1]);i++)
        {
            if(!isdigit(argv[1][i]))
            {
                cout<<wrongFormat<<endl;
                return -1;
            }
        }

        cout<<"Selected camera index: "<<cameraIndex<<endl;
        Mat img1;
        const int MAX_FEATURES = 500;
        Ptr<Feature2D> orb1 = ORB::create(MAX_FEATURES);

        VideoCapture cap(cameraIndex);
        if(!cap.isOpened())  // check if we succeeded
            return -1;

        while(cap.isOpened())
        {
            cap >> img1;
            if(!img1.data)
            {
                cout<<"Reading frame failed"<<endl;
                break;
            }



            //detect features

            std::vector< KeyPoint > keypoints1,keypoints2;
            Mat descriptors1;

            orb1->detectAndCompute(img1,Mat(),keypoints1,descriptors1);

            //draw feature points in img1
            Mat img1_with_features1;
            drawKeypoints(img1,keypoints1, img1_with_features1, Scalar::all(-1),DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
            cout<<"keypoints1.size() = "<<keypoints1.size()<<endl;

            imshow("img1_with_features",img1_with_features1);
            if(waitKey(30) >= 0) break;
        }
    }
    else
    {
        cout<<wrongFormat<<endl;
        return -1;
    }
    return 0;

}
