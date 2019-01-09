
#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>



using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
    cout<<"start loading video"<<endl;


    VideoCapture cap;

    cap = VideoCapture("../../data/video/test.mp4");

    cout<<"video loaded"<<endl;

    Mat curr_frame_color;
    Mat curr_frame;
    Mat feature_frame;

    int MAX_FEATURES = 500;

    Ptr<Feature2D> orb = ORB::create(MAX_FEATURES);
    vector<KeyPoint> keypoints;
    Mat descriptors;

    while(cap.isOpened())
    {
        cap>>curr_frame_color;

        cvtColor(curr_frame_color,curr_frame,CV_RGB2GRAY);


        orb->detectAndCompute(curr_frame,Mat(),keypoints,descriptors);

        cout<<"keypoints.size() = "<<keypoints.size()<<endl;


        //draw keypoints
        drawKeypoints(curr_frame,keypoints,feature_frame);
        //cout<<"curr_frame.cols = "<<curr_frame.cols<<endl;
        








        //imshows
        imshow("curr_frame_color",curr_frame_color);
        imshow("feature_frame",feature_frame);
        waitKey(1);
    }


    cout<<"asdf"<<endl;

    return 0;
}
