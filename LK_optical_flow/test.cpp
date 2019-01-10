
#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc.hpp>

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
    Mat prev_frame;
    Mat feature_frame;

    int MAX_FEATURES = 500;

    Ptr<Feature2D> orb = ORB::create(MAX_FEATURES);
    vector<KeyPoint> keypoints;
    Mat descriptors;


    //check videocapture
    if(!cap.isOpened())
    {
        cout<<"failed to load video"<<endl;
        return 1;
    }

    //get the first frame and get the keypoitns
    cap>>curr_frame_color;

    cvtColor(curr_frame_color,curr_frame,CV_RGB2GRAY);


    orb->detectAndCompute(curr_frame,Mat(),keypoints,descriptors);

    cout<<"keypoints.size() = "<<keypoints.size()<<endl;


    //draw keypoints
    drawKeypoints(curr_frame,keypoints,feature_frame);


    cout<<"keypoints deteted on the first frame, start to do LK optical flow tracking"<<endl;

    //imshow("feature_frame",feature_frame);
    //waitKey(0); 


    curr_frame.copyTo(prev_frame);

    Mat optical_flow_frame;
    vector<Point2f> optical_flow_curr;
    vector<Point2f> optical_flow_predict;

    for(uint i = 0 ; i < keypoints.size() ; i++)
    {
        optical_flow_curr.push_back(keypoints[i].pt);
        //cout<<"keypoints[i] = "<<keypoints[i].pt<<endl;
    }

    //while(cap.isOpened())
    for(int j = 0 ; j< 10; j++)
    {
        cap>>curr_frame_color;
        cvtColor(curr_frame_color,curr_frame,CV_RGB2GRAY);

        //imshow("curr_frame",curr_frame);
        //imshow("prev_frame",prev_frame);
        //waitKey(1);

        //calculate LK_optical_flow
        vector<uchar> status;
        vector<float> err;
        calcOpticalFlowPyrLK(prev_frame,curr_frame,optical_flow_curr,optical_flow_predict,status,err);




        //draw predict points on curr_frame
        int failed_count = 0;
        for(uint i = 0 ; i < optical_flow_predict.size() ; i++)
        {
            if(status[i]==0)
            {
                failed_count++;
            }
            circle( curr_frame, optical_flow_predict[i], 3, Scalar(0,255,0), -1, 8);
        }

        cout<<"LK optical flow predict: "<<optical_flow_predict.size()<<" points, failed_count = "<<failed_count<<endl;
        imshow("curr_frame with predict",curr_frame);
        waitKey(0); 


    }
    cout<<"asdf"<<endl;

    return 0;
}
