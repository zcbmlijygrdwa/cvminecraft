#include <opencv2/opencv.hpp>
//#include <opencv_lib.hpp>
#include <opencv2/viz.hpp>
#include <iostream>
#include <opencv2/features2d/features2d.hpp>

#include "../opencv_vis/point_set/PointCloudVisualizer.hpp"

using namespace cv;
using namespace std;

int main()
{

    int height = 480, width = 640;

    float fx = 525.0f, // default
          fy = 525.0f,
          cx = 319.5f,
          cy = 239.5f;

    Mat depth1;
    Mat depth2;

    Mat img1, img2;
    img1 = imread("rgb1.png");
    img2 = imread("rgb2.png");

    depth1 = imread("1.png", -1);

    depth2 = imread("2.png", -1);

    int MAX_FEATURES = 500;

    cv::Ptr<cv::Feature2D> orb = cv::ORB::create(MAX_FEATURES);
    vector<cv::KeyPoint> kp1, kp2;
    cv::Mat desp1, desp2;
    orb->detectAndCompute( img1, cv::Mat(), kp1, desp1 );
    orb->detectAndCompute( img2, cv::Mat(), kp2, desp2 );
    //cout<<"分别找到了"<<kp1.size()<<"和"<<kp2.size()<<"个特征点"<<endl;

    if(kp1.size()==0||kp2.size()==0)
    {
        return false;
    }


    drawKeypoints(img1,kp1, img1, Scalar::all(-1),DrawMatchesFlags::DEFAULT);
    drawKeypoints(img2,kp2, img2, Scalar::all(-1),DrawMatchesFlags::DEFAULT);

    cv::Ptr<cv::DescriptorMatcher>  matcher = cv::DescriptorMatcher::create( "BruteForce-Hamming");

    double knn_match_ratio=0.8;
    vector< vector<cv::DMatch> > matches_knn;
    matcher->knnMatch( desp1, desp2, matches_knn, 2 );
    vector< cv::DMatch > matches;

    for ( size_t i=0; i<matches_knn.size(); i++ )
    {
        if (matches_knn[i][0].distance < knn_match_ratio * matches_knn[i][1].distance )
            matches.push_back( matches_knn[i][0] );
    }

    cout<<"matches.size() = "<<matches.size()<<endl;
    
    vector<cv::Point2f> points1,points2;
    
    for ( auto m:matches )
    {
        points1.push_back( kp1[m.queryIdx].pt );
        points2.push_back( kp2[m.trainIdx].pt );
    }

    imshow("depth1", depth1);
    imshow("img1", img1);
    imshow("depth2", depth2);
    imshow("img2", img2);

    waitKey(0);

    depth1.convertTo(depth1, CV_32FC1, 1.f / 5000.f);
    depth1.setTo(std::numeric_limits<float>::quiet_NaN(), depth1 == 0);
    depth2.convertTo(depth2, CV_32FC1, 1.f / 5000.f);
    depth2.setTo(std::numeric_limits<float>::quiet_NaN(), depth2 == 0);


    PointCloudVisualizer pcv;
    for (int y = 0; y < 480; y++){
        for (int x = 0; x < 640; x++){
            if (depth1.at<float>(y, x) < 8.0 && depth1.at<float>(y, x) > 0.4){
                //RGB-D Dataset
                float Z = depth1.at<float>(y, x);
                float X = (x - cx) * Z / fx;
                float Y = (y - cy) * Z / fy;
                //cout<<"X = "<<X<<", Y = "<<Y<<", Z = "<<Z<<endl;
                pcv.addColorPoint(Point3f(X, Y, Z),100,100,255);
            }

            if (depth2.at<float>(y, x) < 8.0 && depth2.at<float>(y, x) > 0.4){
                //RGB-D Dataset
                float Z = depth2.at<float>(y, x);
                float X = (x - cx) * Z / fx;
                float Y = (y - cy) * Z / fy;
                pcv.addColorPoint(Point3f(X, Y, Z),100,255,100);
            }
        }
    }

    pcv.commitPoints();

    pcv.show();

    return 0;
}

