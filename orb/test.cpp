#include "opencv2/features2d/features2d.hpp"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


using namespace std;
using namespace cv;

int main()
{
    Mat img1, img2;

    img1 = imread("../data/1.png",CV_LOAD_IMAGE_GRAYSCALE);
    img2 = imread("../data/2.png",CV_LOAD_IMAGE_GRAYSCALE);

    if(!img1.data)
    {
        cout<<"Reading image 1 failed"<<endl;
    }


    if(!img2.data)
    {
        cout<<"Reading image 2 failed"<<endl;
    }

    imshow("img1",img1);
    imshow("img2",img2);

    //detect features
    const int MAX_FEATURES = 500;

    Ptr<Feature2D> orb1 = ORB::create(MAX_FEATURES);

    std::vector< KeyPoint > keypoints1,keypoints2;
    Mat descriptors1;
    Mat descriptors2;

    orb1->detectAndCompute(img1,Mat(),keypoints1,descriptors1);
    orb1->detectAndCompute(img1,Mat(),keypoints2,descriptors1);

    //draw feature points in img1
    Mat img1_with_features1;
    Mat img1_with_features2;
    drawKeypoints(img1,keypoints1, img1_with_features1, Scalar::all(-1),DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    drawKeypoints(img2,keypoints2, img1_with_features2, Scalar::all(-1),DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    
    cout<<"keypoints1.size() = "<<keypoints1.size()<<endl;
    cout<<"keypoints2.size() = "<<keypoints2.size()<<endl;

    imshow("img1_with_features1",img1_with_features1);
    imshow("img1_with_features2",img1_with_features2);

    //cout<<"orb1 ptr= "<<orb1<<endl;
    waitKey(0);   

    return 0;

}
