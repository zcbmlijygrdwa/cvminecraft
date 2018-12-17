#include "opencv2/features2d/features2d.hpp"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


using namespace std;
using namespace cv;

int main()
{
    Mat img1, img2;

    img1 = imread("/home/zhenyu/cvminecraft/orb/data/1.png",CV_LOAD_IMAGE_GRAYSCALE);
    img2 = imread("/home/zhenyu/cvminecraft/orb/data/2.png",CV_LOAD_IMAGE_GRAYSCALE);

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

    waitKey(0);   

    return 0;

}
