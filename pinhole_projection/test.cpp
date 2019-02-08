
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"


#include <iostream>

using namespace cv;
using namespace std;




Mat eulerAnglesToRotationMatrix(vector<float>& theta)
{
    // Calculate rotation about x axis
    Mat R_x = (Mat_<double>(3,3) <<
               1,       0,              0,
               0,       cos(theta[0]),   -sin(theta[0]),
               0,       sin(theta[0]),   cos(theta[0])
               );
     
    // Calculate rotation about y axis
    Mat R_y = (Mat_<double>(3,3) <<
               cos(theta[1]),    0,      sin(theta[1]),
               0,               1,      0,
               -sin(theta[1]),   0,      cos(theta[1])
               );
     
    // Calculate rotation about z axis
    Mat R_z = (Mat_<double>(3,3) <<
               cos(theta[2]),    -sin(theta[2]),      0,
               sin(theta[2]),    cos(theta[2]),       0,
               0,               0,                  1);
     
     
    // Combined rotation matrix
    Mat R = R_z * R_y * R_x;
     
    return R;
 
}





int main(int argc, char** argv)
{
    // Declare the output variables
    const char* default_file = "../../data/image/lane1.jpg";
    const char* filename = argc >=2 ? argv[1] : default_file;
    // Loads an image
    Mat src = imread( filename, 1);
    // Check if image is loaded fine
    if(src.empty()){
        printf(" Error opening image\n");
        printf(" Program Arguments: [image_name -- default %s] \n", default_file);
        return -1;
    }

    Mat project;


    std::vector<cv::Point3d> objectPoints;
    objectPoints.push_back(Point3d(-0.33,0.0,0.0)); //table left upper
    objectPoints.push_back(Point3d( 0.33,0.0,0.0)); //table right upper
    objectPoints.push_back(Point3d( 0.33,0.6,0.0)); //table right lower
    objectPoints.push_back(Point3d(-0.33,1.0,0.0)); //table left upper
    objectPoints.push_back(Point3d(0.0,0.0,20.0)); //table left upper

    cv::Mat tvec(3,1,cv::DataType<double>::type); // translation vector
    tvec.at<double>(0) =-2;
    tvec.at<double>(1) = 0;
    tvec.at<double>(2) =10;

    vector<float> theta;
    //theta.push_back(-2.35519211553);
    //theta.push_back(0.158035071418);
    //theta.push_back(-1.76534038707);

    theta.push_back(-3.14);
    theta.push_back(0.0);
    theta.push_back(0.0);

    //cv::Mat rvec(3,1,cv::DataType<double>::type); // translation vector
    //rvec.at<double>(0) = 0.0;
    //rvec.at<double>(1) = 0.0;
    //rvec.at<double>(2) = 0.0; 

    //cv::Mat tvec(3,1,cv::DataType<double>::type); // translation vector
    //tvec.at<double>(0) = -0.468743786407;
    //tvec.at<double>(1) = 0.516820722065;
    //tvec.at<double>(2) = 0.654226632285;


    cv::Mat rvec(3,1,cv::DataType<double>::type); // translation vector
    //rvec.at<double>(0) = -2.35519211553;
    //rvec.at<double>(1) = 0.158035071418;
    //rvec.at<double>(2) = -1.76534038707;



    Mat rot_mat = eulerAnglesToRotationMatrix(theta);


    cout<<"0rvec = "<<rvec<<endl;
    Mat r3;
    Rodrigues(rot_mat,rvec);

    cout<<"tvec = "<<tvec<<endl;
    cout<<"rvec = "<<rvec<<endl;

    cv::Mat K(3,3,cv::DataType<double>::type); // intrinsic parameter matrix

    K.at<double>(0,0) = 608.8837890625;
    K.at<double>(0,1) = 0.0;
    K.at<double>(0,2) = 305.5226745605469;
    K.at<double>(1,0) = 0.0;
    K.at<double>(1,1) = 608.6896362304688;
    K.at<double>(1,2) = 225.9498291015625;
    K.at<double>(2,0) = 0.0;
    K.at<double>(2,1) = 0.0;
    K.at<double>(2,2) = 1.0;


    cout<<"K = "<<K<<endl;


    // Create zero distortion
    cv::Mat distCoeffs(4,1,cv::DataType<double>::type);
    distCoeffs.at<double>(0) = 0;
    distCoeffs.at<double>(1) = 0;
    distCoeffs.at<double>(2) = 0;
    distCoeffs.at<double>(3) = 0;

    cout<<"distCoeffs = "<<distCoeffs<<endl;

    std::vector<cv::Point2d> projectedPoints;

    cv::projectPoints(objectPoints, rvec, tvec, K, distCoeffs, projectedPoints);

    for(auto p:projectedPoints)
    {
        cout<<" p = "<<p<<endl;
            Point center(cvRound(p.x), cvRound(p.y));
            int radius = 2;
            circle( src, center, 3, Scalar(89,206,88), -1, 8, 0 );// circle center      
        if(p.x>=0 && p.x<src.cols && p.y>=0 && p.y<src.rows)
        {    
        }
    }





    //// Show results
    imshow("Source", src);
    //// Wait and Exit
    waitKey();
    return 0;
}
