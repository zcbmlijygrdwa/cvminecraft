
//camera matrix
//638.654454 0.000000 324.121847
//0.000000 640.380115 239.403421
//0.000000 0.000000 1.000000
//
//distortion
//-0.008626 -0.012390 -0.000213 0.002877 0.000000
//
//rectification
//1.000000 0.000000 0.000000
//0.000000 1.000000 0.000000
//0.000000 0.000000 1.000000
//
//projection
//636.658447 0.000000 325.555775 0.000000
//0.000000 639.854248 239.335321 0.000000
//0.000000 0.000000 1.000000 0.000000


#include "opencv2/opencv.hpp"
using namespace cv;
using namespace std;



void create_intrinsic(Mat& mat, double fx, double fy, double cx, double cy)
{
    mat.at<double>(0,0) = fx;
    mat.at<double>(1,1) = fy;

    mat.at<double>(0,2) = cx;
    mat.at<double>(1,2) = cy;

    mat.at<double>(2,2) = 1;
}



int main(int, char**)
{
    VideoCapture cap(0); // open the default camera
    //VideoCapture cap(1); // open the second camera
    if(!cap.isOpened())  // check if we succeeded
        return -1;
    Mat output;
    namedWindow("output",1);

    Mat camera_intrisic_matrix = Mat::zeros(Size(3, 3), CV_64F);
    create_intrinsic(camera_intrisic_matrix,12,13,14,15);
    cout<<"camera_intrisic_matrix = "<<endl<<camera_intrisic_matrix<<endl;


    vector<Point2d> vec_points_2d;
    vector<Point3d> vec_points_3d;

    Mat rvec;
    Mat tvec;
    vector<double> distCoeffs;

    Size patternSize = Size(8,6);

    for(int i = 0;i<8;i++)
    {
        for(int j = 0 ; j < 6 ; j++)
        {
            vec_points_3d.push_back(Point3d(0.01*i,0.01*j,0));
        }
    }


    while(true)
    {
        Mat frame;
        cap >> frame; // get a new frame from camera
        cvtColor(frame, frame, COLOR_BGR2GRAY);

        output = frame;

        if(!findChessboardCorners(frame,patternSize, vec_points_2d, CALIB_CB_FAST_CHECK)) continue;


        cout<<"vec_points_2d.size() = "<<vec_points_2d.size()<<endl;
        //for(Point2d p : vec_points_2d)
        //{
        //    cout<<"p = "<<p<<endl;
        //}


        //drawChessboardCorners(output, patternSize, vec_points_2d, true);
        //start to do solvePnp
        solvePnP(vec_points_3d,vec_points_2d,camera_intrisic_matrix,distCoeffs,rvec,tvec,false,CV_ITERATIVE);

        cout<<"//*************** Result  ******************"<<endl;

        cout<<"CV_ITERATIVE"<<endl;
        cout<<"rvec = "<<rvec<<endl;
        cout<<"tvec = "<<tvec<<endl;

        imshow("output", output);
        if(waitKey(30) >= 0) break;

    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}
