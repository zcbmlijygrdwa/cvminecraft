#include <iostream>

//#include <ceres/ceres.h>
//#include <ceres/rotation.h>

#include <Eigen/Eigen>
#include <Eigen/Geometry>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include "SolvePnpCeres.hpp"

using namespace cv;
using namespace std;
using namespace Eigen;


Mat my_transform(Mat points_3d)
{
    //double PI = 3.141592653;
    Quaterniond quat_rot;
    quat_rot = AngleAxisd(0.5,Vector3d::UnitX())
        *AngleAxisd(0.1,Vector3d::UnitY())
        *AngleAxisd(1.66,Vector3d::UnitZ());

    //cout<<"quat_rot.matrix() = "<<quat_rot.matrix()<<endl;

    Vector3d translation(1,2,3);

    //cout<<"translation = "<<translation<<endl;

    Isometry3d trans = Isometry3d::Identity();

    trans.rotate(quat_rot);
    trans.pretranslate(translation);

    //cout<<"trans.matrix() = "<<endl<<trans.matrix()<<endl;


    MatrixXd original_points;

    cv2eigen(points_3d,original_points);

    //cout<<"original_points = "<<original_points<<endl;

    int cols = points_3d.cols;

    MatrixXd original_points_with_ones(4,cols);
    original_points_with_ones.block(0,0,3,cols) = original_points;
    original_points_with_ones.block(3,0,1,cols) = MatrixXd::Ones(1,cols);

    //cout<<"original_points_with_ones = "<<endl<<original_points_with_ones<<endl;

    //transform.block<3,3>(0,0) = quat_rot.matrix();
    //transform.block<3,1>(0,3) = translation;

    //cout<<"trans.matrix() = "<<trans.matrix()<<endl;

    Vector3d test;
    test<<1,1,1;
    test = trans*test;

    original_points_with_ones = trans.matrix()*original_points_with_ones;
    //cout<<"after trans, original_points_with_ones= "<<endl<<original_points_with_ones<<endl;

    //cout<<"transform = "<<transform<<endl;
    MatrixXd transformed_points;

    transformed_points = original_points_with_ones.block(0,0,3,cols);
    //cout<<"transformed_points = "<<endl<<transformed_points<<endl;


    Mat output;
    eigen2cv(transformed_points,output);


    //make sure the format is right
    output.convertTo(output, CV_64F);

    //cout<<"output = "<<endl<<output<<endl;

    return output;
}

void create_intrinsic(Mat& mat, double fx, double fy, double cx, double cy)
{
    mat.at<double>(0,0) = fx;
    mat.at<double>(1,1) = fy;

    mat.at<double>(0,2) = cx;
    mat.at<double>(1,2) = cy;

    mat.at<double>(2,2) = 1;
}



void prepare_3d_points(Mat& mat)
{
    int i = 0;
    mat.at<double>(0,i) = 12;
    mat.at<double>(1,i) = 12;
    mat.at<double>(2,i) = 15;

    i = 1;
    mat.at<double>(0,i) = 30;
    mat.at<double>(1,i) = 10;
    mat.at<double>(2,i) = 12;

    i = 2;
    mat.at<double>(0,i) = 33;
    mat.at<double>(1,i) = 33;
    mat.at<double>(2,i) = 33;

    i = 3;
    mat.at<double>(0,i) = 34;
    mat.at<double>(1,i) = 45;
    mat.at<double>(2,i) = 56;

    i = 4;
    mat.at<double>(0,i) = 10;
    mat.at<double>(1,i) = 20;
    mat.at<double>(2,i) = 42;

    i = 5;
    mat.at<double>(0,i) = 40;
    mat.at<double>(1,i) = 6;
    mat.at<double>(2,i) = 12;

    i = 6;
    mat.at<double>(0,i) = 16;
    mat.at<double>(1,i) = 10;
    mat.at<double>(2,i) = 7;

    i = 7;
    mat.at<double>(0,i) = 20;
    mat.at<double>(1,i) = 20;
    mat.at<double>(2,i) = 12;

    i = 8;
    mat.at<double>(0,i) = 21;
    mat.at<double>(1,i) = 23;
    mat.at<double>(2,i) = 14;


    i = 9;
    mat.at<double>(0,i) = 43;
    mat.at<double>(1,i) = 16;
    mat.at<double>(2,i) = 32;

}

vector<Point2d> matToVec2(Mat mat)
{
    vector<Point2d> output;
    for(int i = 0 ; i < mat.cols ; i++)
    {
        output.push_back(Point2d(mat.at<double>(0,i),mat.at<double>(1,i)));
    }
    return output;
}

vector<Point3d> matToVec3(Mat mat)
{
    vector<Point3d> output;
    for(int i = 0 ; i < mat.cols ; i++)
    {
        output.push_back(Point3d(mat.at<double>(0,i),mat.at<double>(1,i),mat.at<double>(2,i)));
    }
    return output;
}


int main(int argc, char** argv)
{

    cout<<"This is the 3D-2D post estimation test."<<endl;


    Mat camera_intrisic_matrix = Mat::zeros(Size(3, 3), CV_64F);
    create_intrinsic(camera_intrisic_matrix,12,13,14,15);
    cout<<"camera_intrisic_matrix = "<<endl<<camera_intrisic_matrix<<endl;


    Mat points_3d = Mat::zeros(Size(10,3),CV_64F);
    prepare_3d_points(points_3d);

    //cout<<"points_3d = "<<endl<<points_3d<<endl;

    Mat points_3d_transformed = my_transform(points_3d);



    vector<Point3d> vec_points_3d = matToVec3(points_3d);

    for(uint i = 0 ; i < vec_points_3d.size() ; i++)
    {
        cout<<"vec_points_3d["<<i<<"] = "<<vec_points_3d[i]<<endl;
    }

    cout<<"-------------------------"<<endl;
    vector<Point3d> vec_points_3d_transformed = matToVec3(points_3d_transformed);
    for(uint i = 0 ; i < vec_points_3d.size() ; i++)
    {
        cout<<"vec_points_3d_transformed["<<i<<"] = "<<vec_points_3d_transformed[i]<<endl;
    }


    Mat points_3d_proj = Mat::zeros(Size(10,3),CV_64F);
    points_3d_proj = camera_intrisic_matrix*points_3d_transformed;

    //normalize
    for(int i = 0 ; i < points_3d_proj.cols ; i++)
    {
        points_3d_proj.at<double>(0,i) /= points_3d_proj.at<double>(2,i);
        points_3d_proj.at<double>(1,i) /= points_3d_proj.at<double>(2,i);
        points_3d_proj.at<double>(2,i) /= points_3d_proj.at<double>(2,i);
    }

    cout<<"-------------------------"<<endl;
    //cout<<"after normalization, points_3d_proj = "<<endl<<points_3d_proj<<endl;
    vector<Point2d> vec_points_2d = matToVec2(points_3d_proj);
    for(uint i = 0 ; i < vec_points_2d.size() ; i++)
    {
        cout<<"vec_points_2d["<<i<<"] = "<<vec_points_2d[i]<<endl;
    }


    cout<<"-------------------------"<<endl;

    //start to do solvePnp
    Mat rvec;
    Mat tvec;
    vector<double> distCoeffs;
    solvePnP(vec_points_3d,vec_points_2d,camera_intrisic_matrix,distCoeffs,rvec,tvec,false,CV_ITERATIVE);

    cout<<"//*************** Result  ******************"<<endl;

    cout<<"CV_ITERATIVE"<<endl;
    cout<<"rvec = "<<rvec<<endl;
    cout<<"tvec = "<<tvec<<endl;

    //solvePnP(vec_points_3d,vec_points_2d,camera_intrisic_matrix,distCoeffs,rvec,tvec,false,CV_P3P);
    //cout<<"CV_P3P"<<endl;
    //cout<<"rvec = "<<rvec<<endl;
    //cout<<"tvec = "<<tvec<<endl;

    solvePnP(vec_points_3d,vec_points_2d,camera_intrisic_matrix,distCoeffs,rvec,tvec,false,CV_EPNP);
    cout<<"//*****************************************"<<endl;
    cout<<"CV_EPNP"<<endl;
    cout<<"rvec = "<<rvec<<endl;
    cout<<"tvec = "<<tvec<<endl;

    SolvePnpCeres solvepnpceres;
    solvepnpceres.init();
    solvepnpceres.setInputs(vec_points_3d,vec_points_2d,camera_intrisic_matrix);
    solvepnpceres.optimize(&rvec,&tvec,false);
    cout<<"//*****************************************"<<endl;
    cout<<"Ceres Least Square"<<endl;
    cout<<"rvec = "<<rvec<<endl;
    cout<<"tvec = "<<tvec<<endl;



    return 0;
}
