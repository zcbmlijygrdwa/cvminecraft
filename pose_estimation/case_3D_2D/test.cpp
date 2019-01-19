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

//struct CostFunctor
//{
//
//    CostFunctor(Point3d& point_3d, Point2d& point_2d, Mat& cam_intrinsics)
//    {
//        p_3d = point_3d;
//        p_2d = point_2d;
//
//        fx = cam_intrinsics.at<double>(0,0);
//        fy = cam_intrinsics.at<double>(1,1);
//        cx = cam_intrinsics.at<double>(0,2);
//        cy = cam_intrinsics.at<double>(1,2);
//    }
//
//
//    template<typename T>
//        bool operator()(const T* const cere_r, const T* const cere_t, T* residual) const
//        {
//            T p_origin[3];
//            T p_rotate[3];
//            T p_rotate_translate[3];
//
//            p_origin[0] = T(p_3d.x);
//            p_origin[1] = T(p_3d.y);
//            p_origin[2] = T(p_3d.z);
//
//
//            // cere_r[0,1,2] are the angle-axis rotation.
//            //ceres::AngleAxisRotatePoint(cere_r,p_origin,p_rotate);
//
//            //using rotation matrix for rotating the 3D point
//
//
//            T roll = cere_r[0];
//            T pitch = cere_r[1];
//            T yaw = cere_r[2];
//
//            Matrix<T,3,3> Rx;
//            Rx<<T(1),T(0),T(0),
//                T(0),cos(roll),-sin(roll),
//                T(0),sin(roll),cos(roll);
//
//            Matrix<T,3,3> Ry;
//            Ry<<cos(pitch),T(0),sin(pitch),
//                T(0),T(1),T(0),
//                -sin(pitch),T(0),cos(pitch);
//
//
//            Matrix<T,3,3> Rz;
//            Rz<<cos(yaw),-sin(yaw),T(0),
//                sin(yaw),cos(yaw),T(0),
//                T(0),T(0),T(1);
//            
//            Matrix<T,3,3> rot = Rx*Ry*Rz;
//
//            Matrix<T,3,1> p_o;
//            p_o<<p_origin[0],
//            p_origin[1],
//            p_origin[2];
//
//            p_o = rot*p_o;
//            p_rotate[0] = p_o(0);
//            p_rotate[1] = p_o(1);
//            p_rotate[2] = p_o(2);
//
//            cout<<"******"<<endl;
//            cout<<"cere_r = "<<cere_r[0]<<", "<<cere_r[1]<<", "<<cere_r[2]<<endl;
//            cout<<"p_origin = "<<p_origin[0]<<", "<<p_origin[1]<<", "<<p_origin[2]<<endl;
//            cout<<"p_rotate = "<<p_rotate[0]<<", "<<p_rotate[1]<<", "<<p_rotate[2]<<endl;
//
//            p_rotate_translate[0] = p_rotate[0] + cere_t[0];
//            p_rotate_translate[1] = p_rotate[1] + cere_t[1];
//            p_rotate_translate[2] = p_rotate[2] + cere_t[2];
//
//
//            //project to 2D
//            const T x = p_rotate_translate[0]/p_rotate_translate[2];
//            const T y = p_rotate_translate[1]/p_rotate_translate[2];
//
//            //assume fx = 520.9, fy = 521.0
//            //assume cx = 325.1, cy = 249.7
//
//            const T u = x*(T)fx+(T)cx;
//            const T v = y*(T)fy+(T)cy;
//
//
//            //evaluate residual
//            residual[0] = u-(T)p_2d.x;
//            residual[1] = v-(T)p_2d.y;
//            return true; 
//        }
//
//    Point3d p_3d;
//    Point2d p_2d;
//    double fx,fy,cx,cy;
//};
//
//
//class CeresHelper
//{
//    public:
//        vector<Point3d> p_3d_set;
//        vector<Point2d> p_2d_set;
//
//        ceres::Problem problem;
//        ceres::Solver::Options option;
//
//        double cere_r[3];
//        double cere_t[3];
//
//
//        void init()
//        {
//
//
//            //take initial guess of cere_r, cere_t
//
//            cere_r[0] = 0.1;
//            cere_r[1] = 0.2;
//            cere_r[2] = 0.3;
//
//            cere_t[0] = 0.1;
//            cere_t[1] = 0.2;
//            cere_t[2] = 0.3;
//
//            option.max_num_iterations = 1500; //default = 50;
//
//        }
//
//        void setInputs(vector<Point3d>& i1, vector<Point2d>& i2,Mat& cam_i)
//        {
//            p_3d_set = i1;
//            p_2d_set = i2;
//
//            if(i1.size()!=i2.size())
//            {
//                cout<<"Two input have different size, wrong inputs."<<endl;
//                return;
//            }
//
//            for(uint i  = 0 ; i < p_3d_set.size();i++)
//            {
//
//                //http://ceres-solver.org/nnls_modeling.html#autodiffcostfunction
//                //<CostFunctor,residual_dimension, dimension_1st_argument_in_operator(), dimension_2nd_argument_in_operator()>
//                ceres::CostFunction* costfunction = new ceres::AutoDiffCostFunction<CostFunctor,2,3,3>(new CostFunctor(p_3d_set[i],p_2d_set[i],cam_i));
//                problem.AddResidualBlock(costfunction,NULL,cere_r,cere_t);
//            }
//        }
//
//        void optimize()
//        {
//            option.linear_solver_type=ceres::DENSE_SCHUR;
//            //输出迭代信息到屏幕
//            option.minimizer_progress_to_stdout=true;
//            //显示优化信息
//            ceres::Solver::Summary summary;
//            //开始求解
//            ceres::Solve(option,&problem,&summary);
//            //显示优化信息
//            cout<<summary.FullReport()<<endl;
//
//            cout<<"Ceres output: cere_r = ["<<cere_r[0]<<", "<<cere_r[1]<<", "<<cere_r[2]<<"]"<<endl;
//            cout<<"Ceres output: cere_t = ["<<cere_t[0]<<", "<<cere_t[1]<<", "<<cere_t[2]<<"]"<<endl;
//        }
//};


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
