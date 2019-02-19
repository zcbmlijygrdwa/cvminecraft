#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include<ceres/ceres.h>



using namespace std;
using namespace Eigen;
using namespace cv;


//第一部分：构建代价函数，重载（）符号，仿函数的小技巧
struct CostFunctor {

    CostFunctor(Point2d& point, Matrix3d& K)
    {
        p_observed_ptr = &point;
        this->K = K;
    }

    Point2d* p_observed_ptr;
    Matrix3d K;


    //each iteration needs a new optimizables: camera matrix and a new point position
    bool operator()(const double* const p_pos, const double* const cam_pos, double* residual) const {

        cout<<"------------"<<endl;

        // cam_pos[0,1,2] are the angle-axis rotation.
        // cam_pos[3,4,5] are the translations.

        Eigen::Matrix3d rot;

        rot = AngleAxisd(cam_pos[0],Vector3d::UnitX())
            *AngleAxisd(cam_pos[1],Vector3d::UnitY())
            *AngleAxisd(cam_pos[2],Vector3d::UnitZ());

        //crate a new transform matrix (Isometry)
        Isometry3d T1 = Isometry3d::Identity();

        T1.rotate(rot);
        //T1.rotate(Quaterniond(0.35,0.2,0.3,0.1).normalized());
        T1.pretranslate(Vector3d(cam_pos[3],cam_pos[4],cam_pos[5]));
        cout<<"T1 = \n"<<T1.matrix()<<endl;

        T1 = T1.inverse();

        Vector3d point(p_pos[0],p_pos[1],p_pos[2]);
        cout<<"point = \n"<<point<<endl;
        cout<<"T1 inverse = \n"<<T1.matrix()<<endl;

        //projection
        Vector3d p_predicted = K*T1*point;
        cout<<"projection"<<endl;


        //normalization
        p_predicted = p_predicted/p_predicted(2);
        cout<<"p_predicted = \n"<<p_predicted<<endl;
        cout<<"p_observed_ptr = "<<p_observed_ptr->x<<", "<<p_observed_ptr->y<<endl;

        residual[0] = p_predicted(0) - p_observed_ptr->x;
        residual[1] = p_predicted(1) - p_observed_ptr->y;
        return true;
    }
};


void pinHoleProj(Matrix3d K, Isometry3d T,vector<Eigen::Vector3d> points_3d,  vector<Eigen::Vector2d> &points_2d1)
{
    //T: transformation from camera coordinate system to world coordinate system
    //T: 3 X 4


    //isometry matrix: internally 4X4 but used as 3X3, homogeneous - nonhomogeneous conversion done internally

    Vector3d p(3);
    int points_size = points_3d.size();
    for(int i = 0; i<points_size;i++)
    {
        Vector3d P = points_3d[i];
        cout<<"P = "<<P<<endl;

        //projection
        p = K*T*P;

        cout<<"p = "<<p<<endl;

        //normalization
        p = p/p(2);

        cout<<"p2 = "<<p<<endl;

        Vector2d pp(2);
        pp<<p(0),p(1);
        points_2d1.push_back(pp);
    }
}

Matrix3d projMat(double fx,double fy,double cx,double cy)
{

    Matrix3d K = Matrix3d::Zero();

    K(0,0) = fx;
    K(1,1) = fy;
    K(0,2) = cx;
    K(1,2) = cy;
    K(2,2) = 1;

    return K;
}

int main()
{


    double fx = 250;
    double fy = 250;
    double cx = 100;
    double cy = 100;


    Matrix3d K = projMat(fx,fy,cx,cy);
    
    cout<<"K = "<<endl<<K<<endl;

    Vector3d P(3);
    Vector3d p(3);
    vector<Eigen::Vector3d> points_3d;
    vector<Eigen::Vector2d> points_2d1;
    vector<Eigen::Vector2d> points_2d2;

    P<<1,1,12;
    points_3d.push_back(P);

    P<<3,7,3;
    points_3d.push_back(P);

    P<<6,14,5;
    points_3d.push_back(P);

    P<<1,5,7;
    points_3d.push_back(P);

    P<<1,2.5,12;
    points_3d.push_back(P);

    P<<5,7,8;
    points_3d.push_back(P);

    P<<2,4,4;
    points_3d.push_back(P);

    P<<3,9,2;
    points_3d.push_back(P);

    P<<7,2,13;
    points_3d.push_back(P);

    P<<-7,3,6;
    points_3d.push_back(P);

    Isometry3d T = Isometry3d::Identity();

    Matrix3d rot_mat;
    rot_mat = AngleAxisd(0.0,Vector3d::UnitX())
        * AngleAxisd(0.0,Vector3d::UnitY())
        * AngleAxisd(0.0,Vector3d::UnitZ());
    //cout<<"rot_mat created = "<<rot_mat<<endl;

    T.rotate(rot_mat);


    Vector3d trans_mat;
    trans_mat<<0,0,0;
    //cout<<"trans_mat created = "<<trans_mat<<endl;

    T.pretranslate(trans_mat);

    cout<<"T = "<<T.matrix()<<endl;
    //take inverse because need to transfer cam back and transfer points away
    pinHoleProj(K,T.inverse(),points_3d,points_2d1);
for(int i = 0 ; i < points_2d1.size() ; i++)
    cout<<"points_2d1["<<i<<"] = "<<points_2d1[i]<<endl;


    //for the 2nd transform

    cout<<endl<<endl<<endl<<"2Nd transform!"<<endl;
    T = Isometry3d::Identity();

    rot_mat = AngleAxisd(0.1,Vector3d::UnitX())
        * AngleAxisd(0.2,Vector3d::UnitY())
        * AngleAxisd(0.3,Vector3d::UnitZ());

    T.rotate(rot_mat);


    trans_mat<<1,2,3;

    T.pretranslate(trans_mat);

    cout<<"T = "<<T.matrix()<<endl;
    cout<<"inv(T) = "<<T.inverse().matrix()<<endl;
    //take inverse because need to transfer cam back and transfer points away
    pinHoleProj(K,T.inverse(),points_3d,points_2d2);


    cout<<"start to find essential matrix"<<endl;

    //convert eigen point set to opencv point set type
    vector<Point2d> points1;
    vector<Point2d> points2;

    for(auto pp:points_2d1)
    {
        Point2d tempP = Point2d(pp(0),pp(1));
        points1.push_back(tempP);

        cout<<"pushng: "<<pp(0)<<", "<<pp(1)<<endl;

    }

    for(auto pp:points_2d2)
    {
        Point2d tempP = Point2d(pp(0),pp(1));
        points2.push_back(tempP);
    }


    double* cam_pos = (double*)calloc(6,sizeof(double));

    cam_pos[0] = 0.1;
    cam_pos[1] = 0.2;
    cam_pos[2] = 0.3;
    cam_pos[3] = 1;
    cam_pos[4] = 2;
    cam_pos[5] = 3;


    double** p_pos_set = (double**)calloc(points1.size(),sizeof(double*));

    ceres::Problem problem;
    for (int i = 0; i < points_2d2.size(); i++) {

        p_pos_set[i] = (double*)calloc(3,sizeof(double));
        //p_pos_set[i][0] = points1[i].x;
        //p_pos_set[i][1] = points1[i].y;
        //p_pos_set[i][2] = 1;    //assuming on the z = 1 plane

        // 由于深度不知道，只能把深度设置为1了
    

        double z = 1;
        p_pos_set[i][0] = ( points1[i].x - cx ) * z / fx;
        p_pos_set[i][1] = ( points1[i].y - cy ) * z / fy;
        p_pos_set[i][2] = z;

        //p_pos_set[i][0] = points_3d[i](0);
        //p_pos_set[i][1] = points_3d[i](1);
        //p_pos_set[i][2] = points_3d[i](2);
        cout<<"points1["<<i<<"] = "<<points1[i]<<endl;
        cout<<"p_pos_set[i] = "<<p_pos_set[i][0]<<" ,"<<p_pos_set[i][1]<<" ,"<<p_pos_set[i][2]<<endl;

        ceres::CostFunction* cost_function =
            new ceres::NumericDiffCostFunction<CostFunctor,ceres::CENTRAL,2,3,6>(new CostFunctor(points2[i],K)); //使用自动求导，将之前的代价函数结构体传入，第一个1是输出维度，即残差的维度，第二个1是输入维度，即待寻优参数x的维度。
        problem.AddResidualBlock(cost_function, NULL, p_pos_set[i],cam_pos); //向问题中添加误差项，本问题比较简单，添加一个就行。

    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";

    cout<<"cam_pos = "<<cam_pos[0]<<", "<<cam_pos[1]<<", "<<cam_pos[2]<<", "<<cam_pos[3]<<", "<<cam_pos[4]<<", "<<cam_pos[5]<<endl;

    return -1;

}
