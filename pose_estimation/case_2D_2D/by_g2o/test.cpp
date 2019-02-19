#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d/calib3d.hpp>

// for g2o
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/types/slam3d/se3quat.h>
#include <g2o/types/sba/types_six_dof_expmap.h>




using namespace std;
using namespace Eigen;
using namespace cv;

void g2o_pose(vector<cv::Point2d>& pts1,vector<cv::Point2d>& pts2,Eigen::Isometry3d* pose)
{
    double fx = 250;
    double fy = 250;
    double cx = 100;
    double cy = 100;

    double percentage = 0.0;
    // 构造g2o中的图
    // 先构造求解器
    g2o::SparseOptimizer    optimizer;
    // 使用Cholmod中的线性方程求解器
    g2o::BlockSolver_6_3::LinearSolverType* linearSolver = new  g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType> ();
    // 6*3 的参数
    // 6 X 3 matrix, why 6 X 3?
    g2o::BlockSolver_6_3* block_solver = new g2o::BlockSolver_6_3( std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType>(linearSolver) );
    // L-M 下降 
    // select a iteration strategy
    g2o::OptimizationAlgorithmLevenberg* algorithm = new g2o::OptimizationAlgorithmLevenberg( std::unique_ptr<g2o::BlockSolver_6_3>(block_solver) );

    optimizer.setAlgorithm( algorithm );
    optimizer.setVerbose( false );

    // 添加节点
    // 两个位姿节点
    for ( int i=0; i<2; i++ )
    {
        g2o::VertexSE3Expmap* v = new g2o::VertexSE3Expmap();
        v->setId(i);
        if ( i == 0)
            v->setFixed( true ); // 第一个点固定为零
        // 预设值为单位Pose，因为我们不知道任何信息
        v->setEstimate( g2o::SE3Quat() );
        optimizer.addVertex( v );
    }
    // 很多个特征点的节点
    // 以第一帧为准
    for ( size_t i=0; i<pts1.size(); i++ )
    {
        g2o::VertexSBAPointXYZ* v = new g2o::VertexSBAPointXYZ();
        v->setId( 2 + i );
        // 由于深度不知道，只能把深度设置为1了
        double z = 1;
        double x = ( pts1[i].x - cx ) * z / fx;
        double y = ( pts1[i].y - cy ) * z / fy;
        cout<<"x = "<<x<<endl;
        v->setMarginalized(true);
        v->setEstimate( Eigen::Vector3d(x,y,z) );
        optimizer.addVertex( v );
    }

    // 准备相机参数
    g2o::CameraParameters* camera = new g2o::CameraParameters( fx, Eigen::Vector2d(cx, cy), 0 );
    camera->setId(0);
    optimizer.addParameter( camera );

    // 准备边
    // 第一帧
    vector<g2o::EdgeProjectXYZ2UV*> edges;
    for ( size_t i=0; i<pts1.size(); i++ )
    {
        g2o::EdgeProjectXYZ2UV*  edge = new g2o::EdgeProjectXYZ2UV();
        edge->setVertex( 0, dynamic_cast<g2o::VertexSBAPointXYZ*>   (optimizer.vertex(i+2)) );
        edge->setVertex( 1, dynamic_cast<g2o::VertexSE3Expmap*>     (optimizer.vertex(0)) );
        edge->setMeasurement( Eigen::Vector2d(pts1[i].x, pts1[i].y ) );
        edge->setInformation( Eigen::Matrix2d::Identity() );
        edge->setParameterId(0, 0);
        edge->setLevel(0);
        // 核函数
        edge->setRobustKernel( new g2o::RobustKernelHuber() );
        optimizer.addEdge( edge );
        edges.push_back(edge);
    }
    // 第二帧
    for ( size_t i=0; i<pts2.size(); i++ )
    {
        g2o::EdgeProjectXYZ2UV*  edge = new g2o::EdgeProjectXYZ2UV();
        edge->setVertex( 0, dynamic_cast<g2o::VertexSBAPointXYZ*>   (optimizer.vertex(i+2)) );
        edge->setVertex( 1, dynamic_cast<g2o::VertexSE3Expmap*>     (optimizer.vertex(1)) );
        edge->setMeasurement( Eigen::Vector2d(pts2[i].x, pts2[i].y ) );
        edge->setInformation( Eigen::Matrix2d::Identity() );
        edge->setParameterId(0,0);
        edge->setLevel(0);
        // 核函数
        edge->setRobustKernel( new g2o::RobustKernelHuber() );
        optimizer.addEdge( edge );
        edges.push_back(edge);
    }

        //cout<<"开始优化"<<endl;
        optimizer.setVerbose(true);
        optimizer.initializeOptimization(0);
        optimizer.optimize(10);
        //cout<<"优化完毕"<<endl;

        //我们比较关心两帧之间的变换矩阵
        g2o::VertexSE3Expmap* v = dynamic_cast<g2o::VertexSE3Expmap*>( optimizer.vertex(1) );
        *pose = v->estimate();

        //Eigen::Isometry3d pose2 = v->estimate();
        //since there is no scale, unify the pose
        cout<<"g2o Pose="<<endl<<pose->matrix()<<endl;
}



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

        //projection
        p = K*T*P;

        //cout<<"p = "<<p<<endl;

        //normalization
        p = p/p(2);

        //cout<<"p2 = "<<p<<endl;

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

    P<<1,1,1;
    points_3d.push_back(P);

    P<<3,7,17;
    points_3d.push_back(P);

    P<<6,14,130;
    points_3d.push_back(P);

    P<<1,5,10;
    points_3d.push_back(P);

    P<<1,2,120;
    points_3d.push_back(P);

    P<<5,7,10;
    points_3d.push_back(P);

    P<<2,4,6;
    points_3d.push_back(P);

    P<<3,9,9;
    points_3d.push_back(P);

    P<<7,2,13;
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
    pinHoleProj(K,T,points_3d,points_2d1);



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
    pinHoleProj(K,T,points_3d,points_2d2);


    cout<<"start to find essential matrix"<<endl;

    //convert eigen point set to opencv point set type
    vector<Point2d> points1;
    vector<Point2d> points2;

    for(auto pp:points_2d1)
    {
        Point2d tempP = Point2d(pp(0),pp(1));
        points1.push_back(tempP);
    }

    for(auto pp:points_2d2)
    {
        Point2d tempP = Point2d(pp(0),pp(1));
        points2.push_back(tempP);
    }


    Mat e_mat,mask;
    e_mat = findEssentialMat(points1,points2,(fx+fy)*0.5,Point2d(cx,cy),RANSAC,0.999,1.0f,mask);    


    cout<<"Essential matrix from opencv = "<<endl<<e_mat<<endl;

    //start to recover pose based on essential matrix
    cout<<"start to find recover pose"<<endl;
    Mat R,t;

    recoverPose(e_mat, points1, points2, R, t, (fx+fy)*0.5, Point2d(cx,cy), mask); 

    Matrix3d rot_from_opencv;
    Vector3d trans_from_opencv;
    cv2eigen(R,rot_from_opencv);    
    cv2eigen(t,trans_from_opencv);    

    Vector3d recovered_rot;
    recovered_rot = rot_from_opencv.eulerAngles(0,1,2);
    cout<<"Pose from opencv, R = "<<recovered_rot.transpose()<<endl<<"t = "<<trans_from_opencv.transpose()<<endl;
    //scale translation
    trans_from_opencv = trans_from_opencv/trans_from_opencv(0);


    cout<<"After rescal, pose from opencv, R = "<<recovered_rot.transpose()<<endl<<"t = "<<trans_from_opencv.transpose()<<endl;
    
    Isometry3d pose_temp = Isometry3d::Identity();

    g2o_pose(points1, points2,&pose_temp);
    cout<<"g2o: pose = "<<endl<<pose_temp.matrix()<<endl;
    return -1;

}
