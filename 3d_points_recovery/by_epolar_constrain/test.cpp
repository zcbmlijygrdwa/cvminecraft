#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d/calib3d.hpp>



using namespace std;
using namespace Eigen;
using namespace cv;

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

    //for(int i = 0;i<120;i++)
    //{
    //    P<<i*3+2,(i-4)*3,i*i-4*i;
    //    points_3d.push_back(P);

    //}   

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

    cout<<"Now start to find 3d locations of points"<<endl;


    //preparing camera projection matrices
    Isometry3d T1 = Isometry3d::Identity();

    Isometry3d T2 = Isometry3d::Identity();
    T2.rotate(rot_from_opencv);
    T2.pretranslate(trans_from_opencv);
    cout<<"T1 = "<<endl<<T1.matrix()<<endl;
    cout<<"T2 = "<<endl<<T2.matrix()<<endl;
    cout<<"K*T1 = "<<endl<<(K*T1).matrix().block(0,0,3,4)<<endl;
    cout<<"K*T2 = "<<endl<<(K*T2).matrix().block(0,0,3,4)<<endl;

    Mat proj_cam_1;
    Mat proj_cam_2;

    eigen2cv((MatrixXd)((K*T1).matrix().block(0,0,3,4)),proj_cam_1);
    eigen2cv((MatrixXd)((K*T2).matrix().block(0,0,3,4)),proj_cam_2);

    cout<<"proj_cam_1 = "<<endl<<proj_cam_1<<endl;
    cout<<"proj_cam_2 = "<<endl<<proj_cam_2<<endl;


    //call triangulatePoints from opencv
    Mat results = cv::Mat(1,points1.size(),CV_64FC4); 
    triangulatePoints(proj_cam_1,proj_cam_2,points1,points2,results);
    //transpose
    results = results.t();
    
    cout<<"results.size() = ["<<results.rows<<","<<results.cols<<"]"<<endl;
    cout<<"results = "<<endl<<results.t()<<endl;

    MatrixXd results_eigen;
    cv2eigen(results,results_eigen);

    
    cout<<"1 results_eigen = "<<endl<<results_eigen<<endl;

    cout<<"results_eigen.size() = ["<<results_eigen.rows()<<","<<results_eigen.cols()<<"]"<<endl;
    //homogeneous to non-homogeneous
    for(int i = 0; i < results_eigen.rows() ; i++)
    {
        results_eigen.row(i) /= results_eigen(i,3);
    }


    cout<<"results_eigen = "<<endl<<results_eigen<<endl;

    return -1;

}
