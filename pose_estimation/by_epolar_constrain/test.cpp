#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>



using namespace std;
using namespace Eigen;

void pinHoleProj(Matrix3d K, Isometry3d T,vector<Eigen::Vector3d> points_3d,  vector<Eigen::Vector2d> &points_2d)
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
        points_2d.push_back(pp);
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
    vector<Eigen::Vector2d> points_2d;

    P<<1,1,1;
    points_3d.push_back(P);

    P<<1,1,10;
    points_3d.push_back(P);

    P<<1,1,100;
    points_3d.push_back(P);

    P<<1,1,1000;
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
    pinHoleProj(K,T,points_3d,points_2d);

    cout<<"2d points = "<<endl;
    for(auto pp:points_2d)
    {
        cout<<pp.transpose()<<endl;
    }


    //for the 2nd transform

    cout<<endl<<endl<<endl<<"2Nd transform!"<<endl;
    T = Isometry3d::Identity();

    rot_mat = AngleAxisd(0.1,Vector3d::UnitX())
                        * AngleAxisd(0.2,Vector3d::UnitY())
                        * AngleAxisd(0.3,Vector3d::UnitZ());

    T.rotate(rot_mat);


    trans_mat<<0,0,0;

    T.pretranslate(trans_mat);

    cout<<"T = "<<T.matrix()<<endl;
    pinHoleProj(K,T,points_3d,points_2d);

    cout<<"2d points = "<<endl;
    for(auto pp:points_2d)
    {
        cout<<pp.transpose()<<endl;
    }

    

    return -1;

}
