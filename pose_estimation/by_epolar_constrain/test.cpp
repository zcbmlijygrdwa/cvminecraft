#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>



using namespace std;
using namespace Eigen;


void pinHoleProj(Matrix3d K, vector<Eigen::Vector3d> points_3d,  vector<Eigen::Vector2d> &points_2d)
{

    Vector3d p(3);
    for(int i = 0; i<points_3d.size();i++)
    {
        Vector3d P = points_3d[i];
        //projection
        p = K*P;

        //normalization
        p = p/p(2);

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


    cout<<"after writting parameters, K = "<<endl<<K<<endl;

    return K;

}

int main()
{


    double fx = 250;
    double fy = 250;
    double cx = 100;
    double cy = 100;


    Matrix3d K = projMat(fx,fy,cx,cy);

    cout<<"after writting parameters, K = "<<endl<<K<<endl;

    Vector3d P(3);
    Vector3d p(3);
    vector<Eigen::Vector3d> points_3d;
    vector<Eigen::Vector2d> points_2d;

    P<<1,1,1000;
    points_3d.push_back(P);

    pinHoleProj(K,points_3d,points_2d);

    cout<<"2d points = "<<endl;
    for(auto pp:points_2d)
    {
        cout<<pp.transpose()<<endl;
    }

    return -1;

}
