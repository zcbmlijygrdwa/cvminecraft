#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry> //for quaternion

using namespace std;
using namespace Eigen;

void quaternionToRot(double w, double x, double y, double z, Matrix3d* rotmat)
{
    //rotation matrix from quaternion by using eigen
    Quaterniond q1;
    q1.w() = w;
    q1.x() = x;
    q1.y() = y;
    q1.z() = z;

    //normalize the quaternion
    q1 = q1.normalized();

    //to rot matrix
    *rotmat = q1.toRotationMatrix();
}


int main(int argc, char** argv)
{


    Vector3d point = Vector3d(0.5,0,0.2);
    cout<<"point = "<<point<<endl;

    Matrix3d* rot_mat_ptr1 = new Matrix3d();
    quaternionToRot(0.35,0.2,0.3,0.1,rot_mat_ptr1);

    Matrix3d* rot_mat_ptr2 = new Matrix3d();
    quaternionToRot(-0.5,0.4,-0.1,0.2,rot_mat_ptr2);
    cout<<*rot_mat_ptr1<<endl;
    cout<<*rot_mat_ptr2<<endl;

    //crate a new transform matrix (Isometry)
    Isometry3d T1 = Isometry3d::Identity();
    //T1.rotate(*rot_mat_ptr1);
    T1.rotate(Quaterniond(0.35,0.2,0.3,0.1).normalized());
    T1.pretranslate(Vector3d(0.3,0.1,0.1));
    cout<<"T1 = \n"<<T1.matrix()<<endl;

    Isometry3d T2 = Isometry3d::Identity();
    //T2.rotate(*rot_mat_ptr2);
    Quaterniond q = AngleAxisd(0.412343,Vector3d::UnitX())
*AngleAxisd(0.0547924,Vector3d::UnitY())
*AngleAxisd(0.718975,Vector3d::UnitZ());
    T2.rotate(q.normalized());
    T2.pretranslate(Vector3d(0,0,0));
    cout<<"T2 = \n"<<T2.matrix()<<endl;
    

    //invert T1
    T1 = T1.inverse();

    //transfer point back
    Vector3d originalPoint = T1*point;
    cout<<"originalPoint = \n"<<originalPoint<<endl;    

    //transfer point to T2
    originalPoint = Vector3d(12, 12, 15);
    Vector3d point2 = T2*originalPoint;
    cout<<"point2 = \n"<<point2<<endl;    
    
return 0;
}

