#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>


using namespace std;
using namespace Eigen;



//Euler to quaternion
Quaterniond E2Q(double roll, double pitch, double yaw)
{
    Quaterniond q;

    //the following step: quaternion = rotation matrix (conversion)
    q = AngleAxisd(roll,Vector3d::UnitX())
        *AngleAxisd(pitch,Vector3d::UnitY())
        *AngleAxisd(yaw,Vector3d::UnitZ());
    return q;
}


//Quaternion to Euler
Matrix3d Q2E(Quaterniond q)
{
    return q.toRotationMatrix(); //order roll pitch yaw
}

int main()
{

    
    cout<<"Q = "<<endl<<E2Q(0.1,0.2,0.3).toRotationMatrix()<<endl;
    
    cout<<"E = "<<endl<<Q2E(E2Q(0.1,0.2,0.3))<<endl;
    
    cout<<"F = "<<endl<<(Matrix3d)AngleAxisd(1.57,Vector3d::UnitX())<<endl;



    return -1;

}
