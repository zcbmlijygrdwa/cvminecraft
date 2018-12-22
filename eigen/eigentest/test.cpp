
#include <iostream>
#include <Eigen/Dense>

typedef Eigen::Matrix<float, 1, 3> Matrix3f;

using namespace std;

int main()
{
    Matrix3f m1;
    m1 <<1,2,3;
    Matrix3f m2;
    m2 <<3,2,1;
    cout<<(m1+m2)<<endl;
}
