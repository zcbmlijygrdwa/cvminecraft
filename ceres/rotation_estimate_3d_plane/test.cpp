
#include<iostream>
#include<ceres/ceres.h>

#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>


using namespace std;
using namespace cv;
using namespace ceres;
using namespace Eigen;

//第一部分：构建代价函数，重载（）符号，仿函数的小技巧
struct CostFunctor {

    CostFunctor(Point3d& point)
    {
        p_ptr = &point;
    }

    Point3d* p_ptr;

    template <typename T>
        bool operator()(const T* const x, T* residual) const {

            //x[0]: roll in this case
            //x[1]: pitch in this case

            // https://en.wikipedia.org/wiki/Rotation_matrix 
            Eigen::Matrix<T,3,3> rot_x;
            rot_x<<T(1),T(0),T(0),
                T(0),cos(x[0]),-sin(x[0]),
                T(0),sin(x[0]),cos(x[0]);

            Eigen::Matrix<T,3,3> rot_y;
            rot_y<<cos(x[1]),T(0),sin(x[1]),
                T(0),T(1),T(0),
                -sin(x[1]),T(0),cos(x[1]);

            Eigen::Matrix<T,3,3> rot;
            rot = rot_x*rot_y;

            Eigen::Matrix<T,3,1> point;
            point<<T(p_ptr->x),
                T(p_ptr->y),
                T(p_ptr->z);
            //cout<<"------------"<<endl;
            //cout<<"roll = "<<x[0]<<", pitch = "<<x[1]<<endl;
            //cout<<"point(0) = "<<point(0)<<", point(1) = "<<point(1)<<", point(2) = "<<point(2)<<endl;
            point = rot*point;
            //cout<<"Apoint(0) = "<<point(0)<<", Apoint(1) = "<<point(1)<<", Apoint(2) = "<<point(2)<<endl;

            T newX = point(0);
            T newY = point(1);
            T newZ = point(2);


            //residual[0] = (T)((newX-p2_ptr->x)*(newX-p2_ptr->x) + (newY-p2_ptr->y)*(newY-p2_ptr->y));
            residual[0] = abs(newZ);
            //cout<<"abs(newZ) = "<<abs(newZ)<<endl;
            return true;
        }
};

//主函数

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);

    double rotation[2]; //solution is 0.12 in roll and 0.34 degrees. Initial guess is 0.1 and 0.4
    rotation[0] = 0.1;
    rotation[1] = 0.4;

    Point2d testp;


    vector<Point3d> points;

    points.push_back(Point3d(0.9428,    2.0255,   -0.0917));
    points.push_back(Point3d(3.7710,    3.1381,   -0.9652));
    points.push_back(Point3d(5.6565,    4.2108,   -1.5077));
    points.push_back(Point3d(5.6565,    2.2252,   -1.7471));
    points.push_back(Point3d(7.5420,    5.2834,   -2.0501));



    // 第二部分：构建寻优问题
    Problem problem;

    for(int i = 0 ; i<points.size() ; i++)
    {
        CostFunction* cost_function =
            new AutoDiffCostFunction<CostFunctor, 1, 2>(new CostFunctor(points[i])); //使用自动求导，将之前的代价函数结构体传入，第一个1是输出维度，即残差的维度，第二个1是输入维度，即待寻优参数x的维度。
        problem.AddResidualBlock(cost_function, NULL, rotation); //向问题中添加误差项，本问题比较简单，添加一个就行。
    }

    //第三部分： 配置并运行求解器
    Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR; //配置增量方程的解法
    options.minimizer_progress_to_stdout = true;//输出到cout
    Solver::Summary summary;//优化信息
    ceres::Solve(options, &problem, &summary);//求解!!!

    std::cout << summary.BriefReport() << "\n";//输出优化的简要信息
    //最终结果
    //std::cout << "x : " << initial_x << " -> " << x << "\n";
    cout<<"rotation: rotation[0] = "<<rotation[0]<<", rotation[1] = "<<rotation[1]<<endl;
    return 0;
}
