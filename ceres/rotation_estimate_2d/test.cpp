
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

    CostFunctor(Point2d& point1, Point2d& point2)
    {
        p1_ptr = &point1;
        p2_ptr = &point2;
    }

    Point2d* p1_ptr;
    Point2d* p2_ptr;

    template <typename T>
        bool operator()(const T* const x, T* residual) const {

            Eigen::Matrix<T,2,2> rot;
            rot<<cos(x[0]) , -sin(x[0]),
                sin(x[0]), cos(x[0]);

            Eigen::Matrix<T,2,1> point;
            point<<T(p1_ptr->x),T(p1_ptr->y);

            point = rot*point;

            T newX = point(0);
            T newY = point(1);

            residual[0] = (T)((newX-p2_ptr->x)*(newX-p2_ptr->x) + (newY-p2_ptr->y)*(newY-p2_ptr->y));
            //residual[0] = T(10.0) - sin(x[0])*x[1];
            return true;
        }
};

//主函数

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);

    // 寻优参数x的初始值，为5
    double initial_x = 0.0;
    //cout<<"initial_x[0] = "<<initial_x[0]<<", initial_x[1] = "<<initial_x[1]<<endl;
    double x;
    x = initial_x;

    //cout<<"x[0] = "<<x[0]<<", x[1] = "<<x[1]<<endl;


    Point2d testp;


    vector<Point2d> points1;
    vector<Point2d> points2;


    points1.push_back(Point2d(1,0));
    points1.push_back(Point2d(2,3));
    points1.push_back(Point2d(4,3));
    points1.push_back(Point2d(23,65));


    points2.push_back(Point2d( 0.6216, 0.7833));
    points2.push_back(Point2d(-1.1068, 3.4315));
    points2.push_back(Point2d(-1.1068, 3.4315));
    points2.push_back(Point2d(-36.6192, 58.4212));


    //double xs[4];
    //xs [0] = 0.7648;
    //xs [1] = 0.8855;
    //xs [2] = 1.0061;
    //xs [3] = -1.4502;

    //double ys[4];
    //ys[0] = 0.6442;
    //ys[1] = 2.0533;
    //ys[2] = 3.4623;
    //ys[3] = 7.9308;

    double theta = 0.0; //solution is -0.9

    // 第二部分：构建寻优问题
    Problem problem;

    for(int i = 0 ; i<points2.size() ; i++)
    {
        CostFunction* cost_function =
            new AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor(points2[i],points1[i])); //使用自动求导，将之前的代价函数结构体传入，第一个1是输出维度，即残差的维度，第二个1是输入维度，即待寻优参数x的维度。
        problem.AddResidualBlock(cost_function, NULL, &x); //向问题中添加误差项，本问题比较简单，添加一个就行。
    }

    //第三部分： 配置并运行求解器
    Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR; //配置增量方程的解法
    options.minimizer_progress_to_stdout = true;//输出到cout
    Solver::Summary summary;//优化信息
    ceres::Solve(options, &problem, &summary);//求解!!!

    std::cout << summary.BriefReport() << "\n";//输出优化的简要信息
    //最终结果
    std::cout << "x : " << initial_x
        << " -> " << x << "\n";
    //cout<<"x[0] = "<<x[0]<<", x[1] = "<<x[1]<<endl;
    return 0;
}
