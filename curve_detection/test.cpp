
// https://docs.opencv.org/3.4/da/d97/tutorial_threshold_inRange.html

#include<opencv2/opencv.hpp>
#include<ceres/ceres.h>

using namespace cv;
using namespace std;


using namespace ceres;

//第一部分：构建代价函数，重载（）符号，仿函数的小技巧
struct CostFunctor {

    CostFunctor(double a, double b)
    {
        x = a;
        y = b;
    }

    bool operator()(const double* const t, double* residual) const {
        //residual[0] = 10.0 - x[0];

        //t[0]: a
        //t[1]: b
        //t[2]: c

        //curve: f(x) = a*x^2 + b*x + c;

        residual[0] = t[0]*x*x+t[1]*x+t[2] - y;
        return true;
    }

    double x;
    double y;
};

//主函数
int main(int argc, char** argv)
{



    const char* default_file = "../../data/image/lane1.jpg";
    const char* filename = argc >=2 ? argv[1] : default_file;

    //Input and Output Image;
    Mat input, input2;

    //Load the image
    input = imread(filename, 1 );

    Mat output = Mat::zeros(Size(input.cols,input.rows),CV_8UC1);

    // first convert the image to grayscale
    cvtColor(input, input, CV_RGB2GRAY);

    // then adjust the threshold to actually make it binary
    threshold(input, input, 100, 255, CV_THRESH_BINARY);

    vector<Point> locations;   // output, locations of non-zero pixels

    //find location of all non-zero pixels
    cv::findNonZero(input, locations);

    cout<<"detected "<<locations.size()<<" non-zero points"<<endl;

    // access pixel coordinates
    //Point pnt = locations[i];

    //for(int i = 0 ; i < locations.size();i++)
    //{
    //    cout<<"locations["<<i<<"] = "<<locations[i]<<endl;
    //}

    google::InitGoogleLogging(argv[0]);

    // 寻优参数x的初始值，为5

    double initial_x[] = {1,1,1};  //initial guess
    double x[] = {1,1,1};  //initial guess

    // 第二部分：构建寻优问题

    //f(x) = 2*x^2 + 3*x + 4;
    //x = 1, y = 9 
    //x = 2, y = 18 
    //x = 3, y = 31 
    //x = 4, y = 48 
    //x = 5, y = 69 

    int x_max = 0;
    int y_max = 0;
    int x_min = input.rows;
    int y_min = input.cols;


    Problem problem;

    for(int i = 0 ; i < locations.size();i++)
    {
        x_min = min(x_min,locations[i].x);
        x_max = max(x_max,locations[i].x);
        y_min = min(y_min,locations[i].y);
        y_max = max(y_max,locations[i].y);

        //cout<<"locations["<<i<<"] = "<<locations[i]<<endl;
        CostFunction* cost_function = new NumericDiffCostFunction<CostFunctor, CENTRAL, 1, 3>(new CostFunctor(locations[i].x,locations[i].y)); //使用自动求导，将之前的代价函数结构体传入，第一个1是输出维度，即残差的维度，第二个1是输入维度，即>待寻优参数x的维度。
        problem.AddResidualBlock(cost_function, NULL, x); //向问题中添加误差项，本问题比较简单，添加一个就行。
    }

    cout<<"x_min = "<<x_min<<endl;
    cout<<"x_max = "<<x_max<<endl;
    cout<<"y_min = "<<y_min<<endl;
    cout<<"y_max = "<<y_max<<endl;

    //第三部分： 配置并运行求解器
    Solver::Options options;
    //options::max_num_iterations = 150;  //Default: 50
    options.linear_solver_type = ceres::DENSE_QR; //配置增量方程的解法
    options.minimizer_progress_to_stdout = true;//输出到cout
    Solver::Summary summary;//优化信息
    Solve(options, &problem, &summary);//求解!!!

    std::cout << summary.BriefReport() << "\n";//输出优化的简要信息
    //最终结果

    std::cout<<"init = "<<initial_x[0]<<", "<<initial_x[1]<<", "<<initial_x[2]<<endl;
    std::cout<<"x = "<<x[0]<<", "<<x[1]<<", "<<x[2]<<endl;

    cout<<"locations[0] = "<<locations[0]<<endl;
    cout<<"locations[1] = "<<locations[1]<<endl;


    //Mat.at<type>(row,col):
    // -------------> col
    // |
    // |
    // |
    // |
    // |
    // |
    // V
    // row


    int trow = 0;
    int tcol = 0;
    //draw the resulted curve
    for(int i = x_min ;  i <=x_max ; i++)
    {
        trow = uint(x[0]*i*i+x[1]*i+x[2]);
        tcol = i;
        cout<<"tcol = "<<tcol<<", trow = "<<trow<<" ,input.cols = "<<input.cols<<", input.rows = "<<input.rows<<endl;
        if(trow>=0 && tcol>=0 && tcol <input.cols && trow<input.rows)
        {
            //trow = 100;
            //tcol = 200;
            input.at<uchar>(trow,tcol) = 255;
            output.at<uchar>(trow,tcol) = 255;
        }
    }

    //Display input and output
    imshow("Input",input);
    imshow("Output",output);

    //imwrite("output.jpg",output);

    waitKey(0);
    return 0;
}
