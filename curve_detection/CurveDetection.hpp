#ifndef CurveDetection_h
#define CurveDetection_h

#include<opencv2/opencv.hpp>
#include<ceres/ceres.h>

using namespace cv;
using namespace std;
using namespace ceres;

class CurveDetection
{
    public:
        Mat input, imgGrayscale,imgBinary;
        vector<Point2d> output;

        // 寻优参数x的初始值，为 {1,1,1}

        double initial_x[3];
        double x[3];

        // 第二部分：构建寻优问题

        //f(x) = 2*x^2 + 3*x + 4;
        //x = 1, y = 9 
        //x = 2, y = 18 
        //x = 3, y = 31 
        //x = 4, y = 48 
        //x = 5, y = 69 

        int x_max;
        int x_min;


        Problem problem;



        Solver::Options options;
        Solver::Summary summary;//优化信息

        CurveDetection()
        {
            //initial guess
            initial_x[0] = 1;
            initial_x[1] = 1;
            initial_x[2] = 1;
            x[0] = 1;
            x[1] = 1;
            x[2] = 1;

            options.linear_solver_type = ceres::DENSE_QR; //配置增量方程的解法
            //options.minimizer_progress_to_stdout = true;//输出到cout

        }

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

        void setColorInput(Mat& in)
        {
            input = in;
            // first convert the image to grayscale
            cvtColor(input, imgGrayscale, CV_RGB2GRAY);

            // then adjust the threshold to actually make it binary
            threshold(imgGrayscale, imgBinary, 100, 255, CV_THRESH_BINARY);

            x_max = 0;
            x_min = imgBinary.rows;

            if(x_min==0)
            {
                cout<<"zero row or column detected!!!"<<endl;
            }
        }

        void setGrayscaleInput(Mat& in)
        {
            imgGrayscale = in;

            threshold(imgGrayscale, imgBinary, 100, 255, CV_THRESH_BINARY);

            x_max = 0;
            x_min = imgBinary.rows;

            if(x_min==0)
            {
                cout<<"zero row or column detected!!!"<<endl;
            }
        }

        void setBinaryInput(Mat& in)
        {
            imgBinary = in;

            x_max = 0;
            x_min = imgBinary.rows;

            if(x_min==0)
            {
                cout<<"zero row or column detected!!!"<<endl;
            }
        }


        void solve()
        {
            vector<Point> locations;   // output, locations of non-zero pixels

            //find location of all non-zero pixels
            cv::findNonZero(imgBinary, locations);

            cout<<"detected "<<locations.size()<<" non-zero points"<<endl;

            if(locations.size()==0)
            {
                cout<<"No non-zero points error!!!"<<endl;
                return;
            }


            for(uint i = 0 ; i < locations.size();i++)
            {
                x_min = min(x_min,locations[i].x);
                x_max = max(x_max,locations[i].x);

                CostFunction* cost_function = new NumericDiffCostFunction<CostFunctor, CENTRAL, 1, 3>(new CostFunctor(locations[i].x,locations[i].y)); //使用自动求导，将之前的代价函数结构体传入，第一个1是输出维度，即残差的维度，第二个1是输入维度，即>待寻优参数x的维度。
                problem.AddResidualBlock(cost_function, NULL, x); //向问题中添加误差项，本问题比较简单，添加一个就行。
            }





            Solve(options, &problem, &summary);//求解!!!

            std::cout << summary.BriefReport() << "\n";//输出优化的简要信息
            //最终结果

            std::cout<<"init = "<<initial_x[0]<<", "<<initial_x[1]<<", "<<initial_x[2]<<endl;
            std::cout<<"x = "<<x[0]<<", "<<x[1]<<", "<<x[2]<<endl;

            output.clear();

            int trow = 0;
            int tcol = 0;
            //draw the resulted curve
            for(int i = x_min ;  i <=x_max ; i++)
            {
                trow = uint(x[0]*i*i+x[1]*i+x[2]);
                tcol = i;
                //cout<<"tcol = "<<tcol<<", trow = "<<trow<<" ,imgBinary.cols = "<<imgBinary.cols<<", imgBinary.rows = "<<imgBinary.rows<<endl;
                if(trow>=0 && tcol>=0 && tcol <imgBinary.cols && trow<imgBinary.rows)
                {
                    output.push_back(Point2d(tcol,trow));
                }
            }
        }        


        vector<Point2d> getResult()
        {
            return output;
        }
};
#endif
