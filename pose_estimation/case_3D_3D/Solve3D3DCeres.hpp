#ifndef SolvePnpCeres_h
#define SolvePnpCeres_h

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include <Eigen/Eigen>
#include <Eigen/Geometry>

#include <iostream>

struct CostFunctor
{

    CostFunctor(cv::Point3d& point_3d1, cv::Point3d& point_3d2)
    {
        p_3d1 = point_3d1;
        p_3d2 = point_3d2;
    }


    template<typename T>
        bool operator()(const T* const cere_r, const T* const cere_t, T* residual) const
        {
            T p_origin[3];
            T p_target[3];
            T p_rotate[3];
            T p_rotate_translate[3];

            p_origin[0] = T(p_3d1.x);
            p_origin[1] = T(p_3d1.y);
            p_origin[2] = T(p_3d1.z);


            // cere_r[0,1,2] are the angle-axis rotation.
            //ceres::AngleAxisRotatePoint(cere_r,p_origin,p_rotate);

            //using rotation matrix for rotating the 3D point


            T roll = cere_r[0];
            T pitch = cere_r[1];
            T yaw = cere_r[2];
            Eigen::Matrix<T,3,3> Rx;
            Rx<<T(1),T(0),T(0),
                T(0),cos(roll),-sin(roll),
                T(0),sin(roll),cos(roll);

            Eigen::Matrix<T,3,3> Ry;
            Ry<<cos(pitch),T(0),sin(pitch),
                T(0),T(1),T(0),
                -sin(pitch),T(0),cos(pitch);


            Eigen::Matrix<T,3,3> Rz;
            Rz<<cos(yaw),-sin(yaw),T(0),
                sin(yaw),cos(yaw),T(0),
                T(0),T(0),T(1);

            Eigen::Matrix<T,3,3> rot = Rx*Ry*Rz;

            Eigen::Matrix<T,3,1> p_o;
            p_o<<p_origin[0],
            p_origin[1],
            p_origin[2];

            p_o = rot*p_o;
            p_rotate[0] = p_o(0);
            p_rotate[1] = p_o(1);
            p_rotate[2] = p_o(2);

            //cout<<"******"<<endl;
            //cout<<"cere_r = "<<cere_r[0]<<", "<<cere_r[1]<<", "<<cere_r[2]<<endl;
            //cout<<"p_origin = "<<p_origin[0]<<", "<<p_origin[1]<<", "<<p_origin[2]<<endl;
            //cout<<"p_rotate = "<<p_rotate[0]<<", "<<p_rotate[1]<<", "<<p_rotate[2]<<endl;

            p_rotate_translate[0] = p_rotate[0] + cere_t[0];
            p_rotate_translate[1] = p_rotate[1] + cere_t[1];
            p_rotate_translate[2] = p_rotate[2] + cere_t[2];


            p_target[0] = T(p_3d2.x);
            p_target[1] = T(p_3d2.y);
            p_target[2] = T(p_3d2.z);

            //evaluate residual
            residual[0] = p_target[0]-p_rotate_translate[0];
            residual[1] = p_target[1]-p_rotate_translate[1];
            residual[2] = p_target[2]-p_rotate_translate[2];
            return true;
        }

    cv::Point3d p_3d1;
    cv::Point3d p_3d2;
    double fx,fy,cx,cy;
};
class Solve3D3DCeres
{
    public:
        std::vector<cv::Point3d> p_3d_set1;
        std::vector<cv::Point3d> p_3d_set2;

        ceres::Problem problem;
        ceres::Solver::Options option;

        double cere_r[3];
        double cere_t[3];


        void init()
        {


            //take initial guess of cere_r, cere_t

            cere_r[0] = 0.1;
            cere_r[1] = 0.2;
            cere_r[2] = 0.3;

            cere_t[0] = 0.1;
            cere_t[1] = 0.2;
            cere_t[2] = 0.3;

            option.max_num_iterations = 1500; //default = 50;

        }
        void setInputs(std::vector<cv::Point3d>& i1, std::vector<cv::Point3d>& i2)
        {
            p_3d_set1 = i1;
            p_3d_set2 = i2;

            if(i1.size()!=i2.size())
            {
                std::cout<<"Two input have different size, wrong inputs."<<std::endl;
                return;
            }

            for(uint i  = 0 ; i < p_3d_set1.size();i++)
            {

                //http://ceres-solver.org/nnls_modeling.html#autodiffcostfunction
                //<CostFunctor,residual_dimension, dimension_1st_argument_in_operator(), dimension_2nd_argument_in_operator()>
                ceres::CostFunction* costfunction = new ceres::AutoDiffCostFunction<CostFunctor,3,3,3>(new CostFunctor(p_3d_set1[i],p_3d_set2[i]));
                problem.AddResidualBlock(costfunction,NULL,cere_r,cere_t);
            }
        }

        void optimize(cv::Mat* rot, cv::Mat* trans, bool ifShowInfo)
        {
            option.linear_solver_type=ceres::DENSE_SCHUR;
            //输出迭代信息到屏幕
            option.minimizer_progress_to_stdout=ifShowInfo;
            //显示优化信息
            ceres::Solver::Summary summary;
            //开始求解
            ceres::Solve(option,&problem,&summary);
            //显示优化信息
            if(ifShowInfo)
            {
                std::cout<<summary.FullReport()<<std::endl;
            }

            //std::cout<<"cere_r[0] = "<<cere_r[0]<<std::endl;
            //std::cout<<"cere_r[1] = "<<cere_r[1]<<std::endl;
            //std::cout<<"cere_r[2] = "<<cere_r[2]<<std::endl;

            rot->at<double>(0,0) = cere_r[0];
            rot->at<double>(1,0) = cere_r[1];
            rot->at<double>(2,0) = cere_r[2];

            trans->at<double>(0,0) = cere_t[0];
            trans->at<double>(1,0) = cere_t[1];
            trans->at<double>(2,0) = cere_t[2];
            //std::cout<<"rot = "<<*rot<<std::endl;
        }
};
#endif
