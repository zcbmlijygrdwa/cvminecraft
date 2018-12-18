#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <Eigen/Geometry>

#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/types/slam3d/edge_se3.h"
#include "g2o/core/factory.h"

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"

using namespace std;
using namespace g2o;
using namespace Eigen;

int main()
{

    g2o::SparseOptimizer optimizer;//全局优化器
    optimizer.setVerbose(true);//调试信息输出
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;//6代表pose的自由度 3代表landmark的自由度
    //linearSolver= new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();
    linearSolver = new g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType>(linearSolver));

    //优化方法LM
    //g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(std::unique_ptr<g2o::BlockSolver_6_3>(solver_ptr));
    g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(std::unique_ptr<g2o::BlockSolver_6_3>(solver_ptr));//这个能将误差全局平摊
    optimizer.setAlgorithm(solver);


    vector<VertexSE3*> vertices;//pose
    vector<EdgeSE3*> odometryEdges;
    vector<EdgeSE3*> edges;

    //设置pose
    int id = 0;
    for(int i=0; i<100; i++)
    {
        VertexSE3* v = new VertexSE3;
        v->setId(id++);

        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();;//旋转和平移的集合,4*4的矩阵
        Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
        Eigen::Vector3d trans =  Eigen::Vector3d(i, 0, 0);
        T.rotate(rot);
        T.translate(trans);
        //cout << "Transform matrix = \n" << T.matrix() <<endl;

        v->setEstimate(T);
        vertices.push_back(v);
        optimizer.addVertex(v);
    }

    //生成里程计的边
    for(int i=1; i<vertices.size(); i++)
    {
        VertexSE3* pre = vertices[i-1];
        VertexSE3* cur = vertices[i];
        Eigen::Isometry3d T = pre->estimate().inverse() * cur->estimate();

        EdgeSE3* e = new EdgeSE3;
        e->setVertex(0,pre);
        e->setVertex(1,cur);
        e->setMeasurement(T);
        Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Zero();
        information.block<3,3>(0,0) = 0.01 * Eigen::Matrix3d::Identity();
        information.block<3,3>(3,3) = 0.01 * Eigen::Matrix3d::Identity();;
        e->setInformation(information);
        odometryEdges.push_back(e);
        edges.push_back(e);
        optimizer.addEdge(e);
    }

    //添加一条首尾相连的边,从尾巴指向头
    {
        EdgeSE3* e = new EdgeSE3;
        Eigen::Isometry3d T = vertices[vertices.size()-1]->estimate().inverse() * vertices[0]->estimate();
        Eigen::Isometry3d T_delta = Eigen::Isometry3d::Identity();
        T_delta.pretranslate(Eigen::Vector3d(1,2,3));
        T = T * T_delta;//故意加上偏差
        e->setVertex(0, vertices[vertices.size()-1]);
        e->setVertex(1, vertices[0]);
        e->setMeasurement(T);
        Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Zero();
        information.block<3,3>(0,0) = 0.01 * Eigen::Matrix3d::Identity();
        information.block<3,3>(3,3) = 0.01 * Eigen::Matrix3d::Identity();;
        e->setInformation(information);
        odometryEdges.push_back(e);
        edges.push_back(e);
        optimizer.addEdge(e);
    }

    //display all vertex before optimazation
    for(int i=1; i<vertices.size(); i++)
    {
        g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>( optimizer.vertex(i) );
        Eigen::Isometry3d T = v->estimate();
        cout<<"vertex["<<i<<"]: traslate = "<<T.translation().matrix().transpose()<<endl;
    }

    cout<<"--------------------------------------------------------------"<<endl;
    optimizer.save("test.g2o");
    optimizer.initializeOptimization();
    optimizer.optimize(10);


    //display all vertex after optimazation
    for(int i=1; i<vertices.size(); i++)
    {
        g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>( optimizer.vertex(i) );
        Eigen::Isometry3d T = v->estimate();
        cout<<"vertex["<<i<<"]: traslate = "<<T.translation().matrix().transpose()<<endl;
    }


    return 0;
}

//来源：CSDN 
//
//原文：https://blog.csdn.net/u010566411/article/details/72904266 
//
//版权声明：本文为博主原创文章，转载请附上博文链接！
