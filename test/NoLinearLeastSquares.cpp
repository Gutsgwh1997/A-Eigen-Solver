#include <iostream>
#include <random>
#include "backend/problem.h"

class abcVertex : public myslam::backend::Vertex
{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        abcVertex() : Vertex(3,3){}
        //abcVertex(int num_dimension, int local_dimension = -1) : myslam::backend::Vertex(num_dimension,-1){}
        virtual std::string TypeInfo() const
        {
            return "abc";
        }
};

class UnaryEdge : public myslam::backend ::Edge
{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        // 设定误差的维度是一维,变量(顶点)的个数是１个
        UnaryEdge(double x, double y, const std::vector<std::string>& vertices_type):Edge(1,1,vertices_type)
        {
            x_=x;
            y_=y;
        }

        // 计算残差
        virtual void ComputeResidual() override
        {
           Vec3 abc = verticies_[0]->Parameters();
           residual_[0] = std::exp(abc(0)*x_*x_ + abc(1)*x_ + abc(2)) - y_;
        }

        //计算雅克比矩阵 
        virtual void ComputeJacobians() override
        {
           Vec3 abc = verticies_[0]->Parameters();
           double exp_fy_ = std::exp(abc(0)*x_*x_ + abc(1)*x_ + abc(2));
           //注意维度!!!!!!
           Eigen::Matrix<double,1,3> jacobian;
           jacobian<<x_*x_*exp_fy_, x_*exp_fy_, exp_fy_;
           jacobians_[0]=jacobian;
        }

        //返回边的类型
        virtual std::string TypeInfo() const  override
        {
            return "CurvFittingEdge";
        }


    public:
        double x_;
        double y_;
};

int main()
{
    double a=1.0, b=2.0, c=2.0;    // Ground Truth
    int N = 100;                   // 数据点
    double w_sigma = 0.1;          // 噪声的标准差

    std::random_device rd;
    std::default_random_engine generator_(rd());
    std::normal_distribution<double> noise(0., w_sigma);

    myslam::backend::Problem problem(myslam::backend::Problem::ProblemType::GENERIC_PROBLEM);

    //基类的指针指向派生类的对象
    std::shared_ptr<abcVertex> vertex(new abcVertex());
    //设置初始值，不可少的一步
    vertex->SetParameters(Eigen::Vector3d (1.,1.,0.));
    //添加顶点，setOrdering中用到了
    problem.AddVertex(vertex);
    

    //构造N此观测
    std::vector<std::string> vertices_type=std::vector<std::string>{"abc"};
    for (int i = 0; i < N; ++i)
    {
        double x = i / 100.;
        double white_n = noise(generator_);
        double y = std::exp(a*x*x+b*x+c) + white_n;

        std::shared_ptr<UnaryEdge> edge(new UnaryEdge(x,y,vertices_type));
        edge->AddVertex(vertex);

        problem.AddEdge(edge);
    }

    std::cout<<"\nTest CurveFitting start..."<<std::endl;
    problem.Solve(60);

    std::cout << "-------After optimization, we got these parameters :" << std::endl;
    std::cout << vertex->Parameters().transpose() << std::endl;
    std::cout << "-------ground truth: " << std::endl;
    std::cout <<a<<"    "<<b<<"    "<<c<< std::endl;

        
    return 0;
}
