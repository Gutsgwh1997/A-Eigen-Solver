#include <iostream>
#include <random>
#include "backend/problem.h"

using namespace std;
using namespace myslam::backend;

class aVertex : public myslam::backend::Vertex
{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        aVertex(int demension, int local_deminsion, string type) : Vertex(demension,local_deminsion), type_(type){}
        //abcVertex(int num_dimension, int local_dimension = -1) : myslam::backend::Vertex(num_dimension,-1){}
        virtual std::string TypeInfo() const
        {
            return type_;
        };
    public:
        string type_;
        
};
class UnaryEdge : public myslam::backend ::Edge
{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        // 设定误差的维度是一维,变量(顶点)的个数是１个
        UnaryEdge(double x, double y, const std::vector<std::string>& vertices_type):Edge(1,3,vertices_type)
        {
            x_=x;
            y_=y;
        }

        // 计算残差
        virtual void ComputeResidual() override
        {
           Vec1 a = verticies_[0]->Parameters();
           Vec1 b = verticies_[1]->Parameters();
           Vec1 c = verticies_[2]->Parameters();
           residual_[0] = std::exp(a(0)*x_*x_ + b(0)*x_ + c(0)) - y_;
        }

        //计算雅克比矩阵 
        virtual void ComputeJacobians() override
        {
           Vec1 a = verticies_[0]->Parameters();
           Vec1 b = verticies_[1]->Parameters();
           Vec1 c = verticies_[2]->Parameters();
           double exp_y = std::exp(a(0)*x_*x_ + b(0)*x_ + c(0));

           //注意维度!!!!!!
           Vec1 tmp;
           tmp<<exp_y*x_*x_;
           jacobians_[0]=tmp;
           tmp<<exp_y*x_;
           jacobians_[1]=tmp;
           tmp<<exp_y;
           jacobians_[2]=tmp;
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

    Problem problem(myslam::backend::Problem::ProblemType::GENERIC_PROBLEM);

    //基类的指针指向派生类的对象
    std::shared_ptr<aVertex> vertexa(new aVertex(1,1,"a"));
    std::shared_ptr<aVertex> vertexb(new aVertex(1,1,"b"));
    std::shared_ptr<aVertex> vertexc(new aVertex(1,1,"c"));
    //设置初始值，不可少的一步
    vertexa->SetParameters(Vec1 (0.));
    vertexb->SetParameters(Vec1 (0.));
    vertexc->SetParameters(Vec1 (0.));
    //设定其在H矩阵中的位置
    vertexa->SetOrderingId(0);
    vertexb->SetOrderingId(1);
    vertexc->SetOrderingId(2);
    //添加顶点，setOrdering中用到了
    problem.AddVertex(vertexa);
    problem.AddVertex(vertexb);
    problem.AddVertex(vertexc);
    

    //构造N此观测
    std::vector<std::string> vertices_type;
    vertices_type.push_back(vertexa->TypeInfo());
    vertices_type.push_back(vertexb->TypeInfo());
    vertices_type.push_back(vertexc->TypeInfo());

    for (int i = 0; i < N; ++i)
    {
        double x = i / 100.;
        double white_n = noise(generator_);
        double y = std::exp(a*x*x+b*x+c) + white_n;

        std::shared_ptr<UnaryEdge> edge(new UnaryEdge(x,y,vertices_type));
        edge->AddVertex(vertexa);
        edge->AddVertex(vertexb);
        edge->AddVertex(vertexc);

        problem.AddEdge(edge);
    }

    std::cout<<"\nTest CurveFitting start..."<<std::endl;
    problem.Solve(60);

    std::cout << "-------After optimization, we got these parameters :" << std::endl;
    std::cout << vertexa->Parameters().transpose() <<" "; 
    std::cout << vertexb->Parameters().transpose() <<" "; 
    std::cout << vertexc->Parameters().transpose() << std::endl;
    std::cout << "-------ground truth: " << std::endl;
    std::cout <<a<<"    "<<b<<"    "<<c<< std::endl;

        
    return 0;
}
