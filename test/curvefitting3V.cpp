#include <iostream>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_multi_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <Eigen/Core>
#include <cmath>
#include <random>
#include <chrono>
using namespace std;

// 曲线模型的顶点，模板参数：优化变量维度和数据类型
// 这里的类型也是_estimate的类型，根据类型重写oplusImpl
class CurveFittingVertex: public g2o::BaseVertex<1, double>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // 重置估计值，纯虚函数，必须重写
    virtual void setToOriginImpl() override // 重置
    {
        _estimate = 0;
    }

    //纯虚函数中定义了参数的类型
    virtual void oplusImpl( const double* update ) override // 更新
    {
        _estimate += update[0];
    }

    // 存盘和读盘：留空,必须重写
    virtual bool read( istream& in ) {}
    virtual bool write( ostream& out ) const  {}
};

class CurveFittingEdge3: public g2o::BaseMultiEdge<1,double>
{
public :
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CurveFittingEdge3( double x): BaseMultiEdge(),_x(x) {
        //此函数可以定义大小,一定需要，否则运行报错！！！！
       resize(3);
    }

    // 计算曲线模型误差
    void computeError() override
    {
        const CurveFittingVertex* v0 = static_cast<const CurveFittingVertex*> (_vertices[0]);
        const double a = v0->estimate();
        const CurveFittingVertex* v1 = static_cast<const CurveFittingVertex*> (_vertices[1]);
        const double b = v1->estimate();
        const CurveFittingVertex* v2 = static_cast<const CurveFittingVertex*> (_vertices[2]);
        const double c = v2->estimate();
        //_error是一个Eigen::Matrix<>,需要（a,b）这样写
        _error(0,0) = std::exp( a*_x*_x + b*_x + c)- _measurement;
    }

    void linearizeOplus() override
    {
        const CurveFittingVertex* v0 = static_cast<const CurveFittingVertex*> (_vertices[0]);
        const double a = v0->estimate();
        const CurveFittingVertex* v1 = static_cast<const CurveFittingVertex*> (_vertices[1]);
        const double b = v1->estimate();
        const CurveFittingVertex* v2 = static_cast<const CurveFittingVertex*> (_vertices[2]);
        const double c = v2->estimate();

        double exp_y = std::exp( a*_x*_x + b*_x + c);
        Eigen::Matrix<double,1,1> tmp;
        tmp<<exp_y*_x*_x;
        _jacobianOplus[0] = tmp;
        tmp<<exp_y*_x;
        _jacobianOplus[1] = tmp;
        tmp<<exp_y;
        _jacobianOplus[2] = tmp;

    }

    // 存盘和读盘：留空,必须重写
    virtual bool read( istream& in ) {}
    virtual bool write( ostream& out ) const  {}
public:
    double _x;
};


int main( int argc, char** argv )
{
    double a=1.0, b=2.0, c=1.0;         // 真实参数值
    int N=100;                                    // 数据点
    double w_sigma=1.;                     // 噪声Sigma值

    //C++11的随机数生成器
    std::default_random_engine generator;
    std::normal_distribution<double> noise(0., w_sigma);


    // 每个误差项优化变量维度为3，误差值维度为1
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<3,1>> Block;
    // 第1步：创建一个线性求解器LinearSolver
    Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();
    // 第2步：创建BlockSolver。并用上面定义的线性求解器初始化
    Block* solver_ptr = new Block(linearSolver);
    // 第3步：创建总求解器solver。并从GN, LM, DogLeg 中选一个，再用上述块求解器BlockSolver初始化
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

    // 设置图模型
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    // 往图中增加顶点
    CurveFittingVertex* v0 = new CurveFittingVertex();
    v0->setEstimate( 0 );
    v0->setId(0);
    CurveFittingVertex* v1 = new CurveFittingVertex();
    v1->setEstimate( 0 );
    v1->setId(1);
    CurveFittingVertex* v2 = new CurveFittingVertex();
    v2->setEstimate( 0 );
    v2->setId(2);
    optimizer.addVertex( v0 );
    optimizer.addVertex( v1 );
    optimizer.addVertex( v2 );

    // 往图中增加边
    for ( int i=0; i<N; i++ )
    {
        // 注意这里不能给太大，否则算不出来,残差飞上天了
        double x = i / 100.;
        double n = noise(generator);
        double y = std::exp( a*x*x+b*x+c) + n;

        CurveFittingEdge3* edge = new CurveFittingEdge3( x );
        edge->setId(i);
        edge->setVertex( 0, v0 );                    // 设置连接的顶点,int值是其在vector<Vertex>中的位置
        edge->setVertex( 1, v1 );
        edge->setVertex( 2, v2 );
        edge->setMeasurement( y );              // 观测数值
        edge->setInformation( Eigen::Matrix<double,1,1>::Identity()); // 信息矩阵：协方差矩阵之逆
        optimizer.addEdge( edge );
    }

    // 执行优化
    cout<<"start optimization"<<endl;
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    optimizer.initializeOptimization();
    optimizer.optimize(60);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>( t2-t1 );
    cout<<"solve time cost = "<<1000*time_used.count()<<"ms. "<<endl;

    // 输出优化值
    double ae = v0->estimate();
    double be = v1->estimate();
    double ce = v2->estimate();
    cout<<"estimated model: "<<ae<<"  "<<be<<"  "<<ce<<"  "<<endl;

    // 查看其在Hessian矩阵中的位置
    int ha = v0->colInHessian();
    int hb = v1->colInHessian();
    int hc = v2->colInHessian();
    cout<<"Position in Hessian :"<<ha<<"  "<<hb<<"  "<<hc<<"  "<<endl;

    cout<<"ground truth: "<<a<<"   "<<b<<"   "<<c<<"   "<<endl;

    return 0;
}
