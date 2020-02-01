#include <iostream>
#include <g2o/core/base_vertex.h>
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
class CurveFittingVertex: public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // 重置估计值，纯虚函数，必须重写
    virtual void setToOriginImpl() override // 重置
    {
        _estimate << 0,0,0;
    }
    
    //纯虚函数中定义了参数的类型
    virtual void oplusImpl( const double* update ) override // 更新
    {
        _estimate += Eigen::Vector3d(update);
    }

    // 存盘和读盘：留空,必须重写
    virtual bool read( istream& in ) {}
    virtual bool write( ostream& out ) const  {}
};

// 误差模型 模板参数：观测值维度，类型，连接顶点类型
class CurveFittingEdge: public g2o::BaseUnaryEdge<1,double,CurveFittingVertex>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CurveFittingEdge( double x ): BaseUnaryEdge(), _x(x) {}

    // 计算曲线模型误差
    void computeError() override
    {
        // 将基类的指针向下强制转换为派生类的指针，为什么不用dynamic_cast??，RTTI只能用于包含虚函数的类层次
        // const CurveFittingVertex* v = static_cast<const CurveFittingVertex*> (_vertices[0]);
        const CurveFittingVertex* v = dynamic_cast<const CurveFittingVertex*> (_vertices[0]);
        const Eigen::Vector3d abc = v->estimate();
        //_error是一个Eigen::Matrix<>,需要（a,b）这样写
        _error(0,0) = std::exp( abc(0,0)*_x*_x + abc(1,0)*_x + abc(2,0) )- _measurement;
    }

    // 自己提供雅克比矩阵,可以不提供，默认数值求导
    void linearizeOplus() override
    {
        // const CurveFittingVertex* v = static_cast<const CurveFittingVertex*> (_vertices[0]);
        const CurveFittingVertex* v = dynamic_cast<const CurveFittingVertex*> (_vertices[0]);
        const Eigen::Vector3d abc = v->estimate();
        double exp_y = std::exp( abc(0,0)*_x*_x + abc(1,0)*_x + abc(2,0) );

        Eigen::Matrix<double, 1, 3> jaco_abc;  // 误差为1维，状态量 3 个，所以是 1x3 的雅克比矩阵
        jaco_abc << _x*_x*exp_y, _x*exp_y, exp_y;
        _jacobianOplusXi = jaco_abc;
    }
    virtual bool read( istream& in ) {}
    virtual bool write( ostream& out ) const {}
public:
    double _x;  // x 值， y 值为 _measurement
};

int main( int argc, char** argv )
{
    double a=1.0, b=2.0, c=1.0;         // 真实参数值
    int N=100;                          // 数据点
    double w_sigma=1.;                 // 噪声Sigma值

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
    CurveFittingVertex* v = new CurveFittingVertex();
    v->setEstimate( Eigen::Vector3d(0,0,0) );
    v->setId(0);
    optimizer.addVertex( v );
    
    // 往图中增加边
    for ( int i=0; i<N; i++ )
    {
        // 注意这里不能给太大，否则算不出来,残查飞上天了
        double x = i/100.;
        double n = noise(generator);
        double y = std::exp( a*x*x+b*x+c) + n;

        CurveFittingEdge* edge = new CurveFittingEdge( x );
        edge->setId(i);
        edge->setVertex( 0, v );                // 设置连接的顶点
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
    Eigen::Vector3d abc_estimate = v->estimate();
    cout<<"estimated model: "<<abc_estimate.transpose()<<endl;
    cout<<"ground truth: "<<a<<"   "<<b<<"   "<<c<<"   "<<endl;
    
    return 0;
}
