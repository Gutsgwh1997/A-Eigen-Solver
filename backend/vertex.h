/**
 * @file vertex.h
 * @brief 仿照g2o::BaseVertex实现的简洁版本的vertex类.
 * @author VIO COURSE
 * @version 1.0
 * @date 2019-12-06
 */
#ifndef MYSLAM_BACKEND_VERTEX_H
#define MYSLAM_BACKEND_VERTEX_H

#include <backend/eigen_types.h>
// #include "backend/eigen_types.h"

namespace myslam {
namespace backend {


/**
* @brief g2o中需要在模板中指定顶点的类型＜维度，类型＞，参数的估计值类型是自己指定的
*        本实现中，只需要指定顶点的维度，类型是Eigen::VectorXd
*/
class Vertex {
   public:
    //当类，或者结构体中包含Eigen成员变量时
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    /**
     * @brief 构造函数
     *
     * @param num_dimension　　 变量的维度
     * @param local_dimensionh  线性化之后的维度,当为-1时(默认等于num_dimension)
     */
    explicit Vertex(int num_dimension, int local_dimension = -1);

    // 析构函数应该声明为虚函数，除非类不用做基类，可以保证正确的析构函数序列被调用
    virtual ~Vertex();

    /**
     * @name C++类成员函数后面接const的作用
     * @{ 实际上约束this指针为常指针，不能修改类的成员变量 */
    /**   表示该函数可以被常量对象调用，常量对象只能调用const函数 @} */

    /// 返回变量维度
    int Dimension() const { return parameters_.rows(); }

    /// 返回变量本地维度
    int LocalDimension() const { return local_dimension_; }

    /// 该顶点的id
    unsigned long Id() const { return id_; }

    /// 返回参数值(变量)
    VecX Parameters() const { return parameters_; }

    /// 返回参数值的引用
    VecX &Parameters() { return parameters_; }

    /// 设置参数值
    void SetParameters(const VecX &params) { parameters_ = params; }

    /// 返回顶点的名称，在子类中实现
    virtual std::string TypeInfo() const = 0;

    /// 加法，可重定义,虚函数实现的多态，根据类的类型调用对应的函数，不是根据指针的类型
    virtual void Plus(const VecX &delta);  //默认是向量加法(参数更新方式，旋转矩阵就是乘法)

    int OrderingId() const { return ordering_id_; }

    void SetOrderingId(unsigned long id) { ordering_id_ = id; };

    /// 固定该点的估计值
    void SetFixed(bool fixed = true) { fixed_ = fixed; }

    /// 测试该点是否被固定
    bool IsFixed() const { return fixed_; }

   protected:
    VecX parameters_;      // 实际存储的变量值, g2o中使用_estimate
    unsigned long id_;     // 顶点的id，自动生成

    /// ordering id是在problem中排序后的id，用于寻找雅可比对应块
    /// ordering id带有维度信息，例如ordering_id=6则对应Hessian中的第6列
    /// 从零开始, 指出hessian矩阵的对应块
    unsigned long ordering_id_ = 0;    //在g2o中叫做_colInHessian,也可以自己设定,此代码没有自动设定
    int local_dimension_;              // 局部参数化维度

    bool fixed_ = false;               // 是否固定
};
}
}

#endif
