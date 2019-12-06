/**
 * @file edge.h
 * @brief 仿照g2o的g2o::BaseMultiEdge类改写的简洁版本
 * @author VIO COURSE
 * @version 1.0
 * @date 2019-12-06
 */
#ifndef MYSLAM_BACKEND_EDGE_H
#define MYSLAM_BACKEND_EDGE_H

#include <memory>
#include <string>
#include "backend/eigen_types.h"

namespace myslam {
namespace backend {

class Vertex;

/**
 * 边负责计算残差，残差一般是预测-观测得到的一个向量，维度在构造函数中定义.
 * g2o中的一元边连接一个顶点（一个变量），多元边可以连接多个顶点
 * 代价函数一般是一个数值，由 残差*信息*残差(某种表示，一般是范数,信息矩阵等价于加权的最小二乘)，后端求和后最小化
 */
class Edge {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    /**
     * 构造函数，会自动分配_residual和jacobian的vector的大小(很重要)，在g2o中需要在父类的构造函数中resize()
     * @param residual_dimension 残差维度
     * @param num_verticies 顶点数量,优化变量的数目
     * @param verticies_types 顶点类型名称，可以不给，不给的话check中不会检查
     */
    explicit Edge(int residual_dimension, int num_verticies, const std::vector<std::string> &verticies_types = std::vector<std::string>());

    virtual ~Edge();

    /// 返回id
    unsigned long Id() const { return id_; }

    /**
     * 设置(添加)一个顶点
     * @param vertex 对应的vertex对象
     */
    bool AddVertex(const std::shared_ptr<Vertex> &vertex) {
        //push_back会调用构造和转移构造函数
        //emplace_back不需要触发copy和转移构造函数
        verticies_.emplace_back(vertex);
        return true;
    }

    /**
     * 设置一些顶点
     * @param vertices 顶点，按引用顺序排列
     * @return
     */
    bool SetVertex(const std::vector<std::shared_ptr<Vertex>> &vertices) {
        verticies_ = vertices;
        return true;
    }

    /// 返回第i个顶点
    std::shared_ptr<Vertex> GetVertex(int i) const {
        return verticies_[i];
    }

    /// 返回所有顶点
    std::vector<std::shared_ptr<Vertex>> Verticies() const {
        return verticies_;
    }

    /// 返回关联顶点个数
    size_t NumVertices() const { return verticies_.size(); }

    /// 返回边的类型信息，在子类中实现
    virtual std::string TypeInfo() const = 0;

    ///代价函数 ，由子类实现,一般是一个实数,残差*信息矩阵*残差
    virtual void ComputeResidual() = 0;

    /**
     * @brief 计算雅克比矩阵,维度代价函数*顶点数目．由子类实现
     * @brief 不支持自动求导
     */
    virtual void ComputeJacobians() = 0;

   ///计算该edge对Hession矩阵的影响，由子类实现
   // virtual void ComputeHessionFactor() = 0;

    /// 计算平方误差，会乘以信息矩阵
    double Chi2();

    /// 返回残差
    VecX Residual() const { return residual_; }

    /// 返回雅可比
    std::vector<MatXX> Jacobians() const { return jacobians_; }

    /// 设置信息矩阵, information_ = sqrt_Omega = w
    void SetInformation(const MatXX &information) {
        information_ = information;
    }

    /// 返回信息矩阵
    MatXX Information() const {
        return information_;
    }

    /// 设置观测信息
    void SetObservation(const VecX &observation) {
        observation_ = observation;
    }

    /// 返回观测信息
    VecX Observation() const { return observation_; }

    /// 检查边的信息是否全部设置
    bool CheckValid();

    int OrderingId() const { return ordering_id_; }

    void SetOrderingId(int id) { ordering_id_ = id; };

protected:
    unsigned long id_;  // edge id
    int ordering_id_;   //edge id in problem,用于寻找雅克比对应块?
    std::vector<std::string> verticies_types_;        // 各顶点类型信息，用于debug
    std::vector<std::shared_ptr<Vertex>> verticies_;  // 该边对应的顶点
    VecX residual_;                 // 残差,g2o中用_error表示
    std::vector<MatXX> jacobians_;  // 雅可比，每个雅可比维度是 residual x vertex[i].local_dimension_
    MatXX information_;             // 信息矩阵,维度residual_dimension*residual_~可用于加权最小二乘
    VecX observation_;              // 观测信息
};

}
}
#endif
