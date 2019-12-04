#include "backend/vertex.h"
#include <iostream>

namespace myslam {
namespace backend {

unsigned long global_vertex_id = 0;

Vertex::Vertex(int num_dimension, int local_dimension) {
    parameters_.resize(num_dimension, 1);
    //一般情况下num_dimension＝local_dimension相等．取决于如何参数化，例如VINS中有一节
    local_dimension_ = local_dimension > 0 ? local_dimension : num_dimension;
    id_ = global_vertex_id++;

    //    std::cout << "Vertex construct num_dimension: " << num_dimension
    //              << " local_dimension: " << local_dimension << " id_: " <<
    //              id_ << std::endl;
}

Vertex::~Vertex() {}

void Vertex::Plus(const VecX &delta) { parameters_ += delta; }

}
}

