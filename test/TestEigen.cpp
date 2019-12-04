#include <Eigen/Core>
#include <iostream>

int main() {

    typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatXX;
    MatXX error_prior;

    Eigen::Matrix<double,3,3> cc;
    cc<<1,2,3,  4,5,6,  7,8,9;

    std::cout<<"error_prior dimension is "<<error_prior.rows()<<std::endl;
    std::cout<<"cc dimension is "<<cc.rows()<<std::endl;

    MatXX I(MatXX::Identity(5,5));
    std::cout<<"MatXX::Identity(5,5) is "<<std::endl;
    std::cout<<I<<std::endl;

    std::cout<<"5*MatXX::Identity(5,5) is "<<std::endl;
    std::cout<<5*I<<std::endl;


    return 0;
}
