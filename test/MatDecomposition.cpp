#include<iostream>
#include<Eigen/Dense>

using namespace std;
using namespace Eigen;

int main(int argc, char** argv)
{
    Matrix3d A;
    Vector3d b;
    A<<1,2,3,  4,5,6,  7,8,10;
    b<<3,3,4;
    cout<<"Here is the matrix A: \n"<<A<<endl;
    cout<<"Here is the vector b: \n"<<b<<endl;

    // QR分解,要求A是满秩的
    // colPivHouseholderQr()返回类colPivHouseholderQr()的对象
    // Vector3d x = A.colPivHouseholderQr().solve(b);
    //此代码等价于
    ColPivHouseholderQR<Matrix3d> dec(A);
    Vector3d x = dec.solve(b);
    cout<<"The solution is : \n"<<x<<endl;

    return 0;
}
