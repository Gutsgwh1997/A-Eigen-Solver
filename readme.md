## 仅依赖Eigen实现的LM算法（以g2o为参照）．  
### 代码框架：
    appbackend
    ├── backend.h
    ├── CMakeLists.txt
    ├── edge.cc
    ├── edge.h
    ├── eigen_types.h
    ├── problem.cc
    ├── problem.h
    ├── vertex.cc
    └── vertex.h
    基础功能实现文件，后端优化的框架．其中一些函数有待完善，用TODO标记出．
    
    app
    ├── CMakeLists.txt
    ├── CurveFitting.cpp
    ├── NoLinearLeastSquares3.cpp
    └── NoLinearLeastSquares.cpp
    使用自构的优化框架实现的一个非线性最小二乘的例子，提供了两种编程思想．

    test
    ├── CMakeLists.txt
    ├── curvefitting3V.cpp
    ├── curvefitting.cpp
    ├── MatDecomposition.cpp
    └── TestEigen.cpp
    学习测试Eigen的矩阵分解；使用g2o实现了上述的最下二乘案例，两种编程思想．

    utils
    ├── CMakeLists.txt
    └── tic_toc.h
    工具集，同于计时．

    xmind
    └── 求解器(problem).xmind
    代码文档，思路整理
### 代码运行  
mkdir build  
cmake ..  
make -j8  
  
  
### 非线性问题
<center> <img src="/media/gwh/学习资料/研究生课题/VINS/从零开始手写VIO/03.第3节 基于优化的IMU预积分与视觉信息融合/problem.png" width="50%" height="50%" /></center>

