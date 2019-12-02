# 将当前目录中的所有源文件名称赋值给参数DIR_SRCS
# aux_source_directory(. DIR_SRCS)
add_executable(testCurveFitting CurveFitting.cpp)
# add_executable(testCurveFitting DIR_SRCS)
target_link_libraries(testCurveFitting ${PROJECT_NAME}_backend)
