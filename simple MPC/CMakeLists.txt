cmake_minimum_required(VERSION 3.10)


project(MPC_Path_Tracking)


set(CMAKE_CXX_STANDARD 11)


find_package(Eigen3 REQUIRED)
find_package(osqp REQUIRED)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR} ${OSQP_INCLUDE_DIRS})

add_executable(mpc_path_tracking mpc.cpp)

target_link_libraries(mpc_path_tracking
osqp
OsqpEigen
)



