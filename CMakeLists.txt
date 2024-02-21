cmake_minimum_required(VERSION 3.28)
project(testing)

set(CMAKE_CXX_STANDARD 23)


add_executable(testing main.cpp parse_urdf.cpp parse_urdf.h)

add_executable(numerical ceres_numeric_test.cpp)

add_executable(reference reference.cpp)

find_package(Ceres CONFIG REQUIRED)
target_link_libraries(testing PRIVATE Ceres::ceres)
target_link_libraries(numerical PRIVATE Ceres::ceres)
target_link_libraries(reference PRIVATE Ceres::ceres)

find_package(Eigen3 CONFIG REQUIRED)
target_link_libraries(testing PRIVATE Eigen3::Eigen)

find_package(urdfdom CONFIG REQUIRED)
target_link_libraries(testing PRIVATE urdfdom::urdf_parser urdfdom::urdfdom_model urdfdom::urdfdom_world urdfdom::urdfdom_sensor)

find_package(glm CONFIG REQUIRED)
target_link_libraries(testing PRIVATE glm::glm)