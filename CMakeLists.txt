cmake_minimum_required(VERSION 3.28)
project(testing)

set(CMAKE_CXX_STANDARD 23)

add_executable(testing main.cpp parse_urdf.cpp parse_urdf.h)

add_executable(test_quat_inv test_quat_inv.cpp parse_urdf.cpp parse_urdf.h)

add_executable(testglm testglm.cpp parse_urdf.cpp parse_urdf.h)

find_package(Ceres CONFIG REQUIRED)
target_link_libraries(testing PRIVATE Ceres::ceres)
target_link_libraries(test_quat_inv PRIVATE Ceres::ceres)

find_package(Eigen3 CONFIG REQUIRED)
target_link_libraries(testing PRIVATE Eigen3::Eigen)

find_package(urdfdom CONFIG REQUIRED)
target_link_libraries(testing PRIVATE urdfdom::urdf_parser urdfdom::urdfdom_model urdfdom::urdfdom_world urdfdom::urdfdom_sensor)
target_link_libraries(test_quat_inv PRIVATE urdfdom::urdf_parser urdfdom::urdfdom_model urdfdom::urdfdom_world urdfdom::urdfdom_sensor)
target_link_libraries(testglm PRIVATE urdfdom::urdf_parser urdfdom::urdfdom_model urdfdom::urdfdom_world urdfdom::urdfdom_sensor)

find_package(glm CONFIG REQUIRED)
target_link_libraries(testing PRIVATE glm::glm)
target_link_libraries(test_quat_inv PRIVATE glm::glm)
target_link_libraries(testglm PRIVATE glm::glm)