cmake_minimum_required(VERSION 3.28)
project(testing)

set(CMAKE_CXX_STANDARD 23)

add_executable(testing main.cpp parse_urdf.cpp parse_urdf.h)

add_executable(6_link 6_link.cpp parse_urdf.cpp parse_urdf.h)

add_executable(testglm testglm.cpp parse_urdf.cpp parse_urdf.h)

add_executable(poseerror poseerror.cpp)

add_executable(kdltest kdl_test.cpp)

add_executable(checkeigen check_eigen.cpp)

add_executable(3_link 3_link.cpp parse_urdf.cpp parse_urdf.h)

find_package(Ceres CONFIG REQUIRED)
target_link_libraries(testing PRIVATE Ceres::ceres)
target_link_libraries(6_link PRIVATE Ceres::ceres)
target_link_libraries(3_link PRIVATE Ceres::ceres)
target_link_libraries(poseerror PRIVATE Ceres::ceres)

find_package(Eigen3 CONFIG REQUIRED)
target_link_libraries(testing PRIVATE Eigen3::Eigen)
target_link_libraries(testglm PRIVATE Eigen3::Eigen)
target_link_libraries(poseerror PRIVATE Eigen3::Eigen)
target_link_libraries(kdltest PRIVATE Eigen3::Eigen)
target_link_libraries(checkeigen PRIVATE Eigen3::Eigen)

find_package(urdfdom CONFIG REQUIRED)
target_link_libraries(testing PRIVATE urdfdom::urdf_parser urdfdom::urdfdom_model urdfdom::urdfdom_world urdfdom::urdfdom_sensor)
target_link_libraries(6_link PRIVATE urdfdom::urdf_parser urdfdom::urdfdom_model urdfdom::urdfdom_world urdfdom::urdfdom_sensor)
target_link_libraries(3_link PRIVATE urdfdom::urdf_parser urdfdom::urdfdom_model urdfdom::urdfdom_world urdfdom::urdfdom_sensor)
target_link_libraries(testglm PRIVATE urdfdom::urdf_parser urdfdom::urdfdom_model urdfdom::urdfdom_world urdfdom::urdfdom_sensor)

find_package(glm CONFIG REQUIRED)
target_link_libraries(testing PRIVATE glm::glm)
target_link_libraries(6_link PRIVATE glm::glm)
target_link_libraries(3_link PRIVATE glm::glm)
target_link_libraries(testglm PRIVATE glm::glm)

find_package(orocos_kdl CONFIG REQUIRED)
target_link_libraries(kdltest PRIVATE orocos-kdl)
