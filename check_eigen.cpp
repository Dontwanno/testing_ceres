#include <iostream>
#include <Eigen/Dense>

int main()
{
    Eigen::Quaternion<double> desired_orientation = Eigen::Quaternion<double>(0, 0, 1, 0);
    Eigen::Quaternion<double> current_orientation = Eigen::Quaternion<double>(1, 0, 0, 0);

    Eigen::Quaternion<double> error = desired_orientation * current_orientation.inverse();

    auto mat = error.toRotationMatrix();

    // print the matrix
    std::cout << mat << std::endl;

    std::cout << error.w() << " " << error.x() << " " << error.y() << " " << error.z() << std::endl;

    return 0;
}