#include <ceres/ceres.h>
#include <ceres/solver.h>
#include <ceres/rotation.h>
#include "parse_urdf.h"
#include "glm/glm.hpp"
#define GLM_ENABLE_EXPERIMENTAL
#include "glm/gtx/quaternion.hpp"
# include <glm/gtx/string_cast.hpp>

template <typename T>
void QuaternionInverse(const T q[4], T q_inverse[4]) {
    q_inverse[0] = q[0];  // Real component
    q_inverse[1] = -q[1]; // Imaginary components negated
    q_inverse[2] = -q[2];
    q_inverse[3] = -q[3];
}

struct CostFunctor {

    CostFunctor(const std::vector<std::array<double, 3>>& axes,
                const std::vector<std::array<double, 4>>& quaternions,
                const std::vector<std::array<double, 3>>& positions,
                const double x[3],
                const double fi[4]
                ) : _axes(axes), _quaternions(quaternions), _positions(positions)
    {
        _x[0] = x[0];
        _x[1] = x[1];
        _x[2] = x[2];

        _fi[0] = fi[0];
        _fi[1] = fi[1];
        _fi[2] = fi[2];
        _fi[3] = fi[3];
    };

    template <typename T>
    bool operator()(const T* const thetas, T* residuals) const {

        auto transform = Eigen::Transform<T, 3, Eigen::Affine>::Identity();

        for (int i = 0; i < _axes.size(); i++)
        {
            Eigen::Vector<T, 3> p = Eigen::Vector<T, 3>(T(_positions[i][0]), T(_positions[i][1]), T(_positions[i][2]));

            Eigen::Quaternion<T> q = Eigen::Quaternion<T>(T(_quaternions[i][0]), T(_quaternions[i][1]), T(_quaternions[i][2]), T(_quaternions[i][3]));

            // apply translation
            transform.translate(p);

            // apply first rotation
            transform.rotate(q);

            Eigen::Vector<T, 3> axis = Eigen::Vector<T, 3>(T(_axes[i][0]), T(_axes[i][1]), T(_axes[i][2]));
            Eigen::AngleAxis<T> angle_axis(thetas[i], axis);
            // apply second rotation
            transform.rotate(angle_axis.toRotationMatrix());
        }
        Eigen::Vector<T, 3> p = transform.translation();

        residuals[0] = T(_x[0]) - p[0];
        residuals[1] = T(_x[1]) - p[1];
        residuals[2] = T(_x[2]) - p[2];

        //output the transform
        std::cout << "Transform:\n" << transform.matrix() << std::endl;

        // 1. Get calculated orientation
        Eigen::Quaternion<T> calculated_orientation(transform.rotation());

        Eigen::Quaternion<T> desired_orientation = Eigen::Quaternion<T>(T(_fi[0]), T(_fi[1]), T(_fi[2]), T(_fi[3]));

        // 2. Calculate orientation difference (error)
        Eigen::Quaternion<T> orientation_error = desired_orientation * calculated_orientation.inverse();

        // 3. Convert to Angle-Axis for residuals (optional)
        Eigen::AngleAxis<T> error_angle_axis(orientation_error);

        // 4. Fill residuals
        residuals[3] = error_angle_axis.angle() * error_angle_axis.axis()(0);
        residuals[4] = error_angle_axis.angle() * error_angle_axis.axis()(1);
        residuals[5] = error_angle_axis.angle() * error_angle_axis.axis()(2);
        residuals[6] = error_angle_axis.angle(); // (Optional: magnitude of the rotation)

        // Eigen::Matrix<T, 3, 3> rotation = transform.rotation();
        // residuals[3] = T(0) - T(rotation(0));
        // residuals[4] = T(0) - T(rotation(1));
        // residuals[5] = T(-1.0) - T(rotation(2));
        // residuals[6] = T(-1.0) - T(rotation(3));
        // residuals[7] = T(0) - T(rotation(4));
        // residuals[8] = T(0) - T(rotation(5));
        // residuals[9] = T(0) - T(rotation(6));
        // residuals[10] = T(0) - T(rotation(7));
        // residuals[11] = T(-1.0) - T(rotation(8));



        return true;
    }

    double _x[3];
    double _fi[4];
    std::vector<std::array<double, 3>> _axes;
    std::vector<std::array<double, 4>> _quaternions;
    std::vector<std::array<double, 3>> _positions;

};

void append_joint(std::vector<std::array<double, 3>>& axes, std::vector<std::array<double, 4>>& quaternions, std::vector<std::array<double, 3>>& positions, const urdf::JointConstSharedPtr& joint)
{
    if (joint == nullptr)
    {
        std::cout << "Joint is null" << std::endl;
    }
    const std::array<double, 3> axis = {joint->axis.x, joint->axis.y, joint->axis.z};

    const double rot_w = joint->parent_to_joint_origin_transform.rotation.w;
    const double rot_x = joint->parent_to_joint_origin_transform.rotation.x;
    const double rot_y = joint->parent_to_joint_origin_transform.rotation.y;
    const double rot_z = joint->parent_to_joint_origin_transform.rotation.z;
    const std::array<double, 4> quaternion = {rot_w, rot_x, rot_y, rot_z};

    const double x_pos = joint->parent_to_joint_origin_transform.position.x;
    const double y_pos = joint->parent_to_joint_origin_transform.position.y;
    const double z_pos = joint->parent_to_joint_origin_transform.position.z;
    const std::array<double, 3> position = {x_pos, y_pos, z_pos};

    axes.push_back(axis);
    quaternions.push_back(quaternion);
    positions.push_back(position);
}

void fill_joints(const urdf::LinkConstSharedPtr& link, std::vector<std::array<double, 3>>& axes, std::vector<std::array<double, 4>>& quaternions, std::vector<std::array<double, 3>>& positions)
{
    std::cout<< "Link: " << link->name << std::endl;
    if (!link->child_joints.empty())
    {
        for (const auto& joint : link->child_joints)
        {
            std::cout << joint->name << std::endl;
            append_joint(axes, quaternions, positions, joint);
        }
    }
    else
    {
        std::cout << "No child joints" << std::endl;
    }
    if (!link->child_links.empty())
    {
        for (const auto& child : link->child_links)
        {
            fill_joints(child, axes, quaternions, positions);
        }
    }
}

void ceres_executer()
{
    std::vector<std::array<double, 3>> axes;
    std::vector<std::array<double, 4>> quaternions;
    std::vector<std::array<double, 3>> positions;

    // ... set up your robot model ...
    urdf::ModelInterfaceSharedPtr robot;
    parse_urdf("../6_link.xml", robot);

    const auto root = robot->getRoot();

    //recursively go through the tree to append axes, quaternions, and positions
    fill_joints(root, axes, quaternions, positions);

    double mypi = glm::pi<double>();

    std::vector<double> angles = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> initial_angles = {mypi, -mypi/2, 0, 0,0 ,-mypi/2};
    const std::vector<double> target_pos = {-4.0, 0.0, 1.0};
    const std::vector<double> target_or = {0, 0.0, 1, 0};


    ceres::Problem problem;

    ceres::CostFunction* cost_function =
        new ceres::AutoDiffCostFunction<CostFunctor, 7, 6>(new CostFunctor(axes, quaternions, positions, target_pos.data(), target_or.data()));

    problem.AddResidualBlock(cost_function, nullptr, angles.data());

    problem.SetParameterLowerBound(angles.data(), 0, -3.14159);
    problem.SetParameterLowerBound(angles.data(), 1, -3.14159);
    problem.SetParameterLowerBound(angles.data(), 2, -3.14159);
    problem.SetParameterLowerBound(angles.data(), 3, -3.14159);
    problem.SetParameterLowerBound(angles.data(), 4, -3.14159);
    problem.SetParameterLowerBound(angles.data(), 5, -3.14159);

    problem.SetParameterUpperBound(angles.data(), 0, 3.14159);
    problem.SetParameterUpperBound(angles.data(), 1, 3.14159);
    problem.SetParameterUpperBound(angles.data(), 2, 3.14159);
    problem.SetParameterUpperBound(angles.data(), 3, 3.14159);
    problem.SetParameterUpperBound(angles.data(), 4, 3.14159);
    problem.SetParameterUpperBound(angles.data(), 5, 3.14159);


    ceres::Solver::Options options;
    // ... configure solver options ...
    // options.trust_region_strategy_type = ceres::DOGLEG;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    // options.function_tolerance = 1e-16;
    // options.parameter_tolerance = 1e-16;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // Print information or retrieve the optimized joint angles
    std::cout << summary.FullReport() << std::endl;

    for (const auto angle: angles)
    {
        std::cout << angle << std::endl;
    }

    glm::mat4 transform = glm::mat4(1.0f);

    for (int i = 0; i < axes.size(); i++)
    {
        glm::vec3 position = glm::vec3(positions[i][0], positions[i][1], positions[i][2]);
        transform = glm::translate(transform, position);

        glm::quat q = glm::quat(quaternions[i][0], quaternions[i][1], quaternions[i][2], quaternions[i][3]);
        transform = transform * glm::toMat4(q);

        glm::vec3 axis = glm::vec3(axes[i][0], axes[i][1], axes[i][2]);
        transform = glm::rotate(transform, (float)angles[i], axis);
    }


    for (int i = 0; i < 4; i++)
    {
        glm::vec4 row = transpose(transform)[i];
        std::cout << glm::to_string(row) << std::endl;
    }

    transform = glm::mat4(1.0f);

    for (int i = 0; i < axes.size(); i++)
    {
        glm::vec3 position = glm::vec3(positions[i][0], positions[i][1], positions[i][2]);
        transform = glm::translate(transform, position);

        glm::quat q = glm::quat(quaternions[i][0], quaternions[i][1], quaternions[i][2], quaternions[i][3]);
        transform = transform * glm::toMat4(q);

        glm::vec3 axis = glm::vec3(axes[i][0], axes[i][1], axes[i][2]);
        transform = glm::rotate(transform, (float)initial_angles[i], axis);
    }


    for (int i = 0; i < 4; i++)
    {
        glm::vec4 row = transpose(transform)[i];
        std::cout << glm::to_string(row) << std::endl;
    }
}

int main() {

    ceres_executer();

    return 0;
    }