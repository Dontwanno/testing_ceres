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

        residuals[0] *= 10;
        residuals[1] *= 10;
        residuals[2] *= 10;

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

        return true;
    }

    double _x[3];
    double _fi[4];
    std::vector<std::array<double, 3>> _axes;
    std::vector<std::array<double, 4>> _quaternions;
    std::vector<std::array<double, 3>> _positions;

};

struct old_CostFunctor {

    old_CostFunctor(const std::vector<std::array<double, 3>>& axes,
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
    bool operator()(const T* thetas, T* residuals) const {
        T p[3];
        T p_out[3];
        T p_next[3];

        T q1[4];
        T q2[4];
        T q_temp[4];
        T q_old[4] = {T(1.0), T(0.0), T(0.0), T(0.0)};
        T q_new[4];
        T q_inv[4];
        T q_error[4];

        T testmat[9];

        T AngleAxis[3];

        for (int i = 0; i < _axes.size(); i++)
        {
            if (i==0)
            {
                p[0] = T(_positions[i][0]);
                p[1] = T(_positions[i][1]);
                p[2] = T(_positions[i][2]);
            }

            // q1 = quaternions[i];
            q1[0] = T(_quaternions[i][0]);
            q1[1] = T(_quaternions[i][1]);
            q1[2] = T(_quaternions[i][2]);
            q1[3] = T(_quaternions[i][3]);

            AngleAxis[0] = _axes[i][0] * thetas[i];
            AngleAxis[1] = _axes[i][1] * thetas[i];
            AngleAxis[2] = _axes[i][2] * thetas[i];

            ceres::AngleAxisToQuaternion(AngleAxis, q2);

            ceres::QuaternionProduct(q1, q2, q_temp);
            ceres::QuaternionProduct(q_old, q_temp, q_new);

            // write new quaternion to old
            q_old[0] = q_new[0];
            q_old[1] = q_new[1];
            q_old[2] = q_new[2];
            q_old[3] = q_new[3];

            // break if last iteration
            if (i < _axes.size()-1)
            {
                p_next[0] = T(_positions[i+1][0]);
                p_next[1] = T(_positions[i+1][1]);
                p_next[2] = T(_positions[i+1][2]);

                ceres::QuaternionRotatePoint(q_new, p_next, p_out);

                // add position to old position
                p[0] += p_out[0];
                p[1] += p_out[1];
                p[2] += p_out[2];
            }
            else
            {
                p_next[0] = T(0.0);
                p_next[1] = T(0.0);
                p_next[2] = T(1.0);

                ceres::QuaternionRotatePoint(q_new, p_next, p_out);

                p[0] += p_out[0];
                p[1] += p_out[1];
                p[2] += p_out[2];
            }

        }

        residuals[0] = T(_x[0]) - p[0];
        residuals[1] = T(_x[1]) - p[1];
        residuals[2] = T(_x[2]) - p[2];

        T q_target[4] = {(T)_fi[0], (T)_fi[1], (T)_fi[2], (T)_fi[3]};

        QuaternionInverse(q_new, q_inv);

        ceres::QuaternionProduct(q_inv, q_target, q_error);

        T q_error_inverse[4];
        // QuaternionInverse(q_error, q_error_inverse);

        residuals[3] = q_error[0];
        residuals[4] = q_error[1];
        residuals[5] = q_error[2];
        residuals[6] = q_error[3];

        std::cout << "p error: " << residuals[0] << " " << residuals[1] << " " << residuals[2] << std::endl;

        std::cout << "q_target: " << q_target[0] << " " << q_target[1] << " " << q_target[2] << " " << q_target[3] << std::endl;
        std::cout << "qnew: " << q_new[0] << " " << q_new[1] << " " << q_new[2] << " " << q_new[3] << std::endl;
        std::cout << "qerror: " << q_error[0] << " " << q_error[1] << " " << q_error[2] << " " << q_error[3] << std::endl;

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
    parse_urdf("../3_link.xml", robot);

    const auto root = robot->getRoot();

    //recursively go through the tree to append axes, quaternions, and positions
    fill_joints(root, axes, quaternions, positions);

    double mypi = glm::pi<double>();

    std::vector<double> angles = {0.0, 0.0, 0.0, 0.0, 0.0, -mypi/2};
    std::vector<double> initial_angles = {0.0, 0.0, 0.0, 0.0, 0.0, mypi/2};
    const std::vector<double> target_pos = {-1.0, 0.0, 5.0};
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