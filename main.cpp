#include <ceres/ceres.h>
#include <ceres/solver.h>
#include <ceres/rotation.h>
#include "parse_urdf.h"
#include "glm/glm.hpp"
#define GLM_ENABLE_EXPERIMENTAL
#include "glm/gtx/quaternion.hpp"
# include <glm/gtx/string_cast.hpp>

struct CostFunctor {

    CostFunctor(const std::vector<std::array<double, 3>>& axes,
                const std::vector<std::array<double, 4>>& quaternions,
                const std::vector<std::array<double, 3>>& positions,
                const double x[3]) :
    _axes(axes), _quaternions(quaternions), _positions(positions)
    {
        _x[0] = x[0];
        _x[1] = x[1];
        _x[2] = x[2];
    }

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

        T AngleAxis[3];

        for (int i = 0; i < 6; i++)
        {
            if (i==0)
            {
                p[0] = T(_positions[i][0]);
                p[1] = T(_positions[i][1]);
                p[2] = T(_positions[i][2]);
            }

            p_next[0] = T(_positions[i+1][0]);
            p_next[1] = T(_positions[i+1][1]);
            p_next[2] = T(_positions[i+1][2]);

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

            ceres::QuaternionRotatePoint(q_new, p_next, p_out);

            // write new quaternion to old
            q_old[0] = q_new[0];
            q_old[1] = q_new[1];
            q_old[2] = q_new[2];
            q_old[3] = q_new[3];

            // add position to old position
            p[0] += p_out[0];
            p[1] += p_out[1];
            p[2] += p_out[2];

            // break if last iteration
            if (i == 4)
            {
                break;
            }

        }

        residuals[0] = T(_x[0]) - p[0];
        residuals[1] = T(_x[1]) - p[1];
        residuals[2] = T(_x[2]) - p[2];

        return true;
    }

    double _x[3]{};
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
    parse_urdf("../urdf.xml", robot);

    const auto root = robot->getRoot();

    //recursively go through the tree to append axes, quaternions, and positions
    fill_joints(root, axes, quaternions, positions);

    std::vector<double> angles = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    const std::vector<double> target = {0.4, 0.4, 0.4};

    ceres::Problem problem;

    ceres::CostFunction* cost_function =
        new ceres::AutoDiffCostFunction<CostFunctor, 3, 6>(new CostFunctor(axes, quaternions, positions, target.data()));

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
    options.trust_region_strategy_type = ceres::DOGLEG;
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
}

int main() {

    ceres_executer();

    std::cout << "Hello, World!" << std::endl;


    return 0;
}