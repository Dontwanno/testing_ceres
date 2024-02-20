#include <ceres/ceres.h>
#include <ceres/solver.h>
#include "parse_urdf.h"
#include <glm/glm.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <ranges>
#include <glm/gtx/string_cast.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtx/quaternion.hpp>


std::vector<glm::dvec3> axes;
std::vector<glm::dquat> orientations;
std::vector<glm::dvec3> positions;
std::vector<glm::dmat4> RotationMatrices;

glm::mat4 CalculateFK(const double* joint_angles,
                      const std::vector<glm::dvec3> &axes = axes,
                      const std::vector<glm::dmat4> &RotationMatrices = RotationMatrices,
                      const std::vector<glm::dvec3>& positions = positions)
{
    glm::dmat4 transform = glm::dmat4(1.0f);

    for (int i = 0; i < axes.size(); i++)
    {
        transform = glm::translate(transform, positions[i]);
        transform = transform * RotationMatrices[i];
        transform = glm::rotate(transform, joint_angles[i], axes[i]);

    }

    return transform;
}


struct CalculateFKValueFunctor {
    bool operator()(const double* r2, double* value) const {
        glm::mat4 mat = CalculateFK(r2);
        value[0] =
        return true;
    }
};

struct CostFunctor {
    CostFunctor(const double x_in[2], const double y_in[2]) {
        x[0] = x_in[0];
        x[1] = x_in[1];
        y[0] = y_in[0];
        y[1] = y_in[1];

        ceres_compute_fk = std::make_unique<ceres::CostFunctionToFunctor<1, 1>>(
          std::make_unique<ceres::NumericDiffCostFunction<
                CalculateFKValueFunctor
              , ceres::CENTRAL, 3, 6
            >
          >()
        );
    }



    template <typename T>
    bool operator()(const T* const angles, T* residual) const
    {
        // Your forward kinematics calculations using doubles here
        auto m = glm::dmat4(1.0f);
        // for (int i = 0; i < axes.size(); i++)
        // {
        //     m = glm::translate(m, positions[i]);
        //     m = m * RotationMatrices[i];
        //     m = glm::rotate(m, angles[i], glm::vec3<T>(axes[i]));
        //
        // }

        m = CalculateFK(angles);

        glm::dvec3 vector = glm::dvec3(m[3]);
        residual[0] = T(0.4 - vector[0]);
        residual[1] = T(0.4 - vector[1]);
        residual[2] = T(0.4 - vector[2]);
        return true;
    }

    std::unique_ptr<ceres::CostFunctionToFunctor<1, 1>> ceres_compute_fk;

};


void append_joint(std::vector<glm::dvec3>& axes, std::vector<glm::dquat>& orientations, std::vector<glm::dvec3>& positions, urdf::JointConstSharedPtr joint)
{
    if (joint == nullptr)
    {
        std::cout << "Joint is null" << std::endl;
    }
    double x = joint->axis.x; double y = joint->axis.y; double z = joint->axis.z;
    const auto axis        = glm::dvec3(x, y, z);

    double rot_w = joint->parent_to_joint_origin_transform.rotation.w;
    double rot_x = joint->parent_to_joint_origin_transform.rotation.x;
    double rot_y = joint->parent_to_joint_origin_transform.rotation.y;
    double rot_z = joint->parent_to_joint_origin_transform.rotation.z;
    const auto quaternion  = glm::dquat(rot_w, rot_x, rot_y, rot_z);

    double x_pos = joint->parent_to_joint_origin_transform.position.x;
    double y_pos = joint->parent_to_joint_origin_transform.position.y;
    double z_pos = joint->parent_to_joint_origin_transform.position.z;
    const auto position    = glm::dvec3(x_pos, y_pos, z_pos);

    axes.push_back(axis);
    orientations.push_back(quaternion);
    positions.push_back(position);
}

void fill_joints(urdf::LinkConstSharedPtr link, std::vector<glm::dvec3>& axes, std::vector<glm::dquat>& orientations, std::vector<glm::dvec3>& positions)
{
    std::cout<< "Link: " << link->name << std::endl;
    if (link->child_joints.size() > 0)
    {
        for (auto joint : link->child_joints)
        {
            std::cout << joint->name << std::endl;
            append_joint(axes, orientations, positions, joint);
        }
    }
    else
    {
        std::cout << "No child joints" << std::endl;
    }
    if (link->child_links.size() > 0)
    {
        for (auto child : link->child_links)
        {
            fill_joints(child, axes, orientations, positions);
        }
    }
}

void printmat4(glm::mat4 mat)
{
    std::cout << "Matrix: " << std::endl;
    for (int i = 0; i < 4; i++)
    {
        std::cout << glm::to_string(mat[i]) << std::endl;
    }
}

int main() {

    // ... set up your robot model ...
    urdf::ModelInterfaceSharedPtr robot;
    parse_urdf("../urdf.xml", robot);

    auto root = robot->getRoot();

    //recursively go through the tree to append axes, orientations, and positions
    fill_joints(root, axes, orientations, positions);

    for (auto quat : orientations)
    {
        RotationMatrices.push_back(glm::toMat4(quat));
    }

    glm::vec3 target_position = glm::vec3(0.4f, 0.4f, 0.4f);

    ceres::Problem problem;

    CostFunctor functor = CostFunctor();

    ceres::CostFunction* cost_function =
        new ceres::AutoDiffCostFunction<CostFunctor, 3, 6>(&functor);

    std::vector<double> x = {2.32520371, 0.68823409, 0.79681649, 0.33239085, 0.84276185, 0.78539816};

    glm::mat4 transform = calculateFK(x.data(), axes, RotationMatrices, positions);
    std::cout << "Initial position: " << glm::to_string(glm::vec3(transform[3])) << std::endl;

    problem.AddResidualBlock(cost_function, nullptr, x.data());

    ceres::Solver::Options options;
    // ... configure solver options ...
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // Print information or retrieve the optimized joint angles
    std::cout << summary.FullReport() << std::endl;

    for (auto angle: x)
    {
        std::cout << angle << std::endl;
    }

    return 0;
}