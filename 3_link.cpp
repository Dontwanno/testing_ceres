#include <ceres/ceres.h>
#include <ceres/solver.h>
#include <ceres/rotation.h>
#include "parse_urdf.h"

struct Pose3d {
    Eigen::Vector3d p;
    Eigen::Quaterniond q;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

template <typename T>
Eigen::Transform<T, 3, Eigen::Affine> CalculateFK (const std::vector<Eigen::Vector<T, 3>>& axes,
                                                   const std::vector<Eigen::Quaternion<T>>& quaternions,
                                                   const std::vector<Eigen::Vector<T, 3>>& positions,
                                                   const T* const thetas)
{
    Eigen::Transform<T, 3, Eigen::Affine> transform = Eigen::Transform<T, 3, Eigen::Affine>::Identity();

    for (int i = 0; i < axes.size(); i++)
    {

        // apply translation
        transform.translate(positions[i]);

        // apply first rotation
        transform.rotate(quaternions[i]);

        const Eigen::Vector<T, 3>& axis = axes[i];
        Eigen::AngleAxis<T> angle_axis(thetas[i], axis);
        // apply second rotation
        transform.rotate(angle_axis.toRotationMatrix());
    }

    return transform;
}


struct CostFunctor {
   CostFunctor(const std::vector<Eigen::Vector3d>& axes,
                  const std::vector<Eigen::Quaterniond>& quaternions,
                  const std::vector<Eigen::Vector3d>& positions,
                  const Pose3d& target_pose
                    ) : _axes(axes), _quaternions(quaternions), _positions(positions), _target_pose(target_pose) {};

        template <typename T>
        bool operator()(const T* const thetas, T* residuals_ptr) const {

        std::vector<Eigen::Vector<T, 3>> positions_T;
        std::vector<Eigen::Quaternion<T>> quaternions_T;
        std::vector<Eigen::Vector<T, 3>> axes_T;

        for (int i = 0; i < _axes.size(); i++)
        {
            axes_T.push_back(_axes[i].template cast<T>());
            quaternions_T.push_back(_quaternions[i].template cast<T>());
            positions_T.push_back(_positions[i].template cast<T>());
        }

        const Eigen::Transform<T, 3, Eigen::Affine> transform = CalculateFK(axes_T, quaternions_T, positions_T, thetas);

        Eigen::Quaternion<T> transform_quat(transform.rotation());

        // Compute the error between the desired orientation and the actual orientation.
        Eigen::Quaternion<T> delta_q = _target_pose.q.template cast<T>() * transform_quat.conjugate();

        Eigen::Vector<T, 3> p_estimated = transform.translation();

        // Compute the residuals.
        // [ position         ]   [ delta_p          ]
        // [ orientation (3x1)] = [ 2 * delta_q(0:2) ]
        Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals(residuals_ptr);
        residuals.template block<3, 1>(0, 0) =
            p_estimated - _target_pose.p.template cast<T>();
        residuals.template block<3, 1>(3, 0) = T(2.0) * delta_q.vec();

        return true;
    }

    Pose3d _target_pose;
    std::vector<Eigen::Vector3d> _axes;
    std::vector<Eigen::Quaterniond> _quaternions;
    std::vector<Eigen::Vector3d> _positions;

};

void append_joint(std::vector<Eigen::Vector3d>& axes, std::vector<Eigen::Quaterniond>& quaternions, std::vector<Eigen::Vector3d>& positions, const urdf::JointConstSharedPtr& joint)
{
    if (joint == nullptr)
    {
        std::cout << "Joint is null" << std::endl;
    }
    const  Eigen::Vector3d axis(joint->axis.x, joint->axis.y, joint->axis.z);

    const double rot_w = joint->parent_to_joint_origin_transform.rotation.w;
    const double rot_x = joint->parent_to_joint_origin_transform.rotation.x;
    const double rot_y = joint->parent_to_joint_origin_transform.rotation.y;
    const double rot_z = joint->parent_to_joint_origin_transform.rotation.z;
    const Eigen::Quaterniond quaternion(rot_w, rot_x, rot_y, rot_z);

    const double x_pos = joint->parent_to_joint_origin_transform.position.x;
    const double y_pos = joint->parent_to_joint_origin_transform.position.y;
    const double z_pos = joint->parent_to_joint_origin_transform.position.z;
    const Eigen::Vector3d position(x_pos, y_pos, z_pos);

    axes.push_back(axis);
    quaternions.push_back(quaternion);
    positions.push_back(position);
}

void fill_joints(const urdf::LinkConstSharedPtr& link, std::vector<Eigen::Vector3d>& axes, std::vector<Eigen::Quaterniond>& quaternions, std::vector<Eigen::Vector3d>& positions)
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

void PrintFKEigen(const std::vector<Eigen::Vector3d>& _axes,
                  const std::vector<Eigen::Quaterniond>& _quaternions,
                  const std::vector<Eigen::Vector3d>& _positions,
                const std::vector<double>& thetas)
{
    auto transform = Eigen::Transform<double, 3, Eigen::Affine>::Identity();

    for (int i = 0; i < _axes.size(); i++)
    {

        // apply translation
        transform.translate(_positions[i]);

        // apply first rotation
        transform.rotate(_quaternions[i]);

        Eigen::AngleAxisd angle_axis(thetas[i], _axes[i]);
        // apply second rotation
        transform.rotate(angle_axis.toRotationMatrix());
    }

    std::cout << "Transform: " << std::endl << transform.matrix() << std::endl;

    // print rotation as quaternion
    Eigen::Quaterniond q(transform.rotation());
    std::cout << "Quaternion: " << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << std::endl;

    // print quaternion magnitude
    std::cout << "Quaternion magnitude: " << q.norm() << std::endl;
}


std::vector<double> random_angles(int size)
{
    std::vector<double> angles;
    for (int i = 0; i < size; i++)
    {
        angles.push_back(static_cast<double>(std::rand()) / RAND_MAX * 2 * 3.14159 - 3.14159);
    }
    return angles;
}

void ceres_executer()
{
    srand((unsigned)time(NULL));

    std::vector<Eigen::Vector3d> axes;
    std::vector<Eigen::Quaterniond> quaternions;
    std::vector<Eigen::Vector3d> positions;

    // ... set up your robot model ...
    urdf::ModelInterfaceSharedPtr robot;
    parse_urdf("../6_link.xml", robot);

    const auto root = robot->getRoot();

    //recursively go through the tree to append axes, quaternions, and positions
    fill_joints(root, axes, quaternions, positions);

    double mypi = EIGEN_PI;

    std::vector<double> angles = random_angles(axes.size());

    for (double angle: angles)
    {
        std::cout << "inital angle: " << angle << std::endl;
    }

    std::vector<double> initial_angles = {mypi/2, 0, 0, 0, mypi/2, mypi/2};

    Pose3d target;
    target.p = Eigen::Vector3d(0, 1, 4);
    target.q = Eigen::Quaterniond(0, 0.707107, -0.707107, 0);

    ceres::Problem problem;

    ceres::CostFunction* new_cost_function =
        new ceres::AutoDiffCostFunction<CostFunctor, 6, 6>(new CostFunctor(axes, quaternions, positions, target));

    problem.AddResidualBlock(new_cost_function, nullptr, angles.data());

    for (int i = 0; i < axes.size(); i++)
    {
        problem.SetParameterLowerBound(angles.data(), i, -EIGEN_PI);
        problem.SetParameterUpperBound(angles.data(), i, EIGEN_PI);
    }

    ceres::Solver::Options options;
    // ... configure solver options ...
    // options.trust_region_strategy_type = ceres::DOGLEG;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    // options.function_tolerance = 1e-16;
    options.parameter_tolerance = 1e-16;

    // options.gradient_tolerance = 1e-16;
    // options.check_gradients = true;
    options.linear_solver_type = ceres::DENSE_QR;
    // options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 1000;


    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // Print information or retrieve the optimized joint angles
    std::cout << summary.FullReport() << std::endl;

    for (const auto angle: angles)
    {
        std::cout << angle << std::endl;
    }

    PrintFKEigen(axes, quaternions, positions, initial_angles);

    PrintFKEigen(axes, quaternions, positions, angles);
}

int main() {

    ceres_executer();

    return 0;
    }