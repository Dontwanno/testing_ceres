#include "parse_urdf.h"
#include "glm/glm.hpp"
#define GLM_ENABLE_EXPERIMENTAL
#include "glm/gtx/quaternion.hpp"
# include <glm/gtx/string_cast.hpp>
#include <array>
#include <vector>
#include <Eigen/Dense>


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

void printmat(glm::mat4 const& mat)
{
    for (int i = 0; i < 4; i++)
    {
        glm::vec4 row = glm::transpose(mat)[i];
        std::cout << glm::to_string(row) << std::endl;
    }
}

void eigencalc(const std::vector<std::array<double, 3>>& _axes,
                const std::vector<std::array<double, 4>>& _quaternions,
                const std::vector<std::array<double, 3>>& _positions,
                const std::vector<double>& thetas)
{
    auto transform = Eigen::Transform<double, 3, Eigen::Affine>::Identity();

    for (int i = 0; i < _axes.size(); i++)
    {
        Eigen::Vector<double, 3> p = Eigen::Vector<double, 3>(double(_positions[i][0]), double(_positions[i][1]), double(_positions[i][2]));

        Eigen::Quaternion<double> q = Eigen::Quaternion<double>(double(_quaternions[i][0]), double(_quaternions[i][1]), double(_quaternions[i][2]), double(_quaternions[i][3]));

        // apply translation
        transform.translate(p);

        // apply first rotation
        transform.rotate(q);

        Eigen::Vector<double, 3> axis = Eigen::Vector<double, 3>(double(_axes[i][0]), double(_axes[i][1]), double(_axes[i][2]));
        Eigen::AngleAxis<double> angle_axis(thetas[i], axis);
        // apply second rotation
        transform.rotate(angle_axis.toRotationMatrix());
    }
    Eigen::Vector<double, 3> p = transform.translation();
    auto rotation = transform.rotation();
    std::cout << "rotation:" << rotation(0) << " " << rotation(1) << " " << rotation(2) << std::endl;
    std::cout << "rotation2: " << rotation(3) << " " << rotation(4) << " " << rotation(5) << std::endl;
    std::cout << "rotation3: " << rotation(6) << " " << rotation(7) << " " << rotation(8) << std::endl;
    std::cout << "Transform: " << std::endl << transform.matrix() << std::endl;
}



int main(int argc, char* argv[])
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


    double pi = glm::pi<double>();
    std::vector<double> angles = {pi/2, 0, 0, 0, -pi/2, -pi/2};

    glm::mat4 transform = glm::mat4(1.0f);

    for (int i = 0; i < axes.size(); i++)
    {
        glm::vec3 position = glm::vec3(positions[i][0], positions[i][1], positions[i][2]);
        transform = glm::translate(transform, position);

        glm::quat q = glm::quat(quaternions[i][0], quaternions[i][1], quaternions[i][2], quaternions[i][3]);
        std::cout << glm::to_string(q) << std::endl;
        transform = transform * glm::toMat4(q);

        glm::vec3 axis = glm::vec3(axes[i][0], axes[i][1], axes[i][2]);
        transform = glm::rotate(transform, (float)angles[i], axis);
    }

    printmat(transform);

    glm::vec3 link = glm::vec3(0, 0, 1);

    glm::vec3 transformed_link = glm::vec3(transform * glm::vec4(link, 1.0f));

    std::cout << "transformed link" << glm::to_string(transformed_link) << std::endl;


    eigencalc(axes, quaternions, positions, angles);

    return 0;
}
