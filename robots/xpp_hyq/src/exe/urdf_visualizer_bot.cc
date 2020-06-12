/********************************************************

@File urdf_visualizer_bot.cc

@Description To visualize quadruped robot in Rviz using URDF.

@Author  Eric Wang

@Date:   2020-01-10

@Association: Harbin Institute of Technology, Shenzhen.

*********************************************************/


#include <iostream>
#include <map>
#include <memory>
#include <string>

#include <ros/init.h>

#include <xpp_hyq/inverse_kinematics_bot.h>
#include <xpp_msgs/topic_names.h>
#include <xpp_states/joints.h>
#include <xpp_states/endeffector_mappings.h>

#include <xpp_vis/cartesian_joint_converter.h>
#include <xpp_vis/urdf_visualizer.h>

using namespace xpp;
using namespace quad;

int main(int argc, char *argv[])
{
  ::ros::init(argc, argv, "quadruped_urdf_visualizer");

  const std::string joint_desired = "xpp/joint_quadruped_des"; // state topic to subsribe.

  auto quadruped_ik = std::make_shared<inverseKinematicsBot>();
  // Converts a cartesian robot representation to joint angles.
  CartesianJointConverter inv_kin_converter(quadruped_ik,
                        xpp_msgs::robot_state_desired, //! eric_wang: Subcribe desired robot state.
                        joint_desired); //! eric_wang: Publish converted joint state.

  // urdf joint names
//  int n_ee = hyq_ik->GetEECount();
  int n_ee = quadruped_ik->GetEECount();
  int n_j  = legJointCount;
  std::vector<UrdfVisualizer::URDFName> joint_names(n_ee*n_j);
  joint_names.at(n_j*LF + HAA) = "front_left_1_joint";
  joint_names.at(n_j*LF + HFE) = "front_left_2_joint";
  joint_names.at(n_j*LF + KFE) = "front_left_3_joint";
  joint_names.at(n_j*RF + HAA) = "front_right_1_joint";
  joint_names.at(n_j*RF + HFE) = "front_right_2_joint";
  joint_names.at(n_j*RF + KFE) = "front_right_3_joint";
  joint_names.at(n_j*LH + HAA) = "rear_left_1_joint";
  joint_names.at(n_j*LH + HFE) = "rear_left_2_joint";
  joint_names.at(n_j*LH + KFE) = "rear_left_3_joint";
  joint_names.at(n_j*RH + HAA) = "rear_right_1_joint";
  joint_names.at(n_j*RH + HFE) = "rear_right_2_joint";
  joint_names.at(n_j*RH + KFE) = "rear_right_3_joint";

  std::string urdf = "robot_description"; // rviz set param as robot_description.
  std::string base_name = "base_link";
  std::string rviz_fixed_frame = "world";


  //! eric_wang: Publish desired robot state to RVIZ by tf tree transform.
  UrdfVisualizer robot_state_desired(urdf, joint_names, base_name, rviz_fixed_frame,
                 joint_desired, "quadruped_des"); //! eric_wang: Subcribe desired joint state from converter.

  ::ros::spin();

  return 1;
}

