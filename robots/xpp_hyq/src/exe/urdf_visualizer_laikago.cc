/********************************************************

@File urdf_visualizer_bot.cc

@Description To visualize Laikago robot in Rviz using URDF.

@Author  Eric Wang

@Date:   2020-06-30

@Association: Harbin Institute of Technology, Shenzhen.

*********************************************************/


#include <iostream>
#include <map>
#include <memory>
#include <string>

#include <ros/init.h>

#include <xpp_hyq/inverse_kinematics_laikago.h>
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

  auto quadruped_ik = std::make_shared<inverseKinematicsLaikago>();
  // Converts a cartesian robot representation to joint angles.
  CartesianJointConverter inv_kin_converter(quadruped_ik,
                        xpp_msgs::robot_state_desired, //! eric_wang: Subcribe desired robot state.
                        joint_desired); //! eric_wang: Publish converted joint state.

  // urdf joint names
//  int n_ee = hyq_ik->GetEECount();
  int n_ee = quadruped_ik->GetEECount();
  int n_j  = legJointCount;
  std::vector<UrdfVisualizer::URDFName> joint_names(n_ee*n_j);
  joint_names.at(n_j*LF + HAA) = "FL_hip_joint";
  joint_names.at(n_j*LF + HFE) = "FL_thigh_joint";
  joint_names.at(n_j*LF + KFE) = "FL_calf_joint";
  joint_names.at(n_j*RF + HAA) = "FR_hip_joint";
  joint_names.at(n_j*RF + HFE) = "FR_thigh_joint";
  joint_names.at(n_j*RF + KFE) = "FR_calf_joint";
  joint_names.at(n_j*LH + HAA) = "RL_hip_joint";
  joint_names.at(n_j*LH + HFE) = "RL_thigh_joint";
  joint_names.at(n_j*LH + KFE) = "RL_calf_joint";
  joint_names.at(n_j*RH + HAA) = "RR_hip_joint";
  joint_names.at(n_j*RH + HFE) = "RR_thigh_joint";
  joint_names.at(n_j*RH + KFE) = "RR_calf_joint";

  std::string urdf = "robot_description"; // rviz set param as robot_description.
  std::string base_name = "base";
  std::string rviz_fixed_frame = "world";


  //! eric_wang: Publish desired robot state to RVIZ by tf tree transform.
  UrdfVisualizer robot_state_desired(urdf, joint_names, base_name, rviz_fixed_frame,
                 joint_desired, "laikago_des"); //! eric_wang: Subcribe desired joint state from converter.

  ::ros::spin();

  return 1;
}

