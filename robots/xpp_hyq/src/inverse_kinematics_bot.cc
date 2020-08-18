/********************************************************

@File Inverse_kinematics_bot.

@Description Inverse kinematics of the quadruped robot.

@Author  Eric Wang

@Date:   2020-01-11

@Association: Harbin Institute of Technology, Shenzhen.

*********************************************************/


#include <xpp_hyq/inverse_kinematics_bot.h>
#include <xpp_states/cartesian_declarations.h>
#include <xpp_states/endeffector_mappings.h>
//#include <xpp_hyq/leg_inverse_kinematics.h>
#include <xpp_hyq/leg_inverse_kinematics_bot.h>

#include <ros/ros.h>

namespace xpp {

inverseKinematicsBot::inverseKinematicsBot()
{
    leg_ik.initializeKinematics();
    ROS_INFO("Quadruped inverse kinematics");
}

inverseKinematicsBot::~inverseKinematicsBot()
{

}

Joints
inverseKinematicsBot::GetAllJointAngles(const EndeffectorsPos& eep_B)
{
//  Vector3d ee_pos_H; // foothold expressed in hip frame
  Vector3d ee_pos_B; // foothold exoressed in base frame
  std::vector<Eigen::VectorXd> q_vec;

  std::string limbType_; // leg configuration
  LimbEnum limb_;
  // make sure always exactly 4 elements
  auto pos_B = eep_B.ToImpl();// foothold position in base frame.
  pos_B.resize(4, pos_B.front());

  for (int ee=0; ee<pos_B.size(); ++ee) {

//    legInverseKinematics::KneeBend bend = legInverseKinematics::Forward;// default direction.
    legInverseKinematicsX::KneeBend bend = legInverseKinematicsX::Forward;
    using namespace quad;
    switch (ee) {
      case LF: // LF = 0
        ee_pos_B = pos_B.at(ee);
        limbType_ = "IN_LEFT";
        limb_ = LimbEnum::LF_LEG;
        ROS_INFO_STREAM("LF EE Pos Base << " << ee_pos_B[0] << " " << ee_pos_B[1] << " " << ee_pos_B[2] << " " << std::endl);
        break;
      case RF:
//        ee_pos_H = pos_B.at(ee).cwiseProduct(Eigen::Vector3d(1,-1,1));
        ee_pos_B = pos_B.at(ee);
        limbType_ = "OUT_LEFT";
        limb_ = LimbEnum::RF_LEG;
        ROS_INFO_STREAM("RF EE Pos Base << " << ee_pos_B[0] << " " << ee_pos_B[1] << " " << ee_pos_B[2] << " " << std::endl);
        break;
      case LH:
//        ee_pos_H = pos_B.at(ee).cwiseProduct(Eigen::Vector3d(-1,1,1));
        ee_pos_B = pos_B.at(ee);
        limbType_ = "OUT_LEFT";
        limb_ = LimbEnum::LH_LEG;
        ROS_INFO_STREAM("LH EE Pos Base << " << ee_pos_B[0] << " " << ee_pos_B[1] << " " << ee_pos_B[2] << " " << std::endl);
        break;
      case RH:
//        ee_pos_H = pos_B.at(ee).cwiseProduct(Eigen::Vector3d(-1,-1,1));
        ee_pos_B = pos_B.at(ee);
        limbType_ = "IN_LEFT";
        limb_ = LimbEnum::RH_LEG;
        ROS_INFO_STREAM("RH EE Pos Base << " << ee_pos_B[0] << " " << ee_pos_B[1] << " " << ee_pos_B[2] << " " << std::endl);
        break;
      default: // joint angles for this foot do not exist
        break;
    }

//    ee_pos_H -= base2hip_LF_; // C -= A :: C = C - A.
//    q_vec.push_back(leg_ik.GetJointAngles(ee_pos_H, bend));
    q_vec.push_back(leg_ik.GetJointAngles(ee_pos_B, limbType_, limb_));
  }

  return Joints(q_vec);
}

} /* namespace xpp */
