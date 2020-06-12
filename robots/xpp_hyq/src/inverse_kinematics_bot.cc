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
#include <xpp_hyq/leg_inverse_kinematics.h>

namespace xpp {

Joints
inverseKinematicsBot::GetAllJointAngles(const EndeffectorsPos& eep_B) const
{
  Vector3d ee_pos_H; // foothold expressed in hip frame
  std::vector<Eigen::VectorXd> q_vec;

  // make sure always exactly 4 elements
  auto pos_B = eep_B.ToImpl();// foothold position in base frame.
  pos_B.resize(4, pos_B.front());

  for (int ee=0; ee<pos_B.size(); ++ee) {

    legInverseKinematics::KneeBend bend = legInverseKinematics::Forward;// default direction.

    using namespace quad;
    switch (ee) {
      case LF:
        ee_pos_H = pos_B.at(ee);
//        bend = legInverseKinematics::Backward;
        break;
      case RF:
        ee_pos_H = pos_B.at(ee).cwiseProduct(Eigen::Vector3d(1,-1,1));
        bend = legInverseKinematics::Backward;
        break;
      case LH:
        ee_pos_H = pos_B.at(ee).cwiseProduct(Eigen::Vector3d(-1,1,1));
        bend = legInverseKinematics::Backward;
        break;
      case RH:
        ee_pos_H = pos_B.at(ee).cwiseProduct(Eigen::Vector3d(-1,-1,1));
//        bend = legInverseKinematics::Backward;
        break;
      default: // joint angles for this foot do not exist
        break;
    }

    ee_pos_H -= base2hip_LF_; // C -= A :: C = C - A.
    q_vec.push_back(leg_ik.GetJointAngles(ee_pos_H, bend));
  }

  return Joints(q_vec);
}

} /* namespace xpp */
