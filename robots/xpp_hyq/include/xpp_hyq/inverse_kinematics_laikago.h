/********************************************************

@File Inverse_kinematics_bot.h

@Description Inverse kinematics of Laikago robot.

@Author  Eric Wang

@Date:   2020-06-30

@Association: Harbin Institute of Technology, Shenzhen.

*********************************************************/

#pragma once

#include <xpp_vis/inverse_kinematics.h>
#include <xpp_hyq/laikaleg_inverse_kinematics.h>

namespace xpp {

/**
 * @brief Inverse kinematics function for the quadruped robot.
 */
class inverseKinematicsLaikago: public InverseKinematics {
public:
  inverseKinematicsLaikago() = default;
  virtual ~inverseKinematicsLaikago() = default;

  /**
   * @brief Returns joint angles to reach for a specific foot position.
   * @param pos_B  3D-position of the foot expressed in the base frame (B).
   */
  Joints GetAllJointAngles(const EndeffectorsPos& pos_b) const override;

  /**
   * @brief Number of endeffectors (feet, hands) this implementation expects.
   */
  int GetEECount() const override { return 4; };

private:
  Vector3d base2hip_LF_ = Vector3d(0.21935, 0.0875, 0.0); //! eric_wang: Measure it from Solidworks.
  LaikagolegInverseKinematics leg_ik;
};

} /* namespace xpp */
