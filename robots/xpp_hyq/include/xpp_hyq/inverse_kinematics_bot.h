/********************************************************

@File Inverse_kinematics_bot.h

@Description Inverse kinematics of quadruped robot.

@Author  Eric Wang

@Date:   2020-01-11

@Association: Harbin Institute of Technology, Shenzhen.

*********************************************************/

#pragma once

#include <xpp_vis/inverse_kinematics.h>
#include <xpp_hyq/leg_inverse_kinematics.h>

namespace xpp {

/**
 * @brief Inverse kinematics function for the quadruped robot.
 */
class inverseKinematicsBot: public InverseKinematics {
public:
  inverseKinematicsBot() = default;
  virtual ~inverseKinematicsBot() = default;

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
  Vector3d base2hip_LF_ = Vector3d(0.420, 0.075, 0.0); //! eric_wang: Measure it from Solidworks.
  legInverseKinematics leg_ik;
};

} /* namespace xpp */
