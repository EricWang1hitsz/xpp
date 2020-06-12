/********************************************************

@File Leg inverse_kinematics.h

@Description Lge inverse kinematics of quadruped robot.

@Author  Eric Wang

@Date:   2020-01-11

@Association: Harbin Institute of Technology, Shenzhen.

*********************************************************/

#pragma once

#include <Eigen/Dense>

namespace xpp {

enum JointID {HAA=0, HFE, KFE, legJointCount};

/**
 * @brief Converts a foot position to joint angles.
 */
class legInverseKinematics
{
public:

  using Vector3d = Eigen::Vector3d;
    /**
   * @author eric wang
   * @brief The KneeBend enum Define knee bending direction.
   */
  enum KneeBend { Forward, Backward };
  /**
   * @brief Default c'tor initializing leg lengths with standard values.
   */
  legInverseKinematics () = default;
  virtual ~legInverseKinematics () = default;

  /**
   * @brief Returns the joint angles to reach a Cartesian foot position.
   * @param ee_pos_H  Foot position xyz expressed in the frame attached
   * at the hip-aa (H).
   */
  Vector3d GetJointAngles(const Vector3d& ee_pos_H, KneeBend bend=Forward) const;

  /**
   * @brief Restricts the joint angles to lie inside the feasible range
   * @param q[in/out]  Current joint angle that is adapted if it exceeds
   * the specified range.
   * @param joint  Which joint (HAA, HFE, KFE) this value represents.
   */
  void EnforceLimits(double& q, JointID joint) const;

private:
  Vector3d hfe_to_haa_z = Vector3d(0.0, 0.0, 0.08); //distance of HFE to HAA in z direction
  double length_thigh = 0.35; // length of upper leg
  double length_shank = 0.33; // length of lower leg
};

} /* namespace xpp */
