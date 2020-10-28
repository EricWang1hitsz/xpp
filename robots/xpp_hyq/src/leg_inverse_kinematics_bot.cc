#include <xpp_hyq/leg_inverse_kinematics_bot.h>
#include <ros/ros.h>

namespace xpp {

bool legInverseKinematicsX::initializeKinematics()
{
//    ROS_INFO_STREAM("Hip to Base << " QK_.hip_pose_in_base_ << std::endl);
    QK_.reset(new QuadrupedKinematics());
}

legInverseKinematicsX::Vector3d
legInverseKinematicsX::GetJointAngles (const Vector3d& ee_pos_B, std::string limbType, LimbEnum limb)
{
    Eigen::Vector3d jointPosition;
    Position foot_position(ee_pos_B);
    //ROS_INFO_STREAM("EE Pos In Base Frame << " << foot_position(0) << " " << foot_position(1) << " " << foot_position(2) << std::endl);
    JointPositionsLimb joint_positions, joint_positions_last;

    QK_->InverseKinematicsSolve(foot_position, limb, joint_positions_last, joint_positions, limbType);
    //ROS_INFO_STREAM("Limb Joint Pos << " << joint_positions(0) << " " << joint_positions(1) << " " << joint_positions(2) << std::endl);
    jointPosition = Eigen::Vector3d(joint_positions.toImplementation());
//    Position foot_position_in_hip = QK_.getPositionFootToHipInHipFrame(limb, foot_position);
//    double px, py, pz; // foot position in hip frame.
//    //!Attention: hip frame is different from base frame.
//    px = foot_position_in_hip(0);
//    py = foot_position_in_hip(1);
//    pz = foot_position_in_hip(2);

//    ROS_INFO_STREAM("Foothold position in Hip frame " << px << py << pz << std::endl);


//    double l1, l2; // leg length.
//    l1 = l2 = 0.35;

//    double d;
//    d = 0.1;

//    double cos_theta3 =
//            (l2*l2 + l1*l1 - ((px*px + py*py + pz*pz) - d*d))/2/l1/l2;

//    if(cos_theta3<-1)
//      cos_theta3 = -1;
//    if(cos_theta3>1)
//      cos_theta3 = 1;

//    Eigen::VectorXd theta3(4);
//    Eigen::MatrixXd results(4, 3);
//    theta3(0) = M_PI - acos(cos_theta3);
//    theta3(1) = M_PI - acos(cos_theta3);
//    theta3(2) = -M_PI + acos(cos_theta3);
//    theta3(3) = -M_PI + acos(cos_theta3);

//    double alpha, beta1, beta2;
//    alpha = atan2(py, px);
//    beta1 = atan2(d,sqrt(fabs(px*px + py*py - d*d)));
//    beta2 = atan2(-d,-sqrt(fabs(px*px + py*py - d*d)));

//    int i = 0;
//    while (i < 4) {
//      double a, b, q1, q2, q3;
//      q3 = MapToPI(theta3(i));
//      // Left arm configure
//      q1 = MapToPI(alpha - beta1);
//      a = atan2(pz, -sqrt(fabs(px * px + py * py - d * d)));
//      b = atan2(l2 * sin(q3), l1 + l2 * cos(q3));
//      if(a > 0)
//      {
//        q2 = MapToPI(a - b - M_PI);
//        results.row(i) << q1,q2,q3;
//      }
//      if(a < 0)
//      {
//        q2 = MapToPI(a - b + M_PI);
//        results.row(i) << q1,q2,q3;
//      }
////      if(ee == 0 || ee == 2) // Left leg.
////      {
////        q1 = 0;
////        jointPosition << q1, q2, q3;
////        ROS_INFO_STREAM("Joint position " << q1 << " " << q2 << " " << " " << q3 << std::endl);
////      }
//      //right arm config
//      i = i + 1;
//      q1 = MapToPI(alpha + beta2);
//      a = atan2(pz, sqrt(fabs(px * px + py * py - d * d)));
//      q2 = MapToPI(a - b + M_PI);
////      if(ee == 1 || ee == 3) // Right leg.
////      {
////          q1 = 0;
////          jointPosition << q1, q2, q3;
////          ROS_INFO_STREAM("Joint position " << q1 << " " << q2 << " " << " " << q3 << std::endl);
////      }
//      results.row(i) << q1, q2, q3;
//    }

//      jointPosition << results(2, 0), results(2, 1), results(2, 2);
////    jointPosition << 0, results(2, 1), results(2, 2);


      return jointPosition;
}

double legInverseKinematicsX::MapToPI(double q)
{
    double out;
    out = q;
    if(q > M_PI)
      out = 2 * M_PI - q;
    if(q < -M_PI)
      out = 2 * M_PI + q;
    return out;
}

}
