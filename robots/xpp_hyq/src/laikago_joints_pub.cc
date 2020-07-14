#include <xpp_vis/inverse_kinematics.h>
#include <xpp_msgs/RobotStateCartesian.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <laikago_msgs/MotorCmd.h>
#include <laikago_msgs/LowCmd.h>
#include <xpp_hyq/inverse_kinematics_laikago.h>
#include <xpp_states/convert.h>

namespace xpp {

class Laikago_joints_publisher
{
public:

    Laikago_joints_publisher()
    {
        robot_state_sub_ = nh_.subscribe("/xpp/state_des", 1, &Laikago_joints_publisher::stateCallback, this);

        joint_pub[0] = nh_.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/FR_hip_controller/command", 1);
        joint_pub[1] = nh_.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/FR_thigh_controller/command", 1);
        joint_pub[2] = nh_.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/FR_calf_controller/command", 1);
        joint_pub[3] = nh_.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/FL_hip_controller/command", 1);
        joint_pub[4] = nh_.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/FL_thigh_controller/command", 1);
        joint_pub[5] = nh_.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/FL_calf_controller/command", 1);
        joint_pub[6] = nh_.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/RR_hip_controller/command", 1);
        joint_pub[7] = nh_.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/RR_thigh_controller/command", 1);
        joint_pub[8] = nh_.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/RR_calf_controller/command", 1);
        joint_pub[9] = nh_.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/RL_hip_controller/command", 1);
        joint_pub[10] = nh_.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/RL_thigh_controller/command", 1);
        joint_pub[11] = nh_.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/RL_calf_controller/command", 1);

        joint_position_pub[0] = nh_.advertise<std_msgs::Float64>("/laikago_gazebo/FR_hip_joint_controller/command", 1);
        joint_position_pub[1] = nh_.advertise<std_msgs::Float64>("/laikago_gazebo/FR_thigh_joint_controller/command", 1);
        joint_position_pub[2] = nh_.advertise<std_msgs::Float64>("/laikago_gazebo/FR_calf_joint_controller/command", 1);
        joint_position_pub[3] = nh_.advertise<std_msgs::Float64>("/laikago_gazebo/FL_hip_joint_controller/command", 1);
        joint_position_pub[4] = nh_.advertise<std_msgs::Float64>("/laikago_gazebo/FL_thigh_joint_controller/command", 1);
        joint_position_pub[5] = nh_.advertise<std_msgs::Float64>("/laikago_gazebo/FL_calf_joint_controller/command", 1);
        joint_position_pub[6] = nh_.advertise<std_msgs::Float64>("/laikago_gazebo/RR_hip_joint_controller/command", 1);
        joint_position_pub[7] = nh_.advertise<std_msgs::Float64>("/laikago_gazebo/RR_thigh_joint_controller/command", 1);
        joint_position_pub[8] = nh_.advertise<std_msgs::Float64>("/laikago_gazebo/RR_calf_joint_controller/command", 1);
        joint_position_pub[9] = nh_.advertise<std_msgs::Float64>("/laikago_gazebo/RL_hip_joint_controller/command", 1);
        joint_position_pub[10] = nh_.advertise<std_msgs::Float64>("/laikago_gazebo/RL_thigh_joint_controller/command", 1);
        joint_position_pub[11] = nh_.advertise<std_msgs::Float64>("/laikago_gazebo/RL_calf_joint_controller/command", 1);
    }

    ~Laikago_joints_publisher()
    {

    }


    void stateCallback(const xpp_msgs::RobotStateCartesian& cart_msg)
    {
        auto cart = Convert::ToXpp(cart_msg);

        // transform feet from world -> base frame
        Eigen::Matrix3d B_R_W = cart.base_.ang.q.normalized().toRotationMatrix().inverse();
        EndeffectorsPos ee_B(cart.ee_motion_.GetEECount());
        for (auto ee : ee_B.GetEEsOrdered())
          ee_B.at(ee) = B_R_W * (cart.ee_motion_.at(ee).p_ - cart.base_.lin.p_);

        Eigen::VectorXd q =  laikago_ik.GetAllJointAngles(ee_B).ToVec();

        lowCmd.motorCmd[0].position = q[0];
        lowCmd.motorCmd[1].position = q[1];
        lowCmd.motorCmd[2].position = q[2];
        lowCmd.motorCmd[3].position = q[3];
        lowCmd.motorCmd[4].position = q[4];
        lowCmd.motorCmd[5].position = q[5];
        lowCmd.motorCmd[6].position = q[6];
        lowCmd.motorCmd[7].position = q[7];
        lowCmd.motorCmd[8].position = q[8];
        lowCmd.motorCmd[9].position = q[9];
        lowCmd.motorCmd[10].position = q[10];
        lowCmd.motorCmd[11].position = q[11];

        jointCmd[3].data = q[0];
        jointCmd[4].data = q[1];
        jointCmd[5].data = q[2];
        ROS_INFO_STREAM("FL_Joint3 :" << jointCmd[2] << std::endl);
        jointCmd[0].data = q[3];
        jointCmd[1].data = q[4];
        jointCmd[2].data = q[5];
        jointCmd[9].data = q[6];
        jointCmd[10].data = q[7];
        jointCmd[11].data = q[8];
        jointCmd[6].data = q[9];
        jointCmd[7].data = q[10];
        jointCmd[8].data = q[11];


    }

    void publish()
    {

//        for(int m = 0; m < 12; m++)
//        {
//            joint_pub[m].publish(lowCmd.motorCmd[m]);
//        }

        for(int n = 0; n < 12; n++)
        {
            joint_position_pub[n].publish(jointCmd[n]);
        }
    }
private:

    ros::NodeHandle nh_;

    ros::Subscriber robot_state_sub_;

    ros::Publisher joint_pub[12];

    ros::Publisher joint_position_pub[12];

    inverseKinematicsLaikago laikago_ik;

    laikago_msgs::LowCmd lowCmd;

    std_msgs::Float64 jointCmd[12];


};

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laikago_joint_publisher");
    ros::NodeHandle nh;
    xpp::Laikago_joints_publisher laikago_joint_pub_;

    ros::Rate loop_rate(100);

    while(ros::ok())
    {

        laikago_joint_pub_.publish();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
