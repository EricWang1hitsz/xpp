#include <Eigen/Eigen>
#include <ros/ros.h>
#include <xpp_msgs/RobotStateJoint.h>
#include <sensor_msgs/JointState.h>
#include <fstream>
#include <iostream>
#include <sstream>

//using namespace Eigen;
using namespace std;

vector<vector<double>> joint_collections_;
double time_ = 0;
void jointStateCallback(const xpp_msgs::RobotStateJoint joint)
{
    sensor_msgs::JointState joint_states_; // 12 joints.
    joint_states_ = joint.joint_state;
    vector<double> joints_collection;
    for(int i = 0; i < 12; i++)
        joints_collection.push_back(joint_states_.position[i]);
    ROS_INFO_STREAM("Joint collection " << joints_collection.size() << std::endl);
    joint_collections_.push_back(joints_collection);

    std::ofstream file;
    file.open("data.txt", ios::app); // continue write at the end of file.
    if(file.fail())
        ROS_WARN("The file cannot be opened!");
    file << time_ << "   " << joints_collection[0] << "   " << joints_collection[1] << "   " << joints_collection[2] << "   " <<
            joints_collection[3] << "   " << joints_collection[4] << "   " << joints_collection[5] << "   " <<
            joints_collection[9] << "   " << joints_collection[10] << "   " << joints_collection[11] << "   " << // rh leg
            joints_collection[6] << "   " << joints_collection[7] << "   " << joints_collection[8] << "   "; // lh leg
    file << "\r\n";
    file.close();
    time_ += 0.0025;
    ROS_INFO_STREAM("Time " << time_ << std::endl);
    ROS_INFO("Save data Once");
    joints_collection.clear();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "jointdata");
    ros::NodeHandle nh_;

//    ros::Rate loop_rate_(100);

    ros::Subscriber joint_subscriber_;
    joint_subscriber_ = nh_.subscribe("xpp/joint_quadruped_des", 1000, jointStateCallback);


    ros::spin();
//    while(ros::ok())
//    {
//        ros::spinOnce();

//        loop_rate_.sleep();
//    }

    return 0;

}
