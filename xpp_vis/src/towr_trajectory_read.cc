#include <xpp_msgs/RobotStateCartesian.h>
#include <xpp_msgs/topic_names.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ros/ros.h>

//using namespace xpp;
using namespace std;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "towr_trajectory_read_node");
    ros::NodeHandle nodehandle_;

    ros::Publisher towr_trajactory_pub_ = nodehandle_.advertise<xpp_msgs::RobotStateCartesian>(xpp_msgs::robot_state_desired, 1000);

    ros::Rate loop_rate(1000);

    string filePath;
    //filePath = "/home/eric/.ros/towr_trajectory.bag";
    string topic0 = "/towr/nlp_iterations_name10";
    string topic1 = "/towr/nlp_iterations_name1";
    vector<string> topics;
    topics.push_back(topic0);
    //topics.push_back(topic1);
    string Topic;
    nodehandle_.param("/topic", Topic, string("/towr/nlp_iterations_name0"));
    nodehandle_.param("/filePath", filePath, string("/home/eric/.ros/dataAnalysis/towr_trajectory.bag"));
    rosbag::Bag bag;
    bag.open(filePath, rosbag::bagmode::Read);
    rosbag::View view(bag, rosbag::TopicQuery(Topic));
    ROS_WARN_STREAM(Topic);

    xpp_msgs::RobotStateCartesian xpp_msgs;

    bool isDataFound = false;
    unsigned int count = 0;


    while(ros::ok())
    {
        ros::spinOnce();

        for(const auto& messageInstance : view)
        {
            xpp_msgs::RobotStateCartesian::ConstPtr message = messageInstance.instantiate<xpp_msgs::RobotStateCartesian>();
            count++;
            //ROS_INFO_STREAM(messageInstance.size());
            //ROS_INFO_STREAM(count); // 2 x 16001
            if(message != NULL)
            {
                xpp_msgs = *message;
                towr_trajactory_pub_.publish(xpp_msgs);
                isDataFound = true;
                // Strive4G8ness: Control publish frequency.
                loop_rate.sleep();
            }
            else
            {
                bag.close();
                ROS_ERROR("Unable to load data from ROS bag");
            }

            if(count >= 16001)
            {
                ros::shutdown();
            }
        }

//        loop_rate.sleep();
    }

    return 0;

}
