#include <ros/ros.h>
#include <ros/package.h>
#include <naoqi_driver/AudioCustomMsg.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <geometry_msgs/Twist.h>

#define ROS
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ControlClient;
typedef boost::shared_ptr<ControlClient> ControlClientPtr;

void audiocallback(const naoqi_driver::AudioCustomMsg& msg);
void moveHeadToPosition(ros::NodeHandle& nh);
void moveToPosition(ControlClientPtr& client, const std::vector<std::string>& jointNames, double duration, std::vector<double> positions);