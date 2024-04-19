// animateBehaviourConfigAndService.h updated version

#ifndef ANIMATEBEHAVIOURCONFIGANDSERVICE_H
#define ANIMATEBEHAVIOURCONFIGANDSERVICE_H

#include <ros/ros.h>
#include <ros/package.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Float64.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/Twist.h>
#include <fstream>
#include <sstream>
#include <string>
#include <map>
#include <vector>
#include <random>
#include <ctime>
#include <trajectory_msgs/JointTrajectory.h>
#include <string> // For std::stod
#include <thread>
#include "actionlib/client/simple_action_client.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include <functional>

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ControlClient;
typedef boost::shared_ptr<ControlClient> ControlClientPtr;

extern std::map<std::string, ros::Publisher> publishers;
extern bool isActive;
extern std::map<std::string, std::string> configParams;
extern std::map<std::string, std::string> topicData;

void trim(std::string &s);
std::string loadConfiguration(const std::string& filename);

void loadDataBasedOnPlatform(const std::string& platform);
bool setActivation(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
//void ensurePublisherInitialized(const std::string& actuatorName, ros::NodeHandle& nh);
void moveToPosition(ControlClientPtr& client, const std::vector<std::string>& jointNames,
                    const std::string& positionName, std::vector<double> positions) ;

std::vector<double> calculateDuration(const std::vector<double>& currentPosition, const std::vector<double>& targetPosition, const std::vector<double>& velocities);

//void head(ros::NodeHandle& nh, std::string headTopic);
void rArm(ros::NodeHandle& nh, std::string rightArmTopic, 
                               int start, int end);
void lArm(ros::NodeHandle& nh, std::string leftArmTopic, 
                               int start, int end);
void rHand(ros::NodeHandle& nh, std::string rightHandTopic, 
                                int start, int end);
void lHand(ros::NodeHandle& nh, std::string leftHandTopic, 
                               int start, int end);
void leg(ros::NodeHandle& nh, std::string legTopic, 
                              int start, int end);
//void wheels(ros::NodeHandle& nh, std::string wheelTopic);

double randomDoubleInRange(double min, double max);
void flexiMovement(ros::NodeHandle& nh);
void subtleBodyMovement(ros::NodeHandle& nh);
void rotationBaseShift(ros::NodeHandle& nh);
void animateBehaviour(const std::string& behaviour, ros::NodeHandle& nh);
void initRandomSeed();
std::vector<double> calculateTargetPosition(const std::vector<double>& homePosition, const std::vector<double>& maxPosition, const std::vector<double>& minPosition);


#endif // ANIMATEBEHAVIOURCONFIGANDSERVICE_H

