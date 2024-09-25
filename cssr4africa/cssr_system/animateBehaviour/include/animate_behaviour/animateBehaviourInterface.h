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
#include <string>
#include <thread>
#include <functional>
#include <cmath>
#include <chrono>
#include <atomic>
#include <boost/thread.hpp>
#include "actionlib/client/simple_action_client.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "cssr_system/set_activation.h"
#include <iostream>
#include <sstream>
#include <std_srvs/SetBool.h>
#include <Eigen/Dense>

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ControlClient;
typedef boost::shared_ptr<ControlClient> ControlClientPtr;

extern std::map<std::string, ros::Publisher> publishers;
extern bool isActive;
extern bool verboseMode;
extern std::map<std::string, std::string> configParams;
extern std::map<std::string, std::string> topicData;
extern std::string platform;
extern double maximumRange;
extern double selectedRange;

extern std::vector<std::string> rArmJointNames;
extern std::vector<std::string> lArmJointNames;
extern std::vector<std::string> rhandJointNames;
extern std::vector<std::string> lhandJointNames;
extern std::vector<std::string> legJointNames;

extern std::ofstream logFile; 

void logToFile(const std::string &message);

void closeAndDeleteLogFile();

bool set_activation(cssr_system::set_activation::Request &req, cssr_system::set_activation::Response &res);

void trim(std::string &s);

void loadConfiguration(const std::string& filename);

void loadDataBasedOnPlatform(const std::string& platform);

void ensurePublisherInitialized(const std::string& actuatorName, ros::NodeHandle& nh);

ControlClientPtr createClient(const std::string& topicName);

void moveToPosition(ControlClientPtr& client, const std::vector<std::string>& jointNames,
                    const std::string& positionName, std::vector<double> positions);

void compute_trajectory(std::vector<double> startPosition, std::vector<double> endPosition, int numberOfJoints,
                        double trajectoryDuration, std::vector<std::vector<double>>& positions,
                        std::vector<std::vector<double>>& velocities, std::vector<std::vector<double>>& accelerations,
                        std::vector<double>& durations);
                        
void compute_trajectory_between_random_positions(std::vector<double>& currentPosition,
                                                 const std::vector<std::vector<double>>& random_positions, 
                                                 int number_of_joints, double trajectory_duration, 
                                                 std::vector<std::vector<double>>& positions, 
                                                 std::vector<std::vector<double>>& velocities, 
                                                 std::vector<std::vector<double>>& accelerations, 
                                                 std::vector<double>& durations);

void moveToPositionBiological(ControlClientPtr& client, 
                              const std::vector<std::string>& jointNames,
                              double gesture_duration,
                              const std::string& positionName, 
                              const std::vector<std::vector<double>>& positions, 
                              const std::vector<std::vector<double>>& velocities,
                              const std::vector<std::vector<double>>& accelerations, 
                              const std::vector<double>& durations);
                              
 void computeTrajectoryDetails(const std::vector<std::vector<double>>& positions, 
                              std::vector<std::vector<double>>& velocities, 
                              std::vector<std::vector<double>>& accelerations, 
                              std::vector<double>& durations, 
                              double totalDuration);

void moveToPositionWithTrajectory(ControlClientPtr& client, 
                                  const std::vector<std::string>& jointNames, 
                                  const std::vector<std::vector<double>>& positions, 
                                  const std::vector<std::vector<double>>& velocities, 
                                  const std::vector<std::vector<double>>& accelerations, 
                                  const std::vector<double>& durations);

void moveToPositiontest(ControlClientPtr& client, 
                                  const std::vector<std::string>& jointNames, 
                                  const std::vector<std::vector<double>>& positions, 
                                  const std::vector<double>& durations);

std::vector<std::vector<double>> calculateTargetPosition(const std::vector<double>& homePosition,
                                                         const std::vector<double>& maxPosition,
                                                         const std::vector<double>& minPosition,
                                                         const std::string& jointType);

double computeDistance(const std::vector<double>& pos1, const std::vector<double>& pos2);

void computeDurationsBasedOnDistance(const std::vector<std::vector<double>>& positions, 
                                     std::vector<double>& durations, 
                                     double baseTimeFactor);

void computeTrajectoryCubicSpline(const std::vector<std::vector<double>>& randomPositions,
                                  const std::vector<double>& currentPosition, 
                                  double totalTime,
                                  std::vector<std::vector<double>>& positions_t, 
                                  std::vector<std::vector<double>>& velocities_t, 
                                  std::vector<std::vector<double>>& accelerations_t, 
                                  std::vector<double>& durations);

double generateRandomPosition(double min, double max);

void flexiMovement(ros::NodeHandle& nh, const std::string& movementType);

void subtleBodyMovement(ros::NodeHandle& nh);

void rotationBaseShift(ros::NodeHandle& nh);

void animateBehaviour(const std::string& behaviour, ros::NodeHandle& nh);

void initRandomSeed();

const std::string FLAG_FILE_PATH = "first_run_flag.txt";

bool isFirstRun();

void updateFirstRunFlag();

std::vector<double> parsePercentages(const std::string& percentagesStr);

void rArm(ros::NodeHandle& nh, std::string rightArmTopic, bool resetPosition);

void lArm(ros::NodeHandle& nh, std::string leftArmTopic, bool resetPosition);

void rHand(ros::NodeHandle& nh, std::string rightHandTopic, bool resetPosition);

void lHand(ros::NodeHandle& nh, std::string leftHandTopic, bool resetPosition);

void leg(ros::NodeHandle& nh, std::string legTopic, bool resetPosition);

void rArml(ros::NodeHandle& nh, std::string rightArmTopic, bool resetPosition);

void lArml(ros::NodeHandle& nh, std::string leftArmTopic, bool resetPosition);

double calculateAngularVelocityZ(double maxAngularVelocity);

void resetAnimateBehaviour();

#endif // ANIMATEBEHAVIOURCONFIGANDSERVICE_H