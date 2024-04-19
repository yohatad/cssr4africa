#ifndef GESTURE_EXECUTION_H
#define GESTURE_EXECUTION_H

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iomanip>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <ros/package.h>
#include <fstream>
#include <sstream>
#include <string>
#include <boost/algorithm/string.hpp>
#include <geometry_msgs/Twist.h>


#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <thread>
#include <vector>
#include <cmath>   

#include <gesture_execution/pepperKinematicsUtilities.h>
// 


using namespace boost::algorithm;
using namespace std;



#define DEICTIC_GESTURES "deictic"
#define DIECTIC_GESTURES "diectic"
#define ICONIC_GESTURES "iconic"
#define SYMBOLIC_GESTURES "symbolic"

#define LINEAR_INTERPOLATION 0
#define BIOLOGICAL_MOTION 1

#define HeadPitchIdx 0
#define HeadYawIdx 1
#define HipPitchIdx 2
#define HipRollIdx 3
#define KneePitchIdx 4
#define LElbowRollIdx 5
#define LElbowYawIdx 6
#define LHandIdx 7 
#define LShoulderPitchIdx 8
#define LShoulderRollIdx 9
#define LWristYawIdx 10
#define RElbowRollIdx 11
#define RElbowYawIdx 12
#define RHandIdx 13 
#define RShoulderPitchIdx 14
#define RShoulderRollIdx 15
#define RWristYawIdx 16

// Maximum and mimimum joint angles
#define MIN_HEAD_YAW -2.0857
#define MAX_HEAD_YAW 2.0857

#define MIN_HEAD_PITCH -0.7068
#define MAX_HEAD_PITCH 0.6371

#define MIN_LSHOULDER_PITCH -2.0857
#define MAX_LSHOULDER_PITCH 2.0857

#define MIN_LSHOULDER_ROLL 0.0087
#define MAX_LSHOULDER_ROLL 1.5620

#define MIN_LELBOW_YAW -2.0857
#define MAX_LELBOW_YAW 2.0857

#define MIN_LELBOW_ROLL -1.5620
#define MAX_LELBOW_ROLL -0.0087

#define MIN_LWRIST_YAW -1.8238
#define MAX_LWRIST_YAW 1.8238

#define MIN_RSHOULDER_PITCH -2.0857
#define MAX_RSHOULDER_PITCH 2.0857

#define MIN_RSHOULDER_ROLL -1.5620
#define MAX_RSHOULDER_ROLL -0.0087

#define MIN_RELBOW_YAW -2.0857
#define MAX_RELBOW_YAW 2.0857

#define MIN_RELBOW_ROLL 0.0087
#define MAX_RELBOW_ROLL 1.5620

#define MIN_RWRIST_YAW -1.8238
#define MAX_RWRIST_YAW 1.8238

#define MIN_HIP_ROLL -0.5149
#define MAX_HIP_ROLL 0.5149

#define MIN_HIP_PITCH -1.0385
#define MAX_HIP_PITCH 1.0385

#define MIN_KNEE_PITCH -0.5149
#define MAX_KNEE_PITCH 0.5149

#define MIN_HEAD_YAW -2.0857
#define MAX_HEAD_YAW 2.0857

#define MIN_HEAD_PITCH -0.7068
#define MAX_HEAD_PITCH 0.6371

#define TORSO_HEIGHT            820.0

#define ROS
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ControlClient;
typedef boost::shared_ptr<ControlClient> ControlClientPtr;


void initialize_server(string *implementationPlatform, string *interpolationType, string *gestureDescriptorsFileName, string *simulatorTopicsFileName, string *robotTopicsFileName, string *verboseModeInput, bool* verboseMode);


/* Read gesture execution configuration file */
void read_gesture_execution_configuration(string* implementationPlatform, string* interpolationType, string* gestureDescriptorsFileName, string* simulatorTopicsFileName, string* robotTopicsFileName, string* verboseMode);

/* Read gesture descriptors */
void read_gesture_descriptors(string gesture_descriptors_file, string* gesture_type, string* gesture_id, int* number_of_waypoints, string* joint_angles);

/* Read the gesture execution input */
void read_gesture_execution_input(string* gesture_type, string* gesture_id, string* robot_location_input, string* debug);

/* Execute pointing gesture -- ID 01 */
int pointing_gesture(float point_x, float point_y, float point_z, int gesture_duration, string topics_file, int interpolation, bool debug);

/* Execute bowing gesture -- ID 02 */
int bowing_gesture(int bow_angle, int gesture_duration, string topics_file, int interpolation, bool debug);

/* Execute nodding gesture -- ID 03 */
int nodding_gesture(int bow_angle, int gesture_duration, string topics_file, int interpolation, bool debug);

/* Execute waving gesture -- ID 04 */
int execute_waving_gesture(string robot_location_input, bool debug);

/* Extract topic names for the respective simulator or physical robot */
string extract_topic(string key, string topic_file_name);

void go_to_home(std::string actuator, std::string topicsFilename, int interpolation, bool debug);
void go_to_home_BM();

void compute_trajectory(std::vector<double> startPosition, std::vector<double> endPosition, int numberOfJoints,
                        double trajectoryDuration, std::vector<std::vector<double>>& positions, 
                        std::vector<std::vector<double>>& velocities, std::vector<std::vector<double>>& accelerations, 
                        std::vector<double>& durations);

void move_to_position(ControlClientPtr& client, const std::vector<std::string>& jointNames, double duration, 
                        const std::string& positionName, std::vector<double> positions);
    
void move_to_position_BM(ControlClientPtr& client, const std::vector<std::string>& jointNames, std::vector<double> duration, double gesture_duration,
                        const std::string& positionName, std::vector<std::vector<double>> positions, std::vector<std::vector<double>> velocities, std::vector<std::vector<double>> accelerations);

void move_to_position_BM_new(ControlClientPtr& armClient, ControlClientPtr& headClient, const std::vector<std::string>& armJointNames, const std::vector<std::string>& headJointNames, std::vector<double> duration, 
                        const std::string& positionName, std::vector<std::vector<double>> armPositions, std::vector<std::vector<double>> headPositions, 
                        std::vector<std::vector<double>> armVelocities, std::vector<std::vector<double>> headVelocities, 
                        std::vector<std::vector<double>> armAccelerations, std::vector<std::vector<double>> headAccelerations);

std::vector<std::vector<double>> calculateDuration(std::vector<double> homePosition, std::vector<double> maxPosition, std::vector<double> minPosition, std::vector<std::vector<double>> velocity);

ControlClientPtr create_client(const std::string& TopicName);
void rArmPointing(std::string rightArmTopic, double shoulderPitch, double shoulderRoll, double elbowYaw, double elbowRoll, double wristYaw, double gesture_duration, int interpolation, bool debug);
void rArmHeadPointing(std::string rightArmTopic, std::string headTopic, double shoulderPitch, double shoulderRoll, double elbowYaw, double elbowRoll, double wristYaw, double headPitch, double headYaw, double gestureDuration, int interpolation, bool debug);
void rArmPointingNew(std::string rightArmTopic, double shoulderPitch, double shoulderRoll, double elbowYaw, double elbowRoll, double wristYaw, double headPitch, double headYaw, double gestureDuration, int interpolation, bool debug);
void rArmPointingHome(std::string rightArmTopic, double shoulderPitch, double shoulderRoll, double elbowYaw, double elbowRoll, double wristYaw, double gestureDuration, int interpolation, bool debug);
void lArmPointing(std::string leftArmTopic, double shoulderPitch, double shoulderRoll, double elbowYaw, double elbowRoll, double wristYaw, double gesture_duration, int interpolation, bool debug);
void lArmHeadPointing(std::string leftArmTopic, std::string headTopic, double shoulderPitch, double shoulderRoll, double elbowYaw, double elbowRoll, double wristYaw, double headPitch, double headYaw, double gestureDuration, int interpolation, bool debug);
void lArmPointingNew(std::string leftArmTopic, double shoulderPitch, double shoulderRoll, double elbowYaw, double elbowRoll, double wristYaw, double headPitch, double headYaw, double gestureDuration, int interpolation, bool debug);
void headNodding(std::string headTopic, int nod_angle, int gesture_duration, int interpolation, bool debug);
void headPointing(std::string headTopic, double headPitch, double headYaw, double gestureDuration, int interpolation, bool debug);
void lArmPointingHome(std::string leftArmTopic, double shoulderPitch, double shoulderRoll, double elbowYaw, double elbowRoll, double wristYaw, double gestureDuration, int interpolation, bool debug);
void legBowing(std::string legTopic, int bow_angle, int gesture_duration, int interpolation, bool debug);
void head(std::string headTopic, int nod_angle, int gesture_duration, bool debug);
void rArm(ros::NodeHandle& nh, std::string rightArmTopic);
void lArm(ros::NodeHandle& nh, std::string leftArmTopic);
void rHand(std::string rightHandTopic, std::string state);
void lHand(std::string leftHandTopic, std::string state);
void leg(std::string legTopic, int bow_angle, int gesture_duration, bool debug);
void wheels(ros::NodeHandle& nh, std::string wheelTopic);

/* Helper Functions */
void prompt_and_exit(int status);

void prompt_and_continue();

#endif // GESTURE_EXECUTION_H