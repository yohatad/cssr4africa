/*  gestureExecutionInterface.h
*
*   Interface file for the gesture execution package in the CSSR4Africa project
*
*   Audit Trail
*   -----------
*   Completed the implementation of the gesture execution package with proper documentation
*   Adedayo Akinade
*   28 June 2024

*
*/
#ifndef GESTURE_EXECUTION_INTERFACE_H
#define GESTURE_EXECUTION_INTERFACE_H

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <iomanip>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <fstream>
#include <sstream>
#include <string>
#include <boost/algorithm/string.hpp>
#include <geometry_msgs/Twist.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <vector>
#include <cmath>   

#include "cssr_system/perform_gesture.h"
#include <gestureExecution/pepperKinematicsUtilitiesInterface.h>


using namespace boost::algorithm;
using namespace std;

// Supported gesture types
#define DEICTIC_GESTURES "deictic"
#define DIECTIC_GESTURES "diectic"
#define ICONIC_GESTURES "iconic"
#define SYMBOLIC_GESTURES "symbolic"
#define BOWING_GESTURE "bow"
#define NODDING_GESTURE "nod"

// Supported interpolation types
#define LINEAR_INTERPOLATION 0
#define BIOLOGICAL_MOTION 1

// Indices of the joints in the joint state message
#define HEAD_PITCH_IDX 0
#define HEAD_YAW_IDX 1
#define HIP_PITCH_IDX 2
#define HIP_ROLL_IDX 3
#define KNEE_PITCH_IDX 4
#define LEFT_ELBOW_ROLL_IDX 5
#define LEFT_ELBOW_YAW_IDX 6
#define LEFT_HAND_IDX 7 
#define LEFT_SHOULDER_PITCH_IDX 8
#define LEFT_SHOULDER_ROLL_IDX 9
#define LEFT_WRIST_YAW_IDX 10
#define RIGHT_ELBOW_ROLL_IDX 11
#define RIGHT_ELBOW_YAW_IDX 12
#define RIGHT_HAND_IDX 13 
#define RIGHT_SHOULDER_PITCH_IDX 14
#define RIGHT_SHOULDER_ROLL_IDX 15
#define RIGHT_WRIST_YAW_IDX 16

// Maximum and mimimum joint angles
#define MIN_HEAD_YAW            -2.0857
#define MAX_HEAD_YAW            2.0857

#define MIN_HEAD_PITCH          -0.7068
#define MAX_HEAD_PITCH          0.6371

#define MIN_LSHOULDER_PITCH     -2.0857
#define MAX_LSHOULDER_PITCH     2.0857

#define MIN_LSHOULDER_ROLL      0.0087
#define MAX_LSHOULDER_ROLL      1.5620

#define MIN_LELBOW_YAW          -2.0857
#define MAX_LELBOW_YAW          2.0857

#define MIN_LELBOW_ROLL         -1.5620
#define MAX_LELBOW_ROLL         -0.0087

#define MIN_LWRIST_YAW          -1.8238
#define MAX_LWRIST_YAW          1.8238

#define MIN_RSHOULDER_PITCH     -2.0857
#define MAX_RSHOULDER_PITCH     2.0857

#define MIN_RSHOULDER_ROLL      -1.5620
#define MAX_RSHOULDER_ROLL      -0.0087

#define MIN_RELBOW_YAW          -2.0857
#define MAX_RELBOW_YAW          2.0857

#define MIN_RELBOW_ROLL         0.0087
#define MAX_RELBOW_ROLL         1.5620

#define MIN_RWRIST_YAW          -1.8238
#define MAX_RWRIST_YAW          1.8238

#define MIN_HIP_ROLL            -0.5149
#define MAX_HIP_ROLL            0.5149

#define MIN_HIP_PITCH           -1.0385
#define MAX_HIP_PITCH           1.0385

#define MIN_KNEE_PITCH          -0.5149
#define MAX_KNEE_PITCH          0.5149

#define MIN_HEAD_YAW            -2.0857
#define MAX_HEAD_YAW            2.0857

#define MIN_HEAD_PITCH          -0.7068
#define MAX_HEAD_PITCH          0.6371

#define ROS

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ControlClient;
typedef boost::shared_ptr<ControlClient> ControlClientPtr;

// Stores the states of the joints. Declared as global variables to allow access from the callback functions
extern std::vector<double> leg_joint_states;
extern std::vector<double> head_joint_states;
extern std::vector<double> right_arm_joint_states;
extern std::vector<double> left_arm_joint_states;

// Stores the robot location in the environment (x, y, theta)
extern std::vector<double> robot_pose;

/* 
 *   Function to read the the robot pose from an input file
 * @param:
 *   robot_pose_input: vector to store the robot pose
 *
 * @return:
 *    None
 */
void read_robot_pose_input(std::vector<double>& robot_pose_input);

/* 
 *   Function to initialize the /gestureExecution/perform_gesture service.
 *   The service is used to execute gestures on the robot. This initialization reads the gesture execution configuration file
 *   and sets the parameters for the gesture execution service.
 *
 * @param:
 *   implementation_platform: string to store the implementation platform
 *   interpolation_type: string to store the interpolation type
 *   gesture_descriptors_config_filename: string to store the gesture descriptors configuration filename
 *   simulator_topics_filename: string to store the simulator topics filename
 *   robot_topics_filename: string to store the robot topics filename
 *   verbose_mode_input: string to store the verbose mode input
 *   verbose_mode: boolean to store the verbose mode
 *
 * @return:
 *   None
 */
void initialize_server(string *implementation_platform, string *interpolation_type, string *gesture_descriptors_config_filename, string *simulator_topics_filename, string *robot_topics_filename, string *verbose_mode_input, bool* verbose_mode);

/* 
 *   Function to read the gesture execution configuration.
 *   The configuration file contains the platform, interpolation type, gesture descriptors, simulator topics, robot topics and verbose mode.
 *
 * @param:
 *   platform: string to store the platform
 *   interpolation: string to store the interpolation type
 *   gesture_descriptors: string to store the gesture descriptors filename
 *   simulator_topics: string to store the simulator topics filename
 *   robot_topics: string to store the robot topics filename
 *   verbose_mode: string to store the verbose mode
 *
 * @return:
 *   None
 */
void read_gesture_execution_configuration(string* platform, string* interpolation, string* gesture_descriptors, string* simulator_topics, string* robot_topics, string* verbose_mode);

/*   
 *   Function to extract the gesture descriptors from the gesture descriptors file.
 *   The gesture descriptors file contains the gesture type, gesture ID, number of waypoints and joint angles.
 *
 *   @param:
 *       gesture_descriptors_file: the gesture descriptors filename
 *       gesture_type: string to store the gesture type
 *       gesture_id: string to store the gesture ID
 *       number_of_waypoints: integer to store the number of waypoints
 *       joint_angles: string to store the joint angles
 *
 *   @return:
 *       None
 */
void read_gesture_descriptors(string gesture_descriptors_file, string* gesture_type, string* gesture_id, int* number_of_waypoints, string* joint_angles);

/*  
 *   Function to read the gesture descriptors configuration from the gesture descriptors configuration file.
 *   The gesture descriptors configuration file contains the gesture ID, gesture arm and gesture descriptors filename.
 *
 *   @param:
 *       gesture_descriptors_file: the gesture descriptors configuration filename
 *       gesture_descriptors_config: vector to store the gesture descriptors configuration
 *
 *   @return:
 *       None
 */
int read_gesture_descriptors_config(string gesture_descriptors_file, std::vector<std::vector<string>>& gesture_descriptors_config);

/*  
 *   Function to extract the gesture arm and gesture descriptors filename from the gesture ID.
 *   The gesture descriptors configuration file contains the gesture ID, gesture arm and gesture descriptors filename.
 *
 *   @param:
 *       gesture_id: the gesture ID
 *       gesture_descriptors_config: vector to store the gesture descriptors configuration
 *       gesture_descriptors_filename: string to store the gesture descriptors filename
 *       gesture_arm: string to store the gesture arm
 *
 *   @return:
 *       None
 */
void extract_info_from_ID(uint8_t gesture_id, std::vector<std::vector<string>> gesture_descriptors_config, string* gesture_descriptors_filename, string* gesture_arm);

/*  
 *   Function to extract the joint angles from the vector of waypoints specidfied in the gesture descriptors file.
 *   The joint angles are extracted from the gesture descriptors file.
 *
 *   @param:
 *       gesture_descriptors_file: the gesture descriptors filename
 *       joint_angles: string to store the joint angles
 *       number_of_waypoints: integer to store the number of waypoints
 *       waypoints: vector to store the waypoints
 *       verbose_mode: boolean to store the verbose mode
 *
 *   @return:
 *       1 if successful, 0 otherwise
 */
int extract_joint_angles(string gesture_descriptors_file, string joint_angles, int number_of_waypoints, std::vector<std::vector<double>>& waypoints, bool verbose_mode);

/*  
 *   Function to extract the topic from the topics file
 *   The function reads the topics file and extracts the topic for the specified key.
 *
 *   @param:
 *       key: the key to search for in the topics file
 *       topic_file_name: the topics filename
 *
 *   @return:
 *       the topic value
 */
string extract_topic(string key, string topic_file_name);

/*  
 *   Function to create a control client
 *   The function creates a control client for the specified topic.
 *
 *   @param:
 *       topic_name: the topic name
 *
 *   @return:
 *       the control client
 */
ControlClientPtr create_client(const std::string& topic_name);

/* 
 *   Function to return all joints of an actuator to the home position.
 *   This function is called on only one actuator at a time.
 *   It used the global variable set above for the home positions of the actuators.
 *
 * @param:
 *   actuator: string indicating the actuator to move to the home position
 *   topics_filename: string indicating the topics filename
 *   interpolation: integer indicating the interpolation type
 *   debug: boolean indicating the debug mode
 *
 * @return:
 *   1 if successful, 0 otherwise
 */
int go_to_home(std::string actuator, std::string topics_filename, int interpolation, bool debug);

/* 
 *   Function to execute deictic gestures. The deictic gestures are executed by pointing the robot's arm to the specified point in the environment.
 *   The pointing coordinates are the x, y and z coordinates of the point in the environment.
 *
 *   @param:
 *       point_x: the x coordinate of the point
 *       point_y: the y coordinate of the point
 *       point_z: the z coordinate of the point
 *       gesture_duration: integer to store the gesture duration
 *       topics_file: string to store the topics filename
 *       interpolation: integer to store the interpolation type
 *       velocity_publisher: the velocity publisher
 *       debug: boolean to store the debug mode
 *
 *   @return:
 *       1 if successful, 0 otherwise
 */
int deictic_gesture(float point_x, float point_y, float point_z, int gesture_duration, string topics_file, int interpolation, ros::Publisher velocity_publisher, bool debug);

/*  
 *   Function to execute iconic gestures. The iconic gestures are executed by moving the robot's arms to the waypoints specified in the gesture descriptors file.
 *   The waypoints are the joint angles for the arms at each point in time.
 *
 *   @param:
 *       gesture_arm_1: the gesture arm 1
 *       waypoints_arm_1: vector to store the waypoints of arm 1
 *       gesture_arm_2: the gesture arm 2
 *       waypoints_arm_2: vector to store the waypoints of arm 2
 *       open_right_hand: boolean to indicate if the right hand should be open
 *       open_left_hand: boolean to indicate if the left hand should be open
 *       gesture_duration: integer to store the gesture duration
 *       topics_file: string to store the topics filename
 *       interpolation: integer to store the interpolation type
 *       debug: boolean to store the debug mode
 *
 *   @return:
 *       1 if successful, 0 otherwise
 */
int iconic_gestures(string gesture_arm_1, std::vector<std::vector<double>> waypoints_arm_1, string gesture_arm_2, std::vector<std::vector<double>> waypoints_arm_2, bool open_right_hand, bool open_left_hand, int gesture_duration, string topics_file, int interpolation, bool debug);

/*  
 *   Function to execute bowing gesture
 *   The robot executes a bowing gesture by moving the leg to a specified angle.
 *
 *   @param:
 *       bow_angle: the angle of the leg
 *       gesture_duration: the duration of the gesture
 *       topics_file: the topics filename
 *       interpolation: the interpolation type
 *       debug: the debug mode
 *
 *   @return:
 *       1 if successful, 0 otherwise
 */
int bowing_gesture(int bow_angle, int gesture_duration, string topics_file, int interpolation, bool debug);

/*  
 *   Function to execute nodding gesture
 *   The robot executes a nodding gesture by moving the head to a specified angle.
 *
 *   @param:
 *       nod_angle: the angle of the head
 *       gesture_duration: the duration of the gesture
 *       topics_file: the topics filename
 *       interpolation: the interpolation type
 *       debug: the debug mode
 *
 *   @return:
 *       1 if successful, 0 otherwise
 */
int nodding_gesture(int nod_angle, int gesture_duration, string topics_file, int interpolation, bool debug);

/*  
 *   Function to move the head to a position specified by the head pitch and head yaw angles
 *   The function moves the head to the specified position using the control client
 *
 *   @param:
 *       head_topic: the topic for the head
 *       head_pitch: the pitch angle of the head
 *       head_yaw: the yaw angle of the head
 *       gesture_duration: the duration of the gesture
 *       interpolation: the type of interpolation to use
 *       debug: boolean to indicate if debugging information should be printed
 *
 *   @return:
 *       None
 */
void head_pointing(std::string head_topic, double head_pitch, double head_yaw, double gesture_duration, int interpolation, bool debug);

/*  
 *   Function to move the right arm to point in a particular direction as specified by the joint angles
 *   The function moves the right arm to point in the specified direction using the control client
 *
 *   @param:
 *       right_arm_topic: the topic for the right arm
 *       shoulder_pitch: the pitch angle of the shoulder
 *       shoulder_roll: the roll angle of the shoulder
 *       elbow_yaw: the yaw angle of the elbow
 *       elbow_roll: the roll angle of the elbow
 *       wrist_yaw: the yaw angle of the wrist
 *       gesture_duration: the duration of the gesture
 *       interpolation: the type of interpolation to use
 *       debug: boolean to indicate if debugging information should be printed
 *
 *   @return:
 *       None
 */
void right_arm_pointing(std::string right_arm_topic, double shoulder_pitch, double shoulder_roll, double elbow_yaw, double elbow_roll, double wrist_yaw, double gesture_duration, int interpolation, bool debug);

/*  
 *   Function to move the left arm to point in a particular direction as specified by the joint angles
 *   The function moves the left arm to point in the specified direction using the control client
 *
 *   @param:
 *       left_arm_topic: the topic for the left arm
 *       shoulder_pitch: the pitch angle of the shoulder
 *       shoulder_roll: the roll angle of the shoulder
 *       elbow_yaw: the yaw angle of the elbow
 *       elbow_roll: the roll angle of the elbow
 *       wrist_yaw: the yaw angle of the wrist
 *       gesture_duration: the duration of the gesture
 *       interpolation: the type of interpolation to use
 *       debug: boolean to indicate if debugging information should be printed
 *
 *   @return:
 *       None
 */
void left_arm_pointing(std::string left_arm_topic, double shoulder_pitch, double shoulder_roll, double elbow_yaw, double elbow_roll, double wrist_yaw, double gesture_duration, int interpolation, bool debug);

/*  
 *   Function to move the head in a nodding motion as specified by the nod angle
 *   
 *   @param:
 *       head_topic: the topic for the head
 *       nod_angle: the angle of the nod
 *       gesture_duration: the duration of the gesture
 *       interpolation: the type of interpolation to use
 *       debug: boolean to indicate if debugging information should be printed
 *
 *   @return:
 *       None
 */
void head_nodding(std::string head_topic, int nod_angle, int gesture_duration, int interpolation, bool debug);

/*  
 *   Function to move the leg in a bowing motion as specified by the bow angle
 *
 *   @param:
 *       leg_topic: the topic for the leg
 *       bow_angle: the angle of the bow
 *       gesture_duration: the duration of the gesture
 *       interpolation: the type of interpolation to use
 *       debug: boolean to indicate if debugging information should be printed
 *
 *   @return:
 *       None
 */
void leg_bowing(std::string leg_topic, int bow_angle, int gesture_duration, int interpolation, bool debug);

/*  
 *   Function to move the right hand to open. close or home position
 *   The function moves the hand to the specified position using the control client
 *
 *   @param:
 *       hand_topic: the topic for the hand
 *       state: the state of the hand (open, close or home)
 *   
 *   @return:
 *       None
 */
void right_hand_control(std::string right_hand_topic, std::string state);

/*  
 *   Function to move the left hand to open. close or home position
 *   The function moves the hand to the specified position using the control client
 *
 *   @param:
 *       hand_topic: the topic for the hand
 *       state: the state of the hand (open, close or home)
 *   
 *   @return:
 *       None
 */
void left_hand_control(std::string left_hand_topic, std::string state);

/*  
 *   Function to rotate the robot by a specified angle in degrees.
 *   The robot is rotated by the specified angle in degrees.
 *
 *   @param:
 *       angle_degrees: the angle in degrees
 *       velocity_publisher: the velocity publisher
 *       debug: boolean to store the debug mode
 *
 *   @return:
 *       None
 */
void rotate_robot(double angle_degrees, ros::Publisher velocity_publisher, bool debug);

/*  
 *   Function to compute the trajectory for an actuator from a start position to an end position
 *   The function uses the minimum-jerk model of biological motion to compute the trajectory
 *
 *   @param:
 *       start_position: vector containing the start position (joint angles) of the actuator
 *       end_position: vector containing the end position (joint angles) of the actuator
 *       number_of_joints: the number of joints in the actuator
 *       trajectory_duration: the duration of the trajectory
 *       positions: vector to store the positions of the computed trajectory
 *       velocities: vector to store the velocities of the computed trajectory
 *       accelerations: vector to store the accelerations of the computed trajectory
 *       durations: vector to store the durations of the computed trajectory
 *
 *   @return:
 *       None
 */
void compute_trajectory(std::vector<double> start_position, std::vector<double> end_position, 
                        int number_of_joints, double trajectory_duration, 
                        std::vector<std::vector<double>>& positions, std::vector<std::vector<double>>& velocities, 
                        std::vector<std::vector<double>>& accelerations, std::vector<double>& durations);

/*  
 *   Function to move the hand to a particular position
 *   The function moves the hand to the specified position using the control client
 *
 *   @param:
 *       hand_client: the control client for the hand
 *       joint_names: vector containing the joint names of the hand
 *       duration: the duration of the movement
 *       position_name: the name of the position
 *       positions: vector containing the joint angles of the position to move the hand to
 *
 *   @return:
 *       None
 */
void move_hand_to_position(ControlClientPtr& hand_client, const std::vector<std::string>& joint_names, double duration, 
                        const std::string& position_name, std::vector<double> positions);

/*  
 *   Function to move a single arm for an iconic gesture. The arm is moved to the waypoints specified in the waypoints vector
 *   The waypoints are the joint angles for the arm at each point in time.
 *
 *   @param:
 *       gesture_arm: the gesture arm
 *       joint_names: vector to store the joint names
 *       waypoints: vector to store the waypoints
 *       open_hand: boolean to indicate if the hand should be open
 *       hand: string to store the hand that should be open or closed
 *       gesture_duration: integer to store the gesture duration
 *       topics_file: string to store the topics filename
 *       interpolation: integer to store the interpolation type
 *       debug: boolean to store the debug mode
 *
 *   @return:
 *       None
 */
void move_single_arm_iconic(string gesture_arm, std::vector<string> joint_names, std::vector<std::vector<double>> waypoints, bool open_hand, string hand, int gesture_duration, string topics_file, int interpolation, bool debug);

/*  
 *   Function to move both arms for an iconic gesture. The arms are moved to the waypoints specified in the waypoints vector
 *   The waypoints are the joint angles for the arms at each point in time.
 *
 *   @param:
 *       gesture_arm_1: the gesture arm 1
 *       gesture_arm_2: the gesture arm 2
 *       joint_names_arm_1: vector to store the joint names of arm 1
 *       joint_names_arm_2: vector to store the joint names of arm 2
 *       waypoints_arm_1: vector to store the waypoints of arm 1
 *       waypoints_arm_2: vector to store the waypoints of arm 2
 *       open_right_hand: boolean to indicate if the right hand should be open
 *       open_left_hand: boolean to indicate if the left hand should be open
 *       gesture_duration: integer to store the gesture duration
 *       topics_file: string to store the topics filename
 *       interpolation: integer to store the interpolation type
 *       debug: boolean to store the debug mode
 *
 *   @return:
 *       None
 */
void move_both_arms_iconic(string gesture_arm_1, string gesture_arm_2, std::vector<string> joint_names_arm_1, std::vector<string> joint_names_arm_2, std::vector<std::vector<double>> waypoints_arm_1, std::vector<std::vector<double>> waypoints_arm_2, bool open_right_hand, bool open_left_hand, int gesture_duration, string topics_file, int interpolation, bool debug);

/*  
 *   Function to move an actuator to a position when using linear interpolation
 *   The actuator is moved using the control client to the specified position
 *
 *   @param:
 *       client: the control client for the actuator
 *       joint_names: vector containing the joint names of the actuator
 *       duration: the duration of the movement
 *       open_hand: boolean to indicate if the hand should be open
 *       hand: the hand to be opened
 *       hand_topic: the topic for the hand
 *       position_name: the name of the position
 *       positions: vector containing the joint angles of the position to move the actuator to
 *
 *   @return:
 *       None
 */                                                
void move_to_position(ControlClientPtr& client, const std::vector<std::string>& joint_names, double duration, 
                        bool open_hand, string hand, string hand_topic, 
                        const std::string& position_name, std::vector<double> positions);

/*  Function to move the arm to a position using the minimum-jerk model of biological motion
 *   The function moves the arm to the specified position using the control client
 *
 *   @param:
 *       client: the control client for the arm
 *       joint_names: vector containing the joint names of the arm
 *       duration: vector containing the duration of the movement
 *       gesture_duration: the duration of the gesture
 *       open_hand: boolean to indicate if the hand should be open
 *       hand: the hand to be opened
 *       hand_topic: the topic for the hand
 *       position_name: the name of the position
 *       positions: vector containing the joint angles of the position to move the arm to
 *       velocities: vector containing the joint velocities of the position to move the arm to
 *       accelerations: vector containing the joint accelerations of the position to move the arm to
 *
 *   @return:
 *       None
 */
void move_to_position_biological_motion(ControlClientPtr& client, const std::vector<std::string>& joint_names, std::vector<double> duration, double gesture_duration,
                        bool open_hand, string hand, string hand_topic, const std::string& position_name, 
                        std::vector<std::vector<double>> positions, std::vector<std::vector<double>> velocities, std::vector<std::vector<double>> accelerations);

/*  
 *   Function to move both arms to the specified positions using linear interpolation
 *   The function moves both arms to the specified positions using the control clients
 *
 *   @param:
 *       arm_1_client: the control client for the first arm
 *       arm_2_client: the control client for the second arm
 *       open_right_hand: boolean to indicate if the right hand should be open
 *       open_left_hand: boolean to indicate if the left hand should be open
 *       arm_1_joint_names: vector containing the joint names of the first arm
 *       arm_2_joint_names: vector containing the joint names of the second arm
 *       arm_1_duration: the duration of the movement for the first arm
 *       arm_2_duration: the duration of the movement for the second arm
 *       position_name: the name of the position
 *       arm_1_positions: vector containing the joint angles of the first arm
 *       arm_2_positions: vector containing the joint angles of the second arm
 *
 *   @return:
 *       None
 */
void move_to_position_iconic(ControlClientPtr& arm_1_client, ControlClientPtr& arm_2_client, bool open_right_hand, bool open_left_hand, 
                        const std::vector<std::string>& arm_1_joint_names, const std::vector<std::string>& arm_2_joint_names, 
                        double arm_1_duration, double arm_2_duration, const std::string& position_name, string topics_file,
                        std::vector<double> arm_1_positions, std::vector<double> arm_2_positions);

/*  
 *   Function to move both arms to the specified positions using the minimum-jerk model of biological motion
 *   The function moves both arms to the specified positions in their respective waypoints using the control clients
 *
 *   @param:
 *       arm_1_client: the control client for the first arm
 *       arm_2_client: the control client for the second arm
 *       open_right_hand: boolean to indicate if the right hand should be open
 *       open_left_hand: boolean to indicate if the left hand should be open
 *       arm_1_joint_names: vector containing the joint names of the first arm
 *       arm_2_joint_names: vector containing the joint names of the second arm
 *       arm_1_duration: vector containing the duration of the movement for the first arm
 *       arm_2_duration: vector containing the duration of the movement for the second arm
 *       gesture_duration: the duration of the gesture
 *       position_name: the name of the position
 *       arm_1_positions: vector containing the joint angles of the first arm
 *       arm_2_positions: vector containing the joint angles of the second arm
 *       arm_1_velocities: vector containing the joint velocities of the first arm
 *       arm_2_velocities: vector containing the joint velocities of the second arm
 *       arm_1_accelerations: vector containing the joint accelerations of the first arm
 *       arm_2_accelerations: vector containing the joint accelerations of the second arm
 *
 *   @return:
 *       None
 */
void move_to_position_biological_motion_iconic(ControlClientPtr& arm_1_client, ControlClientPtr& arm_2_client, bool open_right_hand, bool open_left_hand, 
                        const std::vector<std::string>& arm_1_joint_names, const std::vector<std::string>& arm_2_joint_names, 
                        std::vector<double> arm_1_duration, std::vector<double> arm_2_duration, double gesture_duration, 
                        const std::string& position_name, string topics_file,
                        std::vector<std::vector<double>> arm_1_positions, std::vector<std::vector<double>> arm_2_positions, 
                        std::vector<std::vector<double>> arm_1_velocities, std::vector<std::vector<double>> arm_2_velocities, 
                        std::vector<std::vector<double>> arm_1_accelerations, std::vector<std::vector<double>> arm_2_accelerations);

/*  
 *   Function to prompt the user to press any key to exit the program
 *
 *   @param:
 *       status: the status of the program
 *
 *   @return:
 *       None
 */
void prompt_and_exit(int status);

/*  
 *   Function to prompt the user to press any key to continue or press X to exit the program
 *
 *   @param:
 *       None
 *
 *   @return:
 *       None
 */
void prompt_and_continue();

#endif // GESTURE_EXECUTION_INTERFACE_H