/* overtAttentionImplementation.cpp
*
* Author: Adedayo AKinade
* Date: April 2, 2024
* Version: v1.0
*
* Copyright (C) 2023 CSSR4Africa Consortium
*
* This project is funded by the African Engineering and Technology Network (Afretec)
* Inclusive Digital Transformation Research Grant Programme.
*
* Website: www.cssr4africa.org
*
* This program comes with ABSOLUTELY NO WARRANTY.
*/

#ifndef OVERT_ATTENTION_H
#define OVERT_ATTENTION_H

#include <ros/ros.h>
#include <ros/package.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <string>
#include <boost/algorithm/string.hpp>
#include <geometry_msgs/Twist.h>
#include <vector>
#include <cmath>
#include <math.h>
#include <std_msgs/Float64.h>  // Include for publishing Float64 messages
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include "face_detection/FaceDetectionData.h"
#include "overt_attention/set_mode.h"
#include "overt_attention/set_activation.h"
#include <iomanip> 
#include <regex>
#include <thread>
#include <mutex>

#include <image_transport/image_transport.h>

//opencv
#include <opencv2/opencv.hpp>
#include <opencv2/saliency.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std;

using namespace boost::algorithm;

#define ROS
#define DEBUG 0
#define TORSO_HEIGHT 820.0

#define ATTENTION_SOCIAL_STATE      "social"
#define ATTENTION_SCANNING_STATE    "scanning"
#define ATTENTION_LOCATION_STATE    "location"
#define ATTENTION_SEEKING_STATE     "seeking"
#define ATTENTION_MODE_SOCIAL       0
#define ATTENTION_MODE_SCANNING     1
#define ATTENTION_MODE_SEEKING      2
#define ATTENTION_MODE_LOCATION     3

#define ATTENTION_ENABLED_STATE     "enabled"
#define ATTENTION_DISABLED_STATE    "disabled"
#define ATTENTION_SYSTEM_ENABLED    1
#define ATTENTION_SYSTEM_DISABLED   0

// Indices of the joints in the joint state message
#define HEAD_PITCH_IDX 0
#define HEAD_YAW_IDX 1
#define HIP_PITCH_IDX 2
#define HIP_ROLL_IDX 3
#define KNEE_PITCH_IDX 4

// Maximum and mimimum joint angles
#define MIN_HEAD_YAW            -2.0857
#define MAX_HEAD_YAW            2.0857

#define MIN_HEAD_PITCH          -0.7068
#define MAX_HEAD_PITCH          0.6371

// Number of faces to look through if there are multiple faces
#define NUMBER_OF_FACES_SOCIAL_ATTENTION 3

#define ENGAGEMENT_STATUS_ENGAGED 1
#define ENGAGEMENT_STATUS_NOT_ENGAGED -1
#define ENGAGEMENT_STATUS_NEUTRAL 0

#define ENGAGEMENT_TIMEOUT 5.00


// Stores the states of the joints. Declared as global variables to allow access from the callback functions
extern std::vector<double> head_joint_states;

// Stores the robot location in the environment (x, y, theta)
extern std::vector<double> robot_pose;

// Head joint angles for the robot control during scanning or social attention
// extern double attention_head_pitch;
// extern double attention_head_yaw;
extern std::vector<double> attention_head_pitch;
extern std::vector<double> attention_head_yaw;

extern std::vector<int> face_labels;                               // Stores the labels of the detected faces
extern int last_seen_label;                                   // Stores the label of the last seen face
extern int current_label;                                     // Stores the label of the current face

extern std::vector<double> gaze_angles;                           // Stores the gaze angles of the detected faces

extern double previous_attention_head_pitch;
extern double previous_attention_head_yaw;

extern double angle_of_sound;                        // Stores the angle of the sound source
extern double previous_angle_of_sound;               // Stores the previous angle of the sound source

extern bool face_detected;                         // Stores the status of face detection
extern bool sound_detected;                        // Stores the status of sound detection

// Variables for the attention mode set
extern int attention_mode;                            // Stores the attention mode currently set. Default is -1 on initialization
extern double location_x;                            // Stores the x-coordinate of the location to pay attention to
extern double location_y;                            // Stores the y-coordinate of the location to pay attention to
extern double location_z;                            // Stores the z-coordinate of the location to pay attention to
extern bool location_attended_to;                  // Stores the status if the location request has been attended to once

// Variables for the attention activation set
extern int system_activation_status;                  // Stores the activation status of the attention system. Default is -1 on initialization

// Variable for social attention done once
extern bool social_attention_done;
extern int sound_count;

// Variable for checking if engagement is found - 0 is neutral, 1 is success, -1 is failure
extern int engagement_status;

// Message to publish the engagement status
extern std_msgs::Float64 overt_attention_engagement_status_msg;

// // Publisher for the velocity commands
// extern ros::Publisher velocity_publisher;

// // Declare the publisher of the /overt_attention/engagement_status topic
// extern ros::Publisher overt_attention_engagement_status_pub;

#define VFOV 44.30
#define HFOV 55.20

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ControlClient;
typedef boost::shared_ptr<ControlClient> ControlClientPtr;

// std::mutex goal_mutex;
// std::mutex publish_mutex;

void send_goal(ControlClientPtr& client, control_msgs::FollowJointTrajectoryGoal goal);
void publish_velocity(ros::Publisher& publisher, geometry_msgs::Twist message, ros::Rate loop_rate);


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

/* Print the overt attention configuration */
void print_configuration(string platform, string camera, int realignment_threshold, int x_offset_to_head_yaw, int y_offset_to_head_pitch, string simulator_topics, string robot_topics, string topics_filename, bool debug_mode);

/* Read the overt attention configuration */
/* 
 *   Function to read the overt attention configuration.
 *   The configuration file contains the platform, camera, realignment threshold, x offset to head yaw, y offset to head pitch, simulator topics, robot topics, topics filename, and debug mode.
 *   The function reads the configuration file and sets the values for the specified parameters.
 * 
 * @param:
 *   platform: the platform value
 *   camera: the camera value
 *   realignment_threshold: the realignment threshold value
 *   x_offset_to_head_yaw: the x offset to head yaw value
 *   y_offset_to_head_pitch: the y offset to head pitch value
 *   simulator_topics: the simulator topics value
 *   robot_topics: the robot topics value
 *   topics_filename: the topics filename value
 *   debug_mode: the debug mode value
 * 
 * @return:
 *   0 if the configuration file is read successfully
 *   1 if the configuration file is not read successfully
 */
int read_configuration_file(string* platform, string* camera, int* realignment_threshold, int* x_offset_to_head_yaw, int* y_offset_to_head_pitch, string* simulator_topics, string* robot_topics, string* topics_filename, bool* debug_mode);

/* 
 *   Function that returns the head angles given the head end-effector position (BottomCamera)
 *   The function calculates the head yaw and head pitch angles of the head chain
 *
 * @param:
 *   camera_x: the x position of the head end-effector
 *   camera_y: the y position of the head end-effector
 *   camera_z: the z position of the head end-effector
 *   head_yaw: the head yaw angle to be updated
 *   head_pitch: the head pitch angle to be updated
 *
 * @return:
 *   None
 */
void get_head_angles(double camera_x, double camera_y, double camera_z, double* head_yaw, double* head_pitch);

/*  
 *   Function to move the head to a position specified by the head pitch and head yaw angles
 *   The function moves the head to the specified position using the control client
 *
 *   @param:
 *       head_topic: the topic for the head
 *       head_pitch: the pitch angle of the head
 *       head_yaw: the yaw angle of the head
 *       gesture_duration: the duration of the gesture
 *       debug: boolean to indicate if debugging information should be printed
 *
 *   @return:
 *       None
 */
void move_robot_head(std::string head_topic, double head_pitch, double head_yaw, double gesture_duration, bool debug);

void move_robot_head_wheels(std::string head_topic, double head_pitch, double head_yaw, double gesture_duration, 
                            bool rotate_robot, double angle_radians, ros::Publisher velocity_publisher, bool debug);

/*  
 *   Function to execute the location attention
 *   The function moves the robot's head to look at the specified point in the environment.
 *   
 * @param:
 *   point_x: the x coordinate of the point to look at
 *   point_y: the y coordinate of the point to look at
 *   point_z: the z coordinate of the point to look at
 *   topics_file: the topics file
 *   debug: the debug mode
 * 
 * @return:
 *   1 if the attention is executed successfully
 */
int location_attention(float point_x, float point_y, float point_z, string topics_file, ros::Publisher velocity_publisher, bool debug);

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
 *   Function to move an actuator to a position when using linear interpolation
 *   The actuator is moved using the control client to the specified position
 *
 *   @param:
 *       client: the control client for the actuator
 *       joint_names: vector containing the joint names of the actuator
 *       duration: the duration of the movement
 *       positions: vector containing the joint angles of the position to move the actuator to
 *
 *   @return:
 *       None
 */
void move_to_position(ControlClientPtr& client, const std::vector<std::string>& joint_names, double duration, 
                        std::vector<double> positions);

void move_robot_head_wheels_to_position(ControlClientPtr& head_client, const std::vector<std::string>& joint_names, double duration, 
                        std::vector<double> positions, bool rotate_robot, double angle_radians, ros::Publisher velocity_publisher, bool debug);

/*  Function to move the arm to a position using the minimum-jerk model of biological motion
 *   The function moves the arm to the specified position using the control client
 *
 *   @param:
 *       client: the control client for the arm
 *       joint_names: vector containing the joint names of the arm
 *       duration: vector containing the duration of the movement
 *       gesture_duration: the duration of the gesture
 *       positions: vector containing the joint angles of the position to move the arm to
 *       velocities: vector containing the joint velocities of the position to move the arm to
 *       accelerations: vector containing the joint accelerations of the position to move the arm to
 *
 *   @return:
 *       None
 */
void move_to_position_biological_motion(ControlClientPtr& client, const std::vector<std::string>& joint_names, 
                                        double gesture_duration, std::vector<double> duration, 
                                        std::vector<std::vector<double>> positions, std::vector<std::vector<double>> velocities, 
                                        std::vector<std::vector<double>> accelerations);

void move_robot_head_wheels_to_position_biological_motion(ControlClientPtr& client, const std::vector<std::string>& joint_names, 
                                        double gesture_duration, std::vector<double> duration, 
                                        std::vector<std::vector<double>> positions, std::vector<std::vector<double>> velocities, 
                                        std::vector<std::vector<double>> accelerations,
                                        bool rotate_robot, double angle_radians, ros::Publisher velocity_publisher, bool debug);

/*
 * FUnction to move the robot to a specified position where a face is in the center of the camera
 *
 * @param:
 *      topics_file: the topics file
 *     debug: the debug mode
 * 
 * @return:
 *      1 if the attention is executed successfully
 */ 
int reactive_attention(std::string topics_file, bool debug);

int social_attention(std::string topics_file, int realignment_threshold, ros::Publisher velocity_publisher, bool debug);

int sound_attention(std::string topics_file, bool debug);

int scanning_attention(string topics_file, ros::Publisher velocity_publisher, bool debug);

int seeking_attention(string topics_file, int realignment_threshold, ros::Publisher velocity_publisher, ros::Publisher overt_attention_engagement_status_pub, bool debug);

/* 
 *   Function to convert radians to degrees
 *   This function converts the angle in radians to degrees
 *
 * @param:
 *   radians: the angle in radians
 *
 * @return:
 *   the angle in degrees
 */
double degrees(double radians);

/* 
 *   Function to convert degrees to radians
 *   This function converts the angle in degrees to radians
 *
 * @param:
 *   degrees: the angle in degrees
 *
 * @return:
 *   the angle in radians
 */
double radians(double degrees);
 
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

/*  
 *   Function to compute the angle changes required refocus the robot head on a point it's FOV
 *
 *   @param:
 *       center_x: x coordinate of the point of interest
 *       center_y: y coordinate of the point of interest
 *       image_width: width of the original image
 *       image_height: height of the original image
 *       head_yaw: the required head yaw angle adjustment
 *       head_pitch: the required head pitch angle adjustment
 *
 *   @return:
 *       None
 */
struct AngleChange {
    double delta_yaw;
    double delta_pitch;
};

AngleChange get_angles_from_pixel(double center_x, double center_y, double image_width, double image_height, double head_yaw, double head_pitch);
void extractConfig(std::map<std::string, std::string>& configMap);

#endif // OVERT_ATTENTION_H_