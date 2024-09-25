/* gestureExecutionApplication.cpp
*
* Copyright (C) 2023 CSSR4Africa Consortium
*
* This project is funded by the African Engineering and Technology Network (Afretec)
* Inclusive Digital Transformation Research Grant Programme.
*
* Website: www.cssr4africa.org
*
* This program comes with ABSOLUTELY NO WARRANTY.
*
* <detailed functional description>
* This module is responsible for hosting the service that executes the gestures on the robot. 
* The module receives the gesture type, gesture ID, gesture duration, bow/nod angle, 
* and the location in the world to pay attention/point to in x, y, z coordinates.
* The module then executes the gesture based on the received parameters. 
*
* The module supports the execution of deictic, iconic, symbolic, bow, and nod gestures.
* The iconic gestures currently supported are welcome and wave (goodbye) gestures. 
* 
* The module also supports the selection of the implementation platform (simulator or robot) 
* and the interpolation type (linear or biological motion).
* 
* The module is implemented as a ROS service that receives the gesture parameters 
* and returns the status of the gesture execution.
*
* The gestures could either be executed using linear velocity interpolation or a model of biological motion (minimum-jerk model).
*
* The module subscribes to the /sensor_msgs/joint_states topic to receive the joint states of the robot.
* The module also subscribes to the /robotLocalization/pose topic to receive the coordinates of the robot in the world.
* 
* The module is implemented in C++ and uses the ROS libraries for communication with the robot.
* The module is part of the CSSR4A package and is used to execute gestures on the robot.
*
*
*
...
* Libraries
* Standard libraries
- std::string, std::vector, std::fstream, std::pow, std::sqrt, std::abs
* ROS libraries
- ros/ros.h, ros/package.h, actionlib/client/simple_action_client.h, control_msgs/FollowJointTrajectoryAction.h, geometry_msgs/Twist.h
*
*
*
...
* Parameters
*
* Command-line Parameters
*
* None
*
...
* Configuration File Parameters

* Key                   |     Value 
* --------------------- |     -------------------
* platform                    robot
* interpolation               biological
* gestureDescriptors          gestureDescriptors.dat
* simulatorTopics             simulatorTopics.dat
* robotTopics                 pepperTopics.dat
* verboseMode                 true
*
*
*
...
* Subscribed Topics and Message Types
*
* /sensor_msgs/joint_states
* /robotLocalization/pose
...
* Published Topics and Message Types
* 
* /pepper_dcm/Head_controller/follow_joint_trajectory           trajectory_msgs/JointTrajectory
* /pepper_dcm/RightArm_controller/follow_joint_trajectory       trajectory_msgs/JointTrajectory
* /pepper_dcm/LeftArm_controller/follow_joint_trajectory        trajectory_msgs/JointTrajectory
* /pepper_dcm/RightHand_controller/follow_joint_trajectory      trajectory_msgs/JointTrajectory
* /pepper_dcm/LeftHand_controller/follow_joint_trajectory       trajectory_msgs/JointTrajectory
* /pepper_dcm/Pelvis_controller/follow_joint_trajectory         trajectory_msgs/JointTrajectory
* /pepper_dcm/cmd_moveto                                        geometry_msgs/Twist

* /pepper/Head_controller/follow_joint_trajectory               trajectory_msgs/JointTrajectory
* /pepper/RightArm_controller/follow_joint_trajectory           trajectory_msgs/JointTrajectory
* /pepper/LeftArm_controller/follow_joint_trajectory            trajectory_msgs/JointTrajectory
* /pepper/Pelvis_controller/follow_joint_trajectory             trajectory_msgs/JointTrajectory
* /pepper/cmd_vel                                               geometry_msgs/Twist
*
*
*
...
* Input Data Files
*
* pepperTopics.dat
* simulatorTopics.dat
* gestureDescriptors.dat
*
*
*
...
* Output Data Files
*
* None
*
*
*
...
* Configuration Files
*
* gestureExecutionConfiguration.ini
*
*
*
...
* Example Instantiation of the Module
*
* rosrun gestureExecution perform_gesture
*
*
...
*
* The clients can invoke the service by providing the gesture type, gesture ID, gesture duration, bow_nod angle, 
* and the location in the world to pay attention/point to in x, y, z coordinates.
* The service will execute the gesture based on the received parameters and return the status of the gesture execution.
*
* An example of calling the service is shown below:
* ----- rosservice call /perform_gesture -- deictic 01 3000 25 3.6 2.5 0.82
* This will execute a pointing gesture with a duration of 3000 ms, and the location in the world to point to in x, y, z coordinates.
*
* ----- rosservice call /perform_gesture -- bow 01 3000 25 3.6 2.5 0.82
* This will execute a pointing gesture with a duration of 3000 ms, and bow at an angle of 45 degrees.
*
*
*
*
* Author: Adedayo Akinade, Carnegie Mellon University Africa
* Email: aakinade@andrew.cmu.edu
* Date: April 12, 2024
* Version: v1.0
*
*/

#include "gestureExecution/gestureExecutionInterface.h"

using namespace std;

bool verbose_mode = false;

string implementation_platform;
string interpolation_type;
string gesture_descriptors_config_filename;
string simulator_topics_filename;
string robot_topics_filename;
string verbose_mode_input;

// Publisher for the velocity commands
ros::Publisher velocity_publisher;

// Iconic gestures descriptors table.
// Each row contains the gesture_id for both arms. Repeated values means its only one arm gesture
std::vector<std::vector<string>> gesture_descriptors_table = {{"01", "02"},
                                                            {"02", "01"},
                                                            {"03", ""}};

bool execute_gesture(cssr_system::perform_gesture::Request  &service_request, cssr_system::perform_gesture::Response &service_response){
    int execution_status = 0;                       //Stores the status of the gesture execution
    bool debug = true;                              //Debug mode flag
    string topics_file_name = "";                   //Stores the name of the topics file to be used

    // Read robot location input from data file
    read_robot_pose_input(robot_pose);       //Replace with subscriber to /robotLocalization/pose topic

    // Initialize the server: Read configuration file and set the implementation platform, interpolation type, and topics file name
    initialize_server(&implementation_platform, &interpolation_type, &gesture_descriptors_config_filename, &simulator_topics_filename, &robot_topics_filename, &verbose_mode_input, &verbose_mode);

    if (verbose_mode){
        printf("Request to service: \
                \n\tgesture_type\t\t= %s, \n\tgesture_id\t\t= %d, \n\tgesture_duration\t= %ld ms, \n\tbow_nod_angle\t\t= %d degrees, \
                \n\tlocation_x\t\t= %f, \n\tlocation_y\t\t= %f, \n\tlocation_z\t\t= %f\n\n",\
                service_request.gesture_type.c_str(), service_request.gesture_id, service_request.gesture_duration, service_request.bow_nod_angle, \
                service_request.location_x, service_request.location_y, service_request.location_z);
    }

    // Extract request parameters
    string gesture_type = service_request.gesture_type;
    uint8_t gesture_id = service_request.gesture_id;
    int gesture_duration = service_request.gesture_duration;
    int bow_nod_angle = service_request.bow_nod_angle;
    float point_location_x = service_request.location_x;
    float point_location_y = service_request.location_y;
    float point_location_z = service_request.location_z;

    int interpolation_mode = LINEAR_INTERPOLATION;              // Stores the interpolation mode to be used

    service_response.gesture_success = execution_status;                     // Set the default response to the service

    /* Check if the implementation platform is supported and set the topics file name */
    {
        if (implementation_platform == "simulator"){
            topics_file_name = simulator_topics_filename;
        }
        else if (implementation_platform == "robot"){
            topics_file_name = robot_topics_filename;
        }
        else{
            if(verbose_mode){
                printf("Implementation platform not supported. Supported implementation platforms are: simulator and robot\n");
            }
            return true;
        }
    }
    // printf("Topics file is: %s\n", topics_file_name.c_str());

    /* Check if the interpolation type is supported and set the interpolation mode */
    {
        if (interpolation_type == "linear"){
            interpolation_mode = LINEAR_INTERPOLATION;
        }
        else if (interpolation_type == "biological"){
            interpolation_mode = BIOLOGICAL_MOTION;
        }
        else{
            if(verbose_mode){
                printf("Interpolation type not supported. Supported interpolation types are: linear and biological_motion\n");
            }
            return true;
        }
    }

    /* -----Main gesture execution logic------ */
 
    /* Deictic Gestures Execution */
    if((gesture_type == DEICTIC_GESTURES) || (gesture_type == DIECTIC_GESTURES)){
        if (verbose_mode) {
            printf("Deictic gesture to be executed\n");
        }

        execution_status = deictic_gesture(point_location_x, point_location_y, point_location_z, gesture_duration, topics_file_name, interpolation_mode, velocity_publisher, verbose_mode);
        if(verbose_mode){
            if (execution_status == 1){
                printf("Deictic gesture executed successfully\n");
            }
            else{
                printf("Deictic gesture execution failed\n");
            }
        }
    }

    /* Iconic Gestures Execution */
    else if(gesture_type == ICONIC_GESTURES){
        if (verbose_mode) {
            printf("Iconic gesture to be executed\n");
        }

        string iconic_gesture_type;
        string iconic_gesture_id;
        string gesture_arm = "RArm";
        int number_of_waypoints;
        string joint_angles;

        std::vector<std::vector<double>> gesture_waypoints;
        std::vector<std::vector<double>> gesture_waypoints_arm_1;
        std::vector<std::vector<double>> gesture_waypoints_arm_2;

        // Read gesture descriptors configuration file
        std::vector<std::vector<string>> gesture_descriptors_config;
        int gesture_descriptors_config_read_status = 0;
        gesture_descriptors_config_read_status = read_gesture_descriptors_config(gesture_descriptors_config_filename, gesture_descriptors_config);
        if(gesture_descriptors_config_read_status != 0){
            if(verbose_mode){
                printf("Gesture descriptors configuration file read failed\n");
                return true;
            }
        }

        // Analyse the requested ID to determine if it is one or two-armed gesture from the descriptor table
        string second_arm_ID = "";
        for (int i = 0; i < gesture_descriptors_table.size(); i++){
            if (gesture_id == std::stoi(gesture_descriptors_table[i][0])){
                second_arm_ID = gesture_descriptors_table[i][1];
                break;
            }
        }

        string gesture_arm_1 = "";
        string gesture_arm_2 = "";
        string gesture_descriptors_filename = "";
        int number_of_waypoints_arm_1 = 0;
        int number_of_waypoints_arm_2 = 0;
        string joint_angles_arm_1 = "";
        string joint_angles_arm_2 = "";

        int angles_extracted_arm_1 = 1;
        int angles_extracted_arm_2 = 1;

        bool open_right_hand = true;
        bool open_left_hand = true;

        extract_info_from_ID(gesture_id, gesture_descriptors_config, &gesture_descriptors_filename, &gesture_arm_1);
        read_gesture_descriptors(gesture_descriptors_filename, &iconic_gesture_type, &iconic_gesture_id, &number_of_waypoints_arm_1, &joint_angles_arm_1);
        // Extract joint angles from the gesture descriptors
        angles_extracted_arm_1 = extract_joint_angles(gesture_descriptors_filename, joint_angles_arm_1, number_of_waypoints_arm_1, gesture_waypoints_arm_1, verbose_mode);


        if (second_arm_ID != ""){
            extract_info_from_ID(std::stoi(second_arm_ID), gesture_descriptors_config, &gesture_descriptors_filename, &gesture_arm_2);
            read_gesture_descriptors(gesture_descriptors_filename, &iconic_gesture_type, &iconic_gesture_id, &number_of_waypoints_arm_2, &joint_angles_arm_2);
            // Extract joint angles from the gesture descriptors
            angles_extracted_arm_2 = extract_joint_angles(gesture_descriptors_filename, joint_angles_arm_2, number_of_waypoints_arm_2, gesture_waypoints_arm_2, verbose_mode);
        
        }

        // Execute the iconic gesture
        if(angles_extracted_arm_1 && angles_extracted_arm_2){
            execution_status = iconic_gestures(gesture_arm_1, gesture_waypoints_arm_1, gesture_arm_2, gesture_waypoints_arm_2, open_right_hand, open_left_hand, gesture_duration, topics_file_name, interpolation_mode, verbose_mode);
        }
        if(verbose_mode){
            if (execution_status == 1){
                printf("Iconic gesture executed successfully\n");
            }
            else{
                printf("Iconic gesture execution failed\n");
            }
        }

    }

    /* Symbolic Gestures Execution */
    else if((gesture_type == SYMBOLIC_GESTURES)){
        if (verbose_mode) {
            printf("Symbolic gesture to be executed but not implemented yet\n");
        }

    }
    
    /* Bowing Gesture Execution */
    else if(gesture_type == BOWING_GESTURE){
        if (verbose_mode) {
            printf("Bowing gesture to be executed\n");
        }

        execution_status = bowing_gesture(bow_nod_angle, gesture_duration, topics_file_name, interpolation_mode, verbose_mode);
        printf("--------------------------------------------------------------------------------------------------------------------------------------------------\n");
        if(verbose_mode){
            if (execution_status == 1){
                printf("Bowing gesture executed successfully\n");
            }
            else{
                printf("Bowing gesture execution failed\n");
            }
        }
        // prompt_and_exit(execution_status);
    }

    /* Nodding Gesture Execution */
    else if(gesture_type == NODDING_GESTURE){
        if (verbose_mode) {
            printf("Nodding gesture to be executed\n");
        }

        execution_status = nodding_gesture(bow_nod_angle, gesture_duration, topics_file_name, interpolation_mode, verbose_mode);
        printf("--------------------------------------------------------------------------------------------------------------------------------------------------\n");
        if(debug){
            if (execution_status == 1){
                printf("Nodding gesture executed successfully\n");
            }
            else{
                printf("Nodding gesture execution failed\n");
            }
        }
    }

    /* Gesture type not supported */
    else{
        printf("--------------------------------------------------------------------------------------------------------------------------------------------------\n");
        printf("Gesture type not supported. Supported gesture types are: deictic, iconic, symbolic, bow, and nod.\n");
        // prompt_and_exit(1);
    }

    service_response.gesture_success = execution_status;
    ROS_INFO("Response from service: [%ld]\n", (long int)service_response.gesture_success);
    printf("------------------------------------------------------------------------------------------------------------------------------------------------\n");
    return true;
}

void joint_states_message_received(const sensor_msgs::JointState& msg) {
    // std::vector<std::string> jointNames = {"HeadPitch", "HeadYaw"};
    head_joint_states[0] = msg.position[HEAD_PITCH_IDX];
    head_joint_states[1] = msg.position[HEAD_YAW_IDX];

    // std::vector<std::string> jointNames = {"HipPitch", "HipRoll", "KneePitch"};
    leg_joint_states[0] = msg.position[HIP_PITCH_IDX];
    leg_joint_states[1] = msg.position[HIP_ROLL_IDX];
    leg_joint_states[2] = msg.position[KNEE_PITCH_IDX];

    // std::vector<std::string> jointNames = {"LShoulderPitch", "LShoulderRoll", "LElbowRoll", "LElbowYaw", "LWristYaw"};
    left_arm_joint_states[0] = msg.position[LEFT_SHOULDER_PITCH_IDX];
    left_arm_joint_states[1] = msg.position[LEFT_SHOULDER_ROLL_IDX];
    left_arm_joint_states[2] = msg.position[LEFT_ELBOW_ROLL_IDX];
    left_arm_joint_states[3] = msg.position[LEFT_ELBOW_YAW_IDX];
    left_arm_joint_states[4] = msg.position[LEFT_WRIST_YAW_IDX];

    // std::vector<std::string> jointNames = {"RShoulderPitch", "RShoulderRoll",  "RElbowRoll", "RElbowYaw", "RWristYaw"};
    right_arm_joint_states[0] = msg.position[RIGHT_SHOULDER_PITCH_IDX];
    right_arm_joint_states[1] = msg.position[RIGHT_SHOULDER_ROLL_IDX];
    right_arm_joint_states[2] = msg.position[RIGHT_ELBOW_ROLL_IDX];
    right_arm_joint_states[3] = msg.position[RIGHT_ELBOW_YAW_IDX];
    right_arm_joint_states[4] = msg.position[RIGHT_WRIST_YAW_IDX];
}

int main(int argc, char **argv){
    ros::init(argc, argv, "gestureExecution");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("joint_states", 1000, &joint_states_message_received);
    ros::ServiceServer service = n.advertiseService("/gestureExecution/perform_gesture", execute_gesture);

    velocity_publisher = n.advertise<geometry_msgs::Twist>("/pepper_dcm/cmd_moveto", 1000, true);


    ROS_INFO("Gesture Execution Server Ready\n");


    ros::spin();

    return 0;
}