/* perform_gesture_server.cpp
*
* <detailed functional description>
* This module is responsible for hosting the service that executes the gestures on the robot. 
* The module receives the gesture type, gesture ID, gesture duration, bow angle, nod angle, and the location in the world to pay attention/point to in x, y, z coordinates.
* The module then executes the gesture based on the received parameters. The module supports the execution of deictic, iconic, and symbolic gestures.
* The specific gestures currently supported are pointing, bowing, and nodding gestures. 
* The module also supports the selection of the implementation platform (simulator or robot) and the interpolation type (linear or biological motion).
* 
* The module is implemented as a ROS service that receives the gesture parameters and returns the status of the gesture execution.
* The gestures could either be executed using linear velocity interpolation or a model of biological motion (Minimum Jerk Model).
* The module also subscribes to the joint_states topic to receive the joint states of the robot.
* 
* The module is implemented in C++ and uses the ROS libraries for communication with the robot.
* The module is part of the gesture execution package and is used to execute gestures on the robot.
*
*

...
* Libraries
* Standard libraries
- std::string, std::vector, std::thread, std::fstream, std::cout, std::endl, std::cin, std::pow, std::sqrt, std::abs
* ROS libraries
- ros/ros.h, ros/package.h, actionlib/client/simple_action_client.h, control_msgs/FollowJointTrajectoryAction.h, geometry_msgs/Twist.h

...
* Parameters
*
* Command-line Parameters
*
* None
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

...
* Subscribed Topics and Message Types
*
* /sensor_msgs/joint_states
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
...
* Input Data Files
*
* pepperTopics.dat
* simulatorTopics.dat
...
* Output Data Files
*
* None
...
* Configuration Files
*
* gestureExecutionConfiguration.ini
...
* Example Instantiation of the Module
*
* rosrun gestureExecution perform_gesture_server
...
*
* The clients can call the service by providing the gesture type, gesture ID, gesture duration, bow angle, nod angle, and the location in the world to pay attention/point to in x, y, z coordinates.
* The service will execute the gesture based on the received parameters and return the status of the gesture execution.
* AN example of calling the service is shown below:
* ----- rosservice call /perform_gesture -- deictic 01 3000 25 25 3600 -3600 820
* This will execute a pointing gesture with a duration of 3000 ms, bow angle of 25 degrees, nod angle of 25 degrees, and the location in the world to point to in x, y, z coordinates.


*
* Author: Adedayo Akinade, Carnegie Mellon University Africa
* Email: aakinade@andrew.cmu.edu
* Date: April 12, 2024
* Version: v1.0
*
*/



#include "ros/ros.h"
#include "gesture_execution/perform_gesture.h"
#include "gesture_execution/gestureExecution.h"
#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <iomanip> // 

using namespace std;

bool verbose_mode = false;

string implementationPlatform;
string interpolationType;
string gestureDescriptorsFileName;
string simulatorTopicsFileName;
string robotTopicsFileName;
string verboseModeInput;

// extern std::vector<double> legJointStates;
// extern std::vector<double> headJointStates;
// extern std::vector<double> rArmJointStates;
// extern std::vector<double> lArmJointStates;

std::vector<double> legJointStates = {0.0, 0.0, 0.0};
std::vector<double> headJointStates = {0.0, 0.0};
std::vector<double> rArmJointStates = {0.0, 0.0, 0.0, 0.0, 0.0};
std::vector<double> lArmJointStates = {0.0, 0.0, 0.0, 0.0, 0.0};

bool execute_gesture(gesture_execution::perform_gesture::Request  &req, gesture_execution::perform_gesture::Response &res){
    int execution_status = 0;
    bool debug = true;
    string topics_file_name = "";

    initialize_server(&implementationPlatform, &interpolationType, &gestureDescriptorsFileName, &simulatorTopicsFileName, &robotTopicsFileName, &verboseModeInput, &verbose_mode);

    if (verbose_mode){
        ROS_INFO("Request to service: \
                \n\tgesture_type\t\t= %s, \n\tgesture_id\t\t= %d, \n\tgesture_duration\t= %ld ms, \n\tbow_angle\t\t= %d degrees, \
                \n\tnod_angle\t\t= %d degrees, \n\tlocation_x\t\t= %f, \n\tlocation_y\t\t= %f, \n\tlocation_z\t\t= %f",\
                req.gesture_type.c_str(), req.gesture_id, req.gesture_duration, req.bow_angle, req.nod_angle, \
                req.location_x, req.location_y, req.location_z);
    }

    // Extract request parameters
    string gesture_type = req.gesture_type;
    int gesture_id = req.gesture_id;
    int gesture_duration = req.gesture_duration;
    int bow_angle = req.bow_angle;
    int nod_angle = req.nod_angle;
    float point_location_x = req.location_x;
    float point_location_y = req.location_y;
    float point_location_z = req.location_z;

    int interpolation_mode = LINEAR_INTERPOLATION;

    if (implementationPlatform == "simulator"){
        if (verbose_mode){
            printf("Simulator platform selected\n");
            topics_file_name = simulatorTopicsFileName;
        }
    }
    else if (implementationPlatform == "robot"){
        if (verbose_mode){
            printf("Robot platform selected\n");
            topics_file_name = robotTopicsFileName;
        }
    }
    else{
        printf("Implementation platform not supported. Supported implementation platforms are: simulator and robot\n");
        // prompt_and_exit(1);
    }
    printf("Topics file is: %s\n", topics_file_name.c_str());

    if (interpolationType == "linear"){
        if (verbose_mode){
            printf("Linear interpolation selected\n");
        }
        interpolation_mode = LINEAR_INTERPOLATION;
    }
    else if (interpolationType == "biological"){
        if (verbose_mode){
            printf("Biological motion interpolation selected\n");
        }
        interpolation_mode = BIOLOGICAL_MOTION;
    }
    else{
        printf("Interpolation type not supported. Supported interpolation types are: linear and biological_motion\n");
        // prompt_and_exit(1);
    }

    
    /* Deictic Gestures Execution */
    if((gesture_type == DEICTIC_GESTURES) || (gesture_type == DIECTIC_GESTURES)){
        // if (verbose_mode) {
            printf("Deictic gesture ");
        // }

        if (gesture_id == 01){
            // if(verbose_mode){
                printf("ID is 01; Pointing gesture to be executed\n");
            // }

            execution_status = pointing_gesture(req.location_x, req.location_y, req.location_z, gesture_duration, topics_file_name, interpolation_mode, debug);
            // if(verbose_mode){
                printf("--------------------------------------------------------------------------------------------------------------------------------------------------\n");
                if (execution_status == 1){
                    printf("Pointing gesture executed successfully\n");
                }
                else{
                    printf("Pointing gesture execution failed\n");
                }
            // }

            // prompt_and_exit(execution_status);
        }
        else{
            printf("ID not supported. Supported deictic gesture IDs are: 01\n");
            // prompt_and_exit(1);
        }
    }

    else if((gesture_type == ICONIC_GESTURES) || (gesture_type == SYMBOLIC_GESTURES)){
        if (debug) {
            if (gesture_type == ICONIC_GESTURES){
                printf("Iconic gesture ");
            }
            else{
                printf("Symbolic gesture ");
            }
        }

        if (gesture_id == 02){
            if(debug){
                printf("ID is 02; Bowing gesture to be executed\n");
            }

            execution_status = bowing_gesture(bow_angle, gesture_duration, topics_file_name, interpolation_mode, verbose_mode);
            if(verbose_mode){
                printf("--------------------------------------------------------------------------------------------------------------------------------------------------\n");
                if (execution_status == 1){
                    printf("Bowing gesture executed successfully\n");
                }
                else{
                    printf("Bowing gesture execution failed\n");
                }
            }

            // prompt_and_exit(execution_status);

        }

        else if  (gesture_id == 03){
            if(debug){
                printf("ID is 03; Nodding gesture to be executed\n");
            }

            execution_status = nodding_gesture(nod_angle, gesture_duration, topics_file_name, interpolation_mode, verbose_mode);
            if(debug){
                printf("--------------------------------------------------------------------------------------------------------------------------------------------------\n");
                if (execution_status == 1){
                    printf("Nodding gesture executed successfully\n");
                }
                else{
                    printf("Nodding gesture execution failed\n");
                }
            }

            // prompt_and_exit(execution_status);
        }

        // else if  (gesture_id == 04){
        //     if(verbose_mode){
        //         printf("ID is 04; Waving gesture to be executed\n");
        //     }

        //     execution_status = waving_gesture(robot_location_input, verbose_mode);
        //     if(verbose_mode){
        //         if (execution_status == 1){
        //             printf("--------------------------------------------------------------------------------------------------------------------------------------------------\n");
        //             printf("Waving gesture executed successfully\n");
        //         }
        //         else{
        //             printf("Waving gesture execution failed\n");
        //         }
        //     }

        //     // prompt_and_exit(execution_status);
        // }

        else{
            // printf("--------------------------------------------------------------------------------------------------------------------------------------------------\n");
            printf("ID not supported. Supported iconic gesture IDs are: 02 - Bow, 03 - Nod.\n");
            // prompt_and_exit(1);
        }
    }
    
    /* Gesture type not supported */
    else{
        printf("--------------------------------------------------------------------------------------------------------------------------------------------------\n");
        printf("Gesture type not supported. Supported gesture types are: deictic, iconic, and symbolic\n");
        // prompt_and_exit(1);
    }


    res.gesture_success = execution_status;
    ROS_INFO("Response from service: [%ld]\n", (long int)res.gesture_success);
    return true;
}

void jointStatesMessageReceived(const sensor_msgs::JointState& msg) {
    // std::vector<std::string> jointNames = {"HeadPitch", "HeadYaw"};
    headJointStates[0] = msg.position[HeadPitchIdx];
    headJointStates[1] = msg.position[HeadYawIdx];

    // std::vector<std::string> jointNames = {"HipPitch", "HipRoll", "KneePitch"};
    legJointStates[0] = msg.position[HipPitchIdx];
    legJointStates[1] = msg.position[HipRollIdx];
    legJointStates[2] = msg.position[KneePitchIdx];

    // std::vector<std::string> jointNames = {"LShoulderPitch", "LShoulderRoll", "LElbowRoll", "LElbowYaw", "LWristYaw"};
    lArmJointStates[0] = msg.position[LShoulderPitchIdx];
    lArmJointStates[1] = msg.position[LShoulderRollIdx];
    lArmJointStates[2] = msg.position[LElbowRollIdx];
    lArmJointStates[3] = msg.position[LElbowYawIdx];
    lArmJointStates[4] = msg.position[LWristYawIdx];

    // std::vector<std::string> jointNames = {"RShoulderPitch", "RShoulderRoll",  "RElbowRoll", "RElbowYaw", "RWristYaw"};
    rArmJointStates[0] = msg.position[RShoulderPitchIdx];
    rArmJointStates[1] = msg.position[RShoulderRollIdx];
    rArmJointStates[2] = msg.position[RElbowRollIdx];
    rArmJointStates[3] = msg.position[RElbowYawIdx];
    rArmJointStates[4] = msg.position[RWristYawIdx];
}

int main(int argc, char **argv){
    ros::init(argc, argv, "perform_gesture_server");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("joint_states", 1000, &jointStatesMessageReceived);
    ros::ServiceServer service = n.advertiseService("perform_gesture", execute_gesture);
    ROS_INFO("Ready to execute gesture.");


    ros::spin();

    return 0;
}