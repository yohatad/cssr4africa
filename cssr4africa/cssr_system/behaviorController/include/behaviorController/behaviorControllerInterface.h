/* behaviorControllerInterface.h - interface file for behaviorControllerApplication and behaviorControllerImplementation.
*
* Author: Tsegazeab Taye Tefferi
* Date: April 08, 2025
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

#ifndef BEHAVIOR_CONTROLLER_INTERFACE_H
#define BEHAVIOR_CONTROLLER_INTERFACE_H

#include "ros/ros.h"
#include <ros/package.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>

#include <fstream>
#include <string>
#include <vector>
#include <stdio.h>
#include <stdexcept>

#include "utilities/cultureKnowledgeBaseInterface.h"
#include "utilities/environmentKnowledgeBaseInterface.h"

#include "cssr_system/faceDetectionData.h" 
#include "cssr_system/overtAttentionMode.h"

#include "cssr_system/animateBehaviorSetActivation.h"
#include "cssr_system/gestureExecutionPerformGesture.h"
#include "cssr_system/overtAttentionSetMode.h"
#include "cssr_system/robotLocalizationResetPose.h"
#include "cssr_system/robotNavigationSetGoal.h"
#include "cssr_system/speechEventSetEnabled.h"
#include "cssr_system/speechEventSetLanguage.h"
#include "cssr_system/tabletEventPromptAndGetResponse.h"
#include "cssr_system/textToSpeechSayText.h"

/* Configuration variables */
extern bool verboseMode;
extern bool asrEnabled;
extern std::string missionLanguage;
extern bool testMode;

extern std::string nodeName;


//Node Handler
extern ros::NodeHandle* nh;

/* Returns true if all the topics in a list are available*/
bool checkTopics(std::vector<std::string>& topicsList);

/* Returns true if all the services in a list are available*/
bool checkServices(std::vector<std::string>& servicesList);

/* Returns the value of a key from the configuration file. */
std::string getValueFromConfig(const std::string &key);

/* 
    Returns a behavior tree of type BT::Tree from the scenario specification file
    Scenario specification file must be placed in the data directory
*/
BT::Tree initializeTree(std::string scenario);

/* Returns the current language from the knowledge base*/
std::string getMissionLanguage();
#endif