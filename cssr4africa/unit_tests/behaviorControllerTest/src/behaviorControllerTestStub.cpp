/* behaviorControllerTestStub.cpp   Source code for the simulated services (stubs)
 *
 * Author: Tsegazeab Taye Tefferi
 * Date: April 25, 2025
 * Version: 1.0
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
 */

#include <behaviorControllerTest/behaviorControllerTestUtilitiesInterface.h>

#include "unit_tests/animateBehaviorSetActivation.h"
#include "unit_tests/gestureExecutionPerformGesture.h"
#include "unit_tests/overtAttentionSetMode.h"
#include "unit_tests/robotLocalizationResetPose.h"
#include "unit_tests/robotNavigationSetGoal.h"
#include "unit_tests/speechEventSetEnabled.h"
#include "unit_tests/speechEventSetLanguage.h"
#include "unit_tests/tabletEventPromptAndGetResponse.h"
#include "unit_tests/textToSpeechSayText.h"


bool animateBehaviorSetActivationHandler(unit_tests::animateBehaviorSetActivation::Request &req, unit_tests::animateBehaviorSetActivation::Response &res)
{

    ROS_INFO_STREAM("[/animateBehavior/setActivation]: Set state: " << req.state);

    // do success or failure
    std::string success = hasSucceeded() ?  "1": "0";
    res.success = success;
    return true;
}

bool gestureExecutionPerformGestureHandler(unit_tests::gestureExecutionPerformGesture::Request &req, unit_tests::gestureExecutionPerformGesture::Response &res)
{

    ROS_INFO_STREAM("[/gestureExecution/perform_gesture]: Performing Gesture:"
                    "\n\t type: " << req.gesture_type << 
                    "\n\t id: " << req.gesture_id << 
                    "\n\t duration: " << req.gesture_duration << 
                    "\n\t bow_nod_angle: " << req.bow_nod_angle << 
                    "\n\t location_x: " << req.location_x << 
                    "\n\t location_y: " << req.location_y << 
                    "\n\t location_z: " << req.location_z
    );

    // do success or failure
    bool success = hasSucceeded();
    res.gesture_success = success;
    return true;
}

bool overtAttentionSetModeHandler(unit_tests::overtAttentionSetMode::Request &req, unit_tests::overtAttentionSetMode::Response &res)
{

    ROS_INFO_STREAM("[/overtAttention/set_mode]: Set mode: " << req.state);

    // do success or failure
    bool success = hasSucceeded();
    ros::param::set("/overtAttentionMode",req.state);
    res.mode_set_success = success;
    return true;
}

bool robotLocalizationResetPoseHandler(unit_tests::robotLocalizationResetPose::Request &req, unit_tests::robotLocalizationResetPose::Response &res)
{

    ROS_INFO_STREAM("[/robotNavigation/reset_pose]: Resetting Pose ");

    // do success or failure
    bool success = hasSucceeded();
    res.success = success;
    return true;
}

bool robotNavigationSetGoalHandler(unit_tests::robotNavigationSetGoal::Request &req, unit_tests::robotNavigationSetGoal::Response &res)
{

    ROS_INFO_STREAM("[/robotNavigation/set_goal]: Set goal: " << 
                    "\n\t goal_x: " << req.goal_x << 
                    "\n\t goal_y: " << req.goal_y << 
                    "\n\t goal_theta: " << req.goal_theta
    );

    // do success or failure
    bool success = hasSucceeded();
    res.navigation_goal_success = success;
    return true;
}

bool speechEventSetEnabledHandler(unit_tests::speechEventSetEnabled::Request &req, unit_tests::speechEventSetEnabled::Response &res)
{

    ROS_INFO_STREAM("[/speechEvent/set_enabled]: Set enabled: " << req.status);

    // do success or failure
    bool success = hasSucceeded();
    res.response = success;
    return true;
}

bool speechEventSetLanguageHandler(unit_tests::speechEventSetLanguage::Request &req, unit_tests::speechEventSetLanguage::Response &res)
{

    ROS_INFO_STREAM("[/speechEvent/set_language]: Set language: " << req.language);

    // do success or failure
    bool success = hasSucceeded();
    res.response = success;
    return true;
}

bool tabletEventPromptAndGetResponseHandler(unit_tests::tabletEventPromptAndGetResponse::Request &req, unit_tests::tabletEventPromptAndGetResponse::Response &res)
{

    ROS_INFO_STREAM("[/textToSpeech/say_text]: Prompt Message: '" << req.message);

    // do success or failure
    bool success = hasSucceeded();
    res.success = success;
    return true;
}

bool textToSpeechSayTextHandler(unit_tests::textToSpeechSayText::Request &req, unit_tests::textToSpeechSayText::Response &res)
{

    ROS_INFO_STREAM("[/textToSpeech/say_text]: Say text: '" << req.message << "' in '" << req.language << "'");

    // do success or failure
    bool success = hasSucceeded();
    res.success = success;
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "behaviorControllerTestStub");
    ros::NodeHandle nh;

    /* Retrieve the values from the configuration file       */
    /* Display the error and exit, if the file is unreadable */
    try
    {
        failureRate = std::stof(getValueFromConfig("failureRate"));
        testMode = (getValueFromConfig("testMode") == "true");
    }
    catch (const std::exception& e) {
        ROS_ERROR_STREAM("Fatal Error: "<<e.what());
        ros::shutdown();
        return 0;
    }

    if(testMode){
        failureRate = 0;
    }

    ROS_INFO_STREAM("Mode: "<<(testMode ? "Test" : "Development"));

    ROS_INFO_STREAM("Server failure Rate set at: "<<failureRate);


    /* Advertising the services */
    ros::ServiceServer animateBehaviorSetActivationServer = nh.advertiseService("/animateBehaviour/setActivation", animateBehaviorSetActivationHandler);
    ROS_INFO_STREAM(animateBehaviorSetActivationServer.getService()<<" Ready");
    
    ros::ServiceServer gestureExecutionPerformGestureServer = nh.advertiseService("/gestureExecution/perform_gesture", gestureExecutionPerformGestureHandler);
    ROS_INFO_STREAM(gestureExecutionPerformGestureServer.getService()<<" Ready");

    ros::ServiceServer overtAttentionSetModeServer = nh.advertiseService("/overtAttention/set_mode", overtAttentionSetModeHandler);
    ROS_INFO_STREAM(overtAttentionSetModeServer.getService()<<" Ready");

    ros::ServiceServer robotLocalizationResetPoseServer = nh.advertiseService("/robotLocalization/reset_pose", robotLocalizationResetPoseHandler);
    ROS_INFO_STREAM(robotLocalizationResetPoseServer.getService());

    ros::ServiceServer robotNavigationSetGoalServer = nh.advertiseService("/robotNavigation/set_goal", robotNavigationSetGoalHandler);
    ROS_INFO_STREAM(robotNavigationSetGoalServer.getService()<<" Ready");

    ros::ServiceServer speechEventSetEnabledServer = nh.advertiseService("/speechEvent/set_language", speechEventSetLanguageHandler);
    ROS_INFO_STREAM(speechEventSetEnabledServer.getService()<<" Ready");

    ros::ServiceServer speechEventSetLanguageServer = nh.advertiseService("/speechEvent/set_enabled", speechEventSetEnabledHandler);
    ROS_INFO_STREAM(speechEventSetLanguageServer.getService()<<" Ready");

    ros::ServiceServer tabletEventPromptAndGetResponseServer = nh.advertiseService("/textToSpeech/say_text", textToSpeechSayTextHandler);
    ROS_INFO_STREAM(tabletEventPromptAndGetResponseServer.getService()<<" Ready");

    ros::ServiceServer textToSpeechSayTextServer = nh.advertiseService("/tabletEvent/prompt_and_get_response", tabletEventPromptAndGetResponseHandler);
    ROS_INFO_STREAM(textToSpeechSayTextServer.getService()<<" Ready");

    ros::spin();
    return 0;
}