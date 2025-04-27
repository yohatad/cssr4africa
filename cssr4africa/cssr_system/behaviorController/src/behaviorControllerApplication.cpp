/* behaviorControllerApplication.cpp
* 
* <detailed functional description>
* The component starts the 'Robot Mission Interpreter' ROS Node. This node is the starting point for the robot to execute the mission selected.
* Though, this component exists as a standalone and can be started as such, without the set of ROS nodes that are part of the CSSR4Afica system architecture(see D3.1 System Architecture), it will not be able function.
...
* Libraries
* Standard libraries
- std::string, std::fstream
* ROS libraries
- ros/ros.h, ros/package.h, std_msgs
* BehaviorTree.Cpp libraries
- behaviortree_cpp/bt_factory.h, behaviortree_cpp/loggers/groot2_publisher.h
...
* Parameters
* *
* Command-line Parameters
* *
* None
 ...
* Configuration File Parameters
* Key | Value
* ----|------
* scenarioSpecification | <the mission scenario to be interpreted>
* verboseMode           | <true/false - enables/disables the display of diagnostic messages>
* asrEnabled            | <true/false> - enables/disables the Automatic Speech Recognition. If diabled, pepper's tablet will be primary input method
* testMode              | <true/false> - enables/disabled the test sequence. If enabled, it's the test sequence that will run
...
* Subscribed Topics and Message Types
**
- /faceDetection/data       faceDetectionData.msg
- /overtAttention/mode      overtAttentionMode.msg
- /speechEvent/text         std_msgs::String
...
* Published Topics and Message Types
**
* None
...
* Advertised Services
* 
* None
...
* Services Invoked
* *
* /animateBehaviour/setActivation
* /gestureExecution/perform_gesture
* /overtAttention/set_mode
* /robotLocalization/reset_pose
* /robotNavigation/set_goal
* /speechEvent/set_language
* /speechEvent/set_enabled
* /tabletEvent/prompt_and_get_response
* /textToSpeech/say_text                                    
...
* Input Data Files
*
* lab_tour.xml
...
* Output Data Files
* 
* None
...
* Configuration Files
**
* behaviorControllerConfiguration.ini
...
* Example Instantiation of the Module
* *
* rosrun cssr_system behaviorController
...
* *
* Author: Tsegazeab Taye Tefferi, Carnegie Mellon University Africa
* Email: ttefferi@andrew.cmu.edu
* Date: April 08, 2025
* Version: v1.0
* *
*/

#include "behaviorController/behaviorControllerInterface.h"

/* Configuration variables */
bool verboseMode;
bool asrEnabled;
std::string missionLanguage;
bool testMode;
/*************************/
std::string nodeName;
ros::NodeHandle* nh = nullptr;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "behaviorController");
    BT::Tree tree;
    ros::NodeHandle nodeHandle;
    nh = &nodeHandle;

    std::string scenario;
    nodeName = ros::this_node::getName();
    std::string softwareVersion = "1.0";
    std::string leftIndent = "\n\t\t";
    ROS_INFO_STREAM("[" << nodeName << "]: startup");
    ROS_INFO_STREAM("\n"
         "**************************************************************************************************\n"
         + leftIndent + nodeName + "\t" + softwareVersion
         + leftIndent + "Copyright (C) 2023 CSSR4Africa Consortium"
         + leftIndent + "This project is funded by the African Engineering and Technology Network (Afretec)"
         + leftIndent + "Inclusive Digital Transformation Research Grant Program.\n"
         + leftIndent + "Website: www.cssr4africa.org\n"
         + leftIndent + "This program comes with ABSOLUTELY NO WARRANTY."
         + "\n\n**************************************************************************************************\n\n"
    );

    /* Retrieve the values from the configuration file       */
    /* Display the error and exit, if the file is unreadable */
    try
    {
        scenario = getValueFromConfig("scenarioSpecification");
        verboseMode = (getValueFromConfig("verboseMode") == "true");
        asrEnabled = (getValueFromConfig("asrEnabled") == "true");
        testMode = (getValueFromConfig("testMode") == "true");
    }
    catch (const std::exception& e) {
        ROS_ERROR_STREAM("Fatal Error: "<<e.what());
        ros::shutdown();
        return 0;
    }

    /* Get the currently set language from the knowledge base*/
    missionLanguage = getMissionLanguage();


    ROS_INFO_STREAM("Scenario Specification: "<< scenario);
    ROS_INFO_STREAM("Mission Language: "<<missionLanguage);
    ROS_INFO_STREAM("Mode: "<<(testMode ? "Test" : "Normal"));

    /* List of services to check for life*/
    std::vector<std::string> services = {
        "/animateBehaviour/setActivation",
        "/gestureExecution/perform_gesture",
        "/overtAttention/set_mode",
        "/robotLocalization/reset_pose",
        "/robotNavigation/set_goal",
        "/speechEvent/set_language",
        "/speechEvent/set_enabled",
        "/tabletEvent/prompt_and_get_response",
        "/textToSpeech/say_text"
    };
    
    /* List of topics to check for life*/
    std::vector<std::string> topics = {
        "/faceDetection/data",
        "/overtAttention/mode",
        "/speechEvent/text",
    };

    /* If any of the services from above isn't alive, exit program */
    ROS_INFO_STREAM("Checking Services...");
    if (checkServices(services)) {
        ROS_INFO_STREAM("All services available");
    } else {
        ros::shutdown();
        return 0;
    }

    /* If any of the topics from above isn't alive, exit program */
    ROS_INFO_STREAM("Checking Topics...");
    if (checkTopics(topics)) {
        ROS_INFO_STREAM("All topics available");
    } else {
        ros::shutdown();
        return 0;
    }

    /* Use the mission specification file to create the tree and initiate the mission*/
    tree = initializeTree(scenario);
    BT::Groot2Publisher Groot2Publisher(tree);
    ros::Rate rate(1);
    ROS_INFO_STREAM("Starting Misssion Execution ....");
    while (ros::ok()) {
        tree.tickWhileRunning();
        ros::spinOnce();
        rate.sleep();
        ROS_INFO_STREAM("[" << nodeName << "]: running");
    }

    return 0;
}
