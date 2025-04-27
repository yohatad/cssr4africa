/* behaviorControllerImplementation.cpp   Source code for the implementation of the robot mission node classes and other utility functions
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

#include "behaviorController/behaviorControllerInterface.h"

/* Definitions for printMsg function */
#define INFO_MSG 0
#define WARNING_MSG 1
#define ERROR_MSG 2

/*
Logs the string (args) to the terminal based on the (type).
Wrapper around the default ROS logging functions
*/
void printMsg(int type, std::string args);

/* Fetches the utility phrase from the culture knowledge base using the id and language */
std::string getUtilityPhrase(std::string phraseId, std::string language);

/* 
    Stores the result of a nodes execution in the paramteter server.
    To be used by the test node.
*/
static void storeResult(std::string key, int value);

/***** Global Variables ****/
// Environment Knowledge Base
Environment::EnvironmentKnowledgeBase environmentKnowledgeBase;
Environment::TourSpecificationType tour;
Environment::KeyValueType enviornmentKeyValue;

// Culture Knowledge Base
Culture::KeyValueType cultureKeyValue;
Culture::CultureKnowledgeBase culturalKnowledgeBase;
Culture::Keyword key;
/********************************** */

/****** Mission(Action/Condition) Nodes */
/*
    Handler for the 'HandleFallBack' Action Node
*/
class HandleFallBack : public BT::SyncActionNode
{
   public:
    HandleFallBack(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        std::string treeNodeName = "HandleFallback";
        printMsg(INFO_MSG, "(" + treeNodeName + " Action Node)");

        if (testMode) {
            storeResult(treeNodeName, 1);
        }
        return BT::NodeStatus::SUCCESS;
    }

   private:
    ros::ServiceClient client;
};

/*
    Handler for the 'SetAnimateBehavior' Action Node
    Enables & Disables animate behavior
*/
class SetAnimateBehavior : public BT::SyncActionNode
{
   public:
    SetAnimateBehavior(const std::string &name, const BT::NodeConfiguration &config) : BT::SyncActionNode(name, config)
    {
        /* Define a service client */
        client = nh->serviceClient<cssr_system::animateBehaviorSetActivation>("/animateBehaviour/setActivation");
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        cssr_system::animateBehaviorSetActivation srv;
        std::string state = name();  // retrieve the state from the mission specification
        std::string treeNodeName = "SetAnimateBehavior";

        printMsg(INFO_MSG, treeNodeName + "Action Node");
        printMsg(INFO_MSG, "State: " + state);

        srv.request.state = state;
        /* Make a service call to animateBehavior node and act according to the response*/
        if (client.call(srv)) {
            if (srv.response.success != "1") {
                printMsg(WARNING_MSG, "Called service returned failure");
                if (testMode) {
                    storeResult(treeNodeName, 0);
                }
                return BT::NodeStatus::FAILURE;
            }
        } else {
            printMsg(ERROR_MSG, "Failed to call service");
            if (testMode) {
                storeResult(treeNodeName, 0);
            }
            return BT::NodeStatus::FAILURE;
        }
        if (testMode) {
            storeResult(treeNodeName, 1);
        }
        return BT::NodeStatus::SUCCESS;
    }

   private:
    ros::ServiceClient client;
};

/*
    Handler for the 'SetOvertAttentionMode' Action Node
    Set different values to the attention mode of the overtAttention ROS node
*/
class SetOvertAttentionMode : public BT::SyncActionNode
{
   public:
    SetOvertAttentionMode(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config)
    {
        /* Define a service client */
        client = nh->serviceClient<cssr_system::overtAttentionSetMode>("/overtAttention/set_mode");
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        cssr_system::overtAttentionSetMode srv;
        std::string state = name();  // retrieve the state from the mission specification
        std::string treeNodeName = "SetOvertAttentionMode";

        printMsg(INFO_MSG, treeNodeName + "Action Node");
        printMsg(INFO_MSG, "State: " + state);

        srv.request.state = state;

        // if mode is 'location', retrieve the target from the blackboard
        if (state == "location") {
            Environment::RobotLocationType location;
            Environment::GestureTargetType gestureTarget;
            if (!config().blackboard->rootBlackboard()->get("exhibitGestureTarget", gestureTarget)) {
                printMsg(ERROR_MSG, "Unable to retrieve from blackboard");
                if (testMode) {
                    storeResult(treeNodeName, 0);
                }
                return BT::NodeStatus::FAILURE;
            }

            // Set the retrieved values in order to make a service call
            srv.request.location_x = gestureTarget.x;
            srv.request.location_y = gestureTarget.y;
            srv.request.location_z = gestureTarget.z;
        }

        /* Make a service call to the node and act according to the response*/
        if (client.call(srv)) {
            if (!srv.response.mode_set_success) {
                printMsg(WARNING_MSG, "Called service returned failure");
                if (testMode) {
                    storeResult(treeNodeName, 0);
                }
                return BT::NodeStatus::FAILURE;
            }
        } else {
            printMsg(ERROR_MSG, "Failed to call service");
            if (testMode) {
                storeResult(treeNodeName, 0);
            }
            return BT::NodeStatus::FAILURE;
        }
        if (testMode) {
            storeResult(treeNodeName, 1);
        }
        return BT::NodeStatus::SUCCESS;
    }

   private:
    ros::ServiceClient client;
};

/*
    Handler for the 'SetSpeechEvent' Action Node
    Enables & Disables transcription on the speechEvent ROS node
*/
class SetSpeechEvent : public BT::SyncActionNode
{
   public:
    SetSpeechEvent(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config)
    {
        /* Define a service client */
        client = nh->serviceClient<cssr_system::speechEventSetEnabled>("/speechEvent/set_enabled");
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        cssr_system::speechEventSetEnabled srv;
        std::string status = name();  // retrieve the status from the mission specification
        std::string treeNodeName = "SetSpeechEvent";

        printMsg(INFO_MSG, treeNodeName + "Action Node");
        printMsg(INFO_MSG, "Status: " + status);

        srv.request.status = status;
        /* Make a service call to the node and act according to the response*/
        if (client.call(srv)) {
            if (!srv.response.response) {
                printMsg(WARNING_MSG, "Called service returned failure");
                if (testMode) {
                    storeResult(treeNodeName, 0);
                }
                return BT::NodeStatus::FAILURE;
            }
        } else {
            printMsg(ERROR_MSG, "Failed to call service");
            if (testMode) {
                storeResult(treeNodeName, 0);
            }
            return BT::NodeStatus::FAILURE;
        }
        if (testMode) {
            storeResult(treeNodeName, 1);
        }
        return BT::NodeStatus::SUCCESS;
    }

   private:
    ros::ServiceClient client;
};
/*
    Handler for the 'IsVisitorDiscovered' Condition Node
    Checks for the presence of a visitor via the faceDetection ROS node
*/
class IsVisitorDiscovered : public BT::ConditionNode
{
   public:
    IsVisitorDiscovered(const std::string &name, const BT::NodeConfiguration &config)
        : BT::ConditionNode(name, config), visitorDiscovered(false)
    {
        /* Define a subscriber to the topic */
        subscriber = nh->subscribe("/faceDetection/data", 10, &IsVisitorDiscovered::callback, this);
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        std::string treeNodeName = "IsVisitorDiscovered";

        printMsg(INFO_MSG, treeNodeName + "Condition Node");

        ros::Rate rate(10);
        /* Wait until the topic returns data, indicating arrival of a potential visitor */
        while (ros::ok()) {
            ros::spinOnce();

            if (visitorDiscovered) {
                printMsg(INFO_MSG, "Visitor discovered");
                if (testMode) {
                    storeResult(treeNodeName, 1);
                }
                return BT::NodeStatus::SUCCESS;
            }

            rate.sleep();
        }

        if (testMode) {
            storeResult(treeNodeName, 0);
        }
        return BT::NodeStatus::FAILURE;
    }

   private:
    void callback(const cssr_system::faceDetectionData::ConstPtr &msg)
    {
        visitorDiscovered = false;
        /* if the face_label_id array contains values, it indicates presence of a potential visitor*/
        if (msg->face_label_id.size() > 0) {
            visitorDiscovered = true;
        }
    }

    bool visitorDiscovered;
    ros::Subscriber subscriber;
};

/*
    Handler for the 'IsMutualGazeDiscovered' Condition Node
    Checks for the detection of mutual gaze via overtAttention ROS node
*/
class IsMutualGazeDiscovered : public BT::ConditionNode
{
   public:
    IsMutualGazeDiscovered(const std::string &name, const BT::NodeConfiguration &config)
        : BT::ConditionNode(name, config), seekingStatus("RUNNING")
    {
        /* Define a subscriber to the topic */
        subscriber = nh->subscribe("/overtAttention/mode", 10, &IsMutualGazeDiscovered::callback, this);
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        std::string treeNodeName = "IsMutualGazeDiscovered";

        printMsg(INFO_MSG, treeNodeName + "Condition Node");

        ros::Rate rate(10);
        auto startTime = ros::Time::now();

        /* Keep checking for the detection of mutual gaze */
        while (ros::ok()) {
            ros::spinOnce();
            if (seekingStatus == "SUCCESS") {
                printMsg(INFO_MSG, "Mutual gaze detected");
                if (testMode) {
                    storeResult(treeNodeName, 1);
                }
                return BT::NodeStatus::SUCCESS;
            } else if (seekingStatus == "FAILURE") {
                printMsg(INFO_MSG, "Mutual gaze detection failed");
                if (testMode) {
                    storeResult(treeNodeName, 0);
                }
                return BT::NodeStatus::FAILURE;
            }

            rate.sleep();
        }

        if (testMode) {
            storeResult(treeNodeName, 0);
        }
        return BT::NodeStatus::FAILURE;
    }

   private:
    void callback(const cssr_system::overtAttentionMode::ConstPtr &msg)
    {
        /*
            Values 2 & 3, indicating success & failure respectively are how
            the overtAttention node relays 'seeking' mode status
        */

        if (msg->state == "seeking" && msg->value == 2) {
            seekingStatus = "SUCCESS";
        } else if (msg->state == "seeking" && msg->value == 3) {
            seekingStatus = "FAILURE";
        } else {
            seekingStatus = "RUNNING";
        }
    }

    std::string seekingStatus;
    ros::Subscriber subscriber;
};

/*
    Handler for the 'DescribeExhibitSpeech' Action Node
    Sends exhibit description to be uttered by the textToSpeech ROS node
    The exhibit selected changes as the mission execution loops over the list of exhibits
*/
class DescribeExhibitSpeech : public BT::SyncActionNode
{
   public:
    DescribeExhibitSpeech(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config)
    {
        /* Define a service client */
        client = nh->serviceClient<cssr_system::textToSpeechSayText>("/textToSpeech/say_text");
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        cssr_system::textToSpeechSayText srv;
        srv.request.language = missionLanguage;
        std::string nodeInstance = name();  // retrieve nodeInstance value from the specification
        std::string message;
        std::string treeNodeName = "DescribeExhibit";

        printMsg(INFO_MSG, treeNodeName + "Action Node");

        /* '1' indicates Pre-Gesture Message */
        if (nodeInstance == "1") {
            if (!config().blackboard->get("exhibitPreGestureMessage", message)) {
                printMsg(ERROR_MSG, "Unable to retrieve from blackboard");
                if (testMode) {
                    storeResult(treeNodeName, 0);
                }
                return BT::NodeStatus::FAILURE;
            }
        } else if (nodeInstance == "2") {  // '2' indicates Post-Gesture message
            if (!config().blackboard->get("exhibitPostGestureMessage", message)) {
                printMsg(ERROR_MSG, "Unable to retrieve from blackboard");
                if (testMode) {
                    storeResult(treeNodeName, 0);
                }
                return BT::NodeStatus::FAILURE;
            }
        } else {
            printMsg(WARNING_MSG, "Invalid Node Instance");
            if (testMode) {
                storeResult(treeNodeName, 0);
            }
            return BT::NodeStatus::FAILURE;
        }

        srv.request.message = message;
        /* Make a service call to the node and act according to the response*/
        if (client.call(srv)) {
            if (!srv.response.success) {
                printMsg(WARNING_MSG, "Called service returned failure");
                if (testMode) {
                    storeResult(treeNodeName, 0);
                }
                return BT::NodeStatus::FAILURE;
            }
        } else {
            printMsg(ERROR_MSG, "Failed to call service");
            if (testMode) {
                storeResult(treeNodeName, 0);
            }
            return BT::NodeStatus::FAILURE;
        }
        ros::Duration(3.0).sleep();  // Provides a bit of buffer after the speech
        if (testMode) {
            storeResult(treeNodeName, 1);
        }
        return BT::NodeStatus::SUCCESS;
    }

   private:
    ros::ServiceClient client;
};

/*
    Handler for the 'SayText' Action Node
    Sends text to be uttered by the textToSpeech ROS node
*/
class SayText : public BT::SyncActionNode
{
   public:
    SayText(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config)
    {
        /* Define a service client */
        client = nh->serviceClient<cssr_system::textToSpeechSayText>("/textToSpeech/say_text");
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        cssr_system::textToSpeechSayText srv;
        srv.request.language = missionLanguage;
        std::string utilityPhraseId = name();  // retrieve id from the mission specification
        std::string treeNodeName = "SayText";

        printMsg(INFO_MSG, treeNodeName + "Action Node");

        srv.request.message = getUtilityPhrase(utilityPhraseId, missionLanguage);
        /* Make a service call to the node and act according to the response*/
        if (client.call(srv)) {
            if (!srv.response.success) {
                printMsg(WARNING_MSG, "Called service returned failure");
                if (testMode) {
                    storeResult(treeNodeName, 0);
                }
                return BT::NodeStatus::FAILURE;
            }
        } else {
            printMsg(ERROR_MSG, "Failed to call service");
            if (testMode) {
                storeResult(treeNodeName, 0);
            }
            return BT::NodeStatus::FAILURE;
        }
        if (testMode) {
            storeResult(treeNodeName, 1);
        }
        return BT::NodeStatus::SUCCESS;
    }

   private:
    ros::ServiceClient client;
};

/*
    Handler for the 'PerformDeicticGesture' Action Node
    Performs a deictic gesture via the gestureExecution ROS node
*/
class PerformDeicticGesture : public BT::SyncActionNode
{
   public:
    PerformDeicticGesture(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config)
    {
        /* Define a service client */
        client = nh->serviceClient<cssr_system::gestureExecutionPerformGesture>("/gestureExecution/perform_gesture");
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        cssr_system::gestureExecutionPerformGesture srv;
        Environment::GestureTargetType gestureTarget;
        std::string treeNodeName = "PerformDeicticGesture";

        printMsg(INFO_MSG, treeNodeName + "Action Node");

        /* Retrieve gesture values from the blackboard*/
        if (!config().blackboard->rootBlackboard()->get("exhibitGestureTarget", gestureTarget)) {
            printMsg(ERROR_MSG, "Unable to retrieve from blackboard");
            if (testMode) {
                storeResult(treeNodeName, 0);
            }
            return BT::NodeStatus::FAILURE;
        }

        srv.request.gesture_type = "deictic";
        srv.request.gesture_id = 01;
        srv.request.gesture_duration = 3000;
        srv.request.bow_nod_angle = 0;
        srv.request.location_x = gestureTarget.x;
        srv.request.location_y = gestureTarget.y;
        srv.request.location_z = gestureTarget.z;

        /* Make a service call to the node and act according to the response*/
        if (client.call(srv)) {
            if (!srv.response.gesture_success) {
                printMsg(WARNING_MSG, "Called service returned failure");
                if (testMode) {
                    storeResult(treeNodeName, 0);
                }
                return BT::NodeStatus::FAILURE;
            }
        } else {
            printMsg(ERROR_MSG, "Failed to call service");
            if (testMode) {
                storeResult(treeNodeName, 0);
            }
            return BT::NodeStatus::FAILURE;
        }
        if (testMode) {
            storeResult(treeNodeName, 1);
        }
        return BT::NodeStatus::SUCCESS;
    }

   private:
    ros::ServiceClient client;
};

/*
    Handler for the 'PerformIconicGesture' Action Node
    Performs an iconic gesture via the gestureExecution ROS node
*/
class PerformIconicGesture : public BT::SyncActionNode
{
   public:
    PerformIconicGesture(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config)
    {
        /* Define a service client */
        client = nh->serviceClient<cssr_system::gestureExecutionPerformGesture>("/gestureExecution/perform_gesture");
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        cssr_system::gestureExecutionPerformGesture srv;
        std::string iconicGestureType = name();  // retrieve the gesture type from the mission specification
        std::string treeNodeName = "PerformIconicGesture";

        printMsg(INFO_MSG, treeNodeName + "Action Node");
        printMsg(INFO_MSG, "Gesture: " + iconicGestureType);

        srv.request.gesture_type = "iconic";
        srv.request.gesture_duration = 3000;
        srv.request.bow_nod_angle = 0;
        srv.request.location_x = 0;
        srv.request.location_y = 0;
        srv.request.location_z = 0;

        /*
            '01' and '03' represent the 'welcome' and 'goodbye'
             iconic gestures in the gestureExecution node
        */
        if (iconicGestureType == "welcome") {
            srv.request.gesture_id = 01;
        } else if (iconicGestureType == "goodbye") {
            srv.request.gesture_id = 03;
        } else {
            printMsg(ERROR_MSG, "Undefined Iconic Gesture Type");
            if (testMode) {
                storeResult(treeNodeName, 0);
            }
            return BT::NodeStatus::FAILURE;
        }

        /* Make a service call to the node and act according to the response*/
        if (client.call(srv)) {
            if (!srv.response.gesture_success) {
                printMsg(WARNING_MSG, "Called service returned failure");
                if (testMode) {
                    storeResult(treeNodeName, 0);
                }
                return BT::NodeStatus::FAILURE;
            }
        } else {
            printMsg(ERROR_MSG, "Failed to call service");
            if (testMode) {
                storeResult(treeNodeName, 0);
            }
            return BT::NodeStatus::FAILURE;
        }
        if (testMode) {
            storeResult(treeNodeName, 1);
        }
        return BT::NodeStatus::SUCCESS;
    }

   private:
    ros::ServiceClient client;
};

/*
    Handler for the 'PressYesNoDialogue' Action Node
    Initiates a dialogue on the robot's table via tabletEvent ROS node
*/
class PressYesNoDialogue : public BT::SyncActionNode
{
   public:
    PressYesNoDialogue(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config)
    {
        /* Define a service client */
        client = nh->serviceClient<cssr_system::tabletEventPromptAndGetResponse>("/tabletEvent/prompt_and_get_response");
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        std::string treeNodeName = "PressYesNoDialogue(Tablet)";
        cssr_system::tabletEventPromptAndGetResponse srv;
        srv.request.message = "'Yes'|'No'";

        printMsg(INFO_MSG, treeNodeName + " Action Node");

        /* Make a service call to the node and act according to the response*/
        if (client.call(srv)) {
            if (!srv.response.success) {
                printMsg(WARNING_MSG, "Called service returned failure");
                if (testMode) {
                    storeResult(treeNodeName, 0);
                }
                return BT::NodeStatus::FAILURE;
            }
        } else {
            printMsg(ERROR_MSG, "Failed to call service");
            if (testMode) {
                storeResult(treeNodeName, 0);
            }
            return BT::NodeStatus::FAILURE;
        }
        if (testMode) {
            storeResult(treeNodeName, 1);
        }
        return BT::NodeStatus::SUCCESS;
    }

   private:
    ros::ServiceClient client;
};

/*
    Handler for the 'RetrieveListOfExhibits' Action Node
    Retrieve the exhibits for the tour from the environment knowledge base
*/
class RetrieveListOfExhibits : public BT::SyncActionNode
{
   public:
    RetrieveListOfExhibits(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        std::string treeNodeName = "RetrieveListOfExhibits";

        printMsg(INFO_MSG, treeNodeName + "Action Node");

        /* Get the tour infomration from the enviornment knowledge base*/
        environmentKnowledgeBase.getTour(&tour);

        /* if tour.numberOfLocations is 0, there are no exhibits and the mission cannot continue*/
        if (tour.numberOfLocations == 0) {
            printMsg(ERROR_MSG, "Number of Exhibits is 0");
            if (testMode) {
                storeResult(treeNodeName, 0);
            }
            return BT::NodeStatus::FAILURE;
        }

        config().blackboard->set("visits", 0);
        if (testMode) {
            storeResult(treeNodeName, 1);
        }
        return BT::NodeStatus::SUCCESS;
    }
};

/*
    Handler for the 'SelectExhibit' Action Node
    Selects the next exhibit from the list
*/
class SelectExhibit : public BT::SyncActionNode
{
   public:
    SelectExhibit(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        std::string treeNodeName = "SelectExhibit";

        printMsg(INFO_MSG, treeNodeName + "Action Node");

        int visits = 0;
        std::string preGestureMessage;
        std::string postGestureMessage;

        if (!config().blackboard->get("visits", visits)) {
            printMsg(WARNING_MSG, "Exhibit list empty");
            if (testMode) {
                storeResult(treeNodeName, 0);
            }
            return BT::NodeStatus::FAILURE;
        }
        // Select the next exhibit to visit
        environmentKnowledgeBase.getValue(tour.locationIdNumber[visits], &enviornmentKeyValue);

        // Retrieve the exhibit description based on the current set language
        if (missionLanguage == "English") {
            preGestureMessage = enviornmentKeyValue.preGestureMessageEnglish;
            postGestureMessage = enviornmentKeyValue.postGestureMessageEnglish;
        } else if (missionLanguage == "Kinyarwanda") {
            preGestureMessage = enviornmentKeyValue.preGestureMessageKinyarwanda;
            postGestureMessage = enviornmentKeyValue.postGestureMessageKinyarwanda;
        } else if (missionLanguage == "IsiZulu") {
            preGestureMessage = enviornmentKeyValue.preGestureMessageIsiZulu;
            postGestureMessage = enviornmentKeyValue.postGestureMessageIsiZulu;
        } else {
            printMsg(ERROR_MSG, "Unknown language set");
            if (testMode) {
                storeResult(treeNodeName, 0);
            }
            return BT::NodeStatus::FAILURE;
        }

        // Store the values in the blackboard to be retrieved by other mission nodes
        config().blackboard->set("exhibitPreGestureMessage", preGestureMessage);
        config().blackboard->set("exhibitPostGestureMessage", postGestureMessage);
        config().blackboard->rootBlackboard()->set("exhibitLocation", enviornmentKeyValue.robotLocation);
        config().blackboard->rootBlackboard()->set("exhibitGestureTarget", enviornmentKeyValue.gestureTarget);

        printMsg(INFO_MSG, std::string("Visiting: ") + enviornmentKeyValue.robotLocationDescription);

        config().blackboard->set("visits", ++visits);  // indicate that the current exhbit is already 'visitited', when checked later
        if (testMode) {
            storeResult(treeNodeName, 1);
        }
        return BT::NodeStatus::SUCCESS;
    }
};

/*
    Handler for the 'IsListWithExhibit' Condition Node
    Checks if there are any more exhibits to visit
*/
class IsListWithExhibit : public BT::ConditionNode
{
   public:
    IsListWithExhibit(const std::string &name, const BT::NodeConfiguration &config)
        : BT::ConditionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        std::string treeNodeName = "IsListWithExhibit";

        printMsg(INFO_MSG, treeNodeName + "Condition Node");

        int visits = 0;
        if (!config().blackboard->get("visits", visits)) {
            printMsg(ERROR_MSG, "Unable to retrieve from blackboard");
            if (testMode) {
                storeResult(treeNodeName, 0);
            }
            return BT::NodeStatus::FAILURE;
        }

        // if the number of 'visits' value exceeds the number of locations, then all exhibits have been visited
        if (visits < tour.numberOfLocations) {
            if (testMode) {
                storeResult(treeNodeName, 1);
            }
            return BT::NodeStatus::SUCCESS;
        } else {
            printMsg(INFO_MSG, "ALL LANDMARKS VISITED");
            return BT::NodeStatus::FAILURE;
        }
    }
};

/*
    Handler for the 'Navigate' Action Node
    Performs navigation via the robotNaviggation ROS node
*/
class Navigate : public BT::SyncActionNode
{
   public:
    Navigate(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config)
    {
        /* Define a service client */
        client = nh->serviceClient<cssr_system::robotNavigationSetGoal>("/robotNavigation/set_goal");
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        cssr_system::robotNavigationSetGoal srv;
        Environment::RobotLocationType location;
        std::string treeNodeName = "Navigate";

        printMsg(INFO_MSG, treeNodeName + "Action Node");

        // Retrieve location values fromthe blackboard
        if (!config().blackboard->rootBlackboard()->get("exhibitLocation", location)) {
            printMsg(ERROR_MSG, "Unable to retrieve from blackboard");
            if (testMode) {
                storeResult(treeNodeName, 0);
            }
            return BT::NodeStatus::FAILURE;
        }
        srv.request.goal_x = location.x;
        srv.request.goal_y = location.y;
        srv.request.goal_theta = location.theta;

        /* Make a service call to the node and act according to the response*/
        if (client.call(srv)) {
            if (!srv.response.navigation_goal_success) {
                printMsg(WARNING_MSG, "Called service returned failure");
                if (testMode) {
                    storeResult(treeNodeName, 0);
                }
                return BT::NodeStatus::FAILURE;
            }
        } else {
            printMsg(ERROR_MSG, "Failed to call service");
            if (testMode) {
                storeResult(treeNodeName, 0);
            }
            return BT::NodeStatus::FAILURE;
        }
        if (testMode) {
            storeResult(treeNodeName, 1);
        }
        return BT::NodeStatus::SUCCESS;
    }

   private:
    ros::ServiceClient client;
};

/*
    Handler for the 'ResetRobotPose' Action Node
    Resets the current pose via the robotLocalization ROS node
*/
class ResetRobotPose : public BT::SyncActionNode
{
   public:
    ResetRobotPose(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config)
    {
        /* Define a service client */
        client = nh->serviceClient<cssr_system::robotLocalizationResetPose>("/robotLocalization/reset_pose");
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        cssr_system::robotLocalizationResetPose srv;
        std::string treeNodeName = "ResetRobotPose";

        printMsg(INFO_MSG, treeNodeName + "Action Node");

        /* Make a service call to the node and act according to the response*/
        if (client.call(srv)) {
            if (!srv.response.success) {
                printMsg(WARNING_MSG, "Called service returned failure");
                if (testMode) {
                    storeResult(treeNodeName, 0);
                }
                return BT::NodeStatus::FAILURE;
            }
        } else {
            printMsg(ERROR_MSG, "Failed to call service");
            if (testMode) {
                storeResult(treeNodeName, 0);
            }
            return BT::NodeStatus::FAILURE;
        }
        if (testMode) {
            storeResult(treeNodeName, 1);
        }
        return BT::NodeStatus::SUCCESS;
    }

   private:
    ros::ServiceClient client;
};
/*
    Handler for the 'GetVisitorResponse' Action Node
    Retrieves visitor responses via the speechEvent ROS node
*/
class GetVisitorResponse : public BT::SyncActionNode
{
   public:
    GetVisitorResponse(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config), isResponseReceived(false)
    {
        subscriber = nh->subscribe("/speechEvent/text", 10, &GetVisitorResponse::callback, this);
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        std::string treeNodeName = "GetVisitorResponse";

        printMsg(INFO_MSG, treeNodeName + "Action Node");
        ros::Rate rate(1);
        ros::Time startTime = ros::Time::now();
        auto blackboard = config().blackboard->rootBlackboard();
        while (ros::ok()) {
            ros::spinOnce();
            if (isResponseReceived) {
                /** Set of words to check for an affirmative response from the visitor */
                std::map<std::string, std::vector<std::string>> affirmativeWords;
                affirmativeWords["English"] = {"yes", "great", "absolutely", "go", "happy", "good", "love"};
                affirmativeWords["Kinyarwanda"] = {"yego", "ntakibazo", "nibyo"};

                /*If any of the 'affirmative' words are detected, set 'visitorResponse' as yes in the blackboard */
                for (const string &affirmativeWord : affirmativeWords[missionLanguage]) {
                    if (visitorResponse.find(affirmativeWord) != string::npos) {
                        blackboard->set("visitorResponse", "yes");
                        printMsg(INFO_MSG, "Visitor Response: " + visitorResponse);
                        if (testMode) {
                            storeResult(treeNodeName, 1);
                        }
                        return BT::NodeStatus::SUCCESS;
                    }
                }
            }

            /* If there is no response within 5 seconds, set 'visitorResponse' as no in the blackboard*/
            if ((ros::Time::now() - startTime).toSec() > 5) {
                printMsg(WARNING_MSG, "No affirmative response received");
                blackboard->set("visitorResponse", "no");
                if (testMode) {
                    storeResult(treeNodeName, 0);
                }
                return BT::NodeStatus::FAILURE;
            }

            rate.sleep();
        }
        if (testMode) {
            storeResult(treeNodeName, 0);
        }
        return BT::NodeStatus::FAILURE;
    }

   private:
    void callback(const std_msgs::String::ConstPtr &msg)
    {
        visitorResponse = msg->data;
        isResponseReceived = true;
    }

    std::string visitorResponse;
    bool isResponseReceived;
    ros::Subscriber subscriber;
};

/*
    Handler for the 'IsVisitorResponseYes' Condition Node
    Checks if the response from the visitor is affirmative
    Validates the final check to start the tour or end interaction
*/
class IsVisitorResponseYes : public BT::ConditionNode
{
   public:
    IsVisitorResponseYes(const std::string &name, const BT::NodeConfiguration &config)
        : BT::ConditionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        std::string treeNodeName = "IsVisitorResponseYes";

        printMsg(INFO_MSG, treeNodeName + "Condition Node");

        auto blackboard = config().blackboard->rootBlackboard();
        std::string visitorResponse;
        if (blackboard->get("visitorResponse", visitorResponse)) {
            if (visitorResponse == "yes") {
                if (testMode) {
                    storeResult(treeNodeName, 1);
                }
                return BT::NodeStatus::SUCCESS;
            }
        }

        if (testMode) {
            storeResult(treeNodeName, 0);
        }
        return BT::NodeStatus::FAILURE;
    }
};

/*
    Handler for the 'StartOfTree' Action Node
    Used for logging debugging information if verboseMose is enabled
    Used for initializing certain mission paramters
        - the language used by speechEvent ROS node for transcription
*/
class StartOfTree : public BT::SyncActionNode
{
   public:
    StartOfTree(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config), testStarted(false)
    {
        /* Define a service client */
        client = nh->serviceClient<cssr_system::speechEventSetLanguage>("/speechEvent/set_language");
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        std::string treeNodeName = "StartOfTree";
        printMsg(INFO_MSG,
                 "\n============================\n"
                 "\n\tSTART OF TREE\n"
                 "\n============================\n");


        if(testMode){
            if(testStarted){
                storeResult("TestEnded",1);
                printMsg(INFO_MSG,"Test Sequence Completed. Waiting for test node to finish.");
                ros::Rate rate(100);
                while(ros::ok()){
                    rate.sleep();
                }                
            }else{
                testStarted = true;
                printMsg(INFO_MSG,"Test Sequence started.");
                storeResult("TestStarted",1);
            }
        }

        // As this node is executed at the start of the mission, this is where
        // variables with mission execution lifetime are set

        // Setting the language for speechEvent if ASR is enabled
        if (asrEnabled) {
            cssr_system::speechEventSetLanguage srv;
            srv.request.language = missionLanguage;

            if (client.call(srv)) {
                if (!srv.response.response) {
                    printMsg(WARNING_MSG, "Called service returned failure");
                    if (testMode) {
                        storeResult(treeNodeName, 0);
                    }
                    return BT::NodeStatus::FAILURE;
                }
            } else {
                printMsg(ERROR_MSG, "Failed to call service");
                if (testMode) {
                    storeResult(treeNodeName, 0);
                }
                return BT::NodeStatus::FAILURE;
            }
        }
        if (testMode) {
            storeResult(treeNodeName, 1);
        }
        return BT::NodeStatus::SUCCESS;
    }

   private:
    ros::ServiceClient client;
    bool testStarted;
};

/*
    Handler for the 'IsASREnabled' Condition Node
    Checks if ASR is set to enabled in the configuration file
*/
class IsASREnabled : public BT::ConditionNode
{
   public:
    IsASREnabled(const std::string &name, const BT::NodeConfiguration &config)
        : BT::ConditionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        std::string treeNodeName = "IsASREnabled";

        printMsg(INFO_MSG, treeNodeName + "Condition Node");
        if (asrEnabled) {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }
};

/*****************************************************/

BT::Tree initializeTree(std::string scenario)
{
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<StartOfTree>("StartOfTree");
    factory.registerNodeType<DescribeExhibitSpeech>("DescribeExhibitSpeech");
    factory.registerNodeType<GetVisitorResponse>("GetVisitorResponse");
    factory.registerNodeType<HandleFallBack>("HandleFallBack");
    factory.registerNodeType<IsASREnabled>("IsASREnabled");
    factory.registerNodeType<IsListWithExhibit>("IsListWithExhibit");
    factory.registerNodeType<IsMutualGazeDiscovered>("IsMutualGazeDiscovered");
    factory.registerNodeType<IsVisitorDiscovered>("IsVisitorDiscovered");
    factory.registerNodeType<IsVisitorResponseYes>("IsVisitorResponseYes");
    factory.registerNodeType<Navigate>("Navigate");
    factory.registerNodeType<PerformDeicticGesture>("PerformDeicticGesture");
    factory.registerNodeType<PerformIconicGesture>("PerformIconicGesture");
    factory.registerNodeType<PressYesNoDialogue>("PressYesNoDialogue");
    factory.registerNodeType<ResetRobotPose>("ResetRobotPose");
    factory.registerNodeType<RetrieveListOfExhibits>("RetrieveListOfExhibits");
    factory.registerNodeType<SayText>("SayText");
    factory.registerNodeType<SelectExhibit>("SelectExhibit");
    factory.registerNodeType<SetAnimateBehavior>("SetAnimateBehavior");
    factory.registerNodeType<SetOvertAttentionMode>("SetOvertAttentionMode");
    factory.registerNodeType<SetSpeechEvent>("SetSpeechEvent");

    return factory.createTreeFromFile(ros::package::getPath(ROS_PACKAGE_NAME) + "/behaviorController/data/" + scenario + ".xml");
}

/***** Utility Functions ******/

/*
Logs the string (args) to the terminal based on the (type).
Wrapper around the default ROS logging functions
*/
void printMsg(int type, std::string args)
{
    if (!verboseMode) {
        return;
    }

    std::string msg = "[" + nodeName + "]: " + args;
    switch (type) {
        case INFO_MSG:
            ROS_INFO_STREAM(msg);
            break;
        case WARNING_MSG:
            ROS_WARN_STREAM(msg);
            break;
        case ERROR_MSG:
            ROS_ERROR_STREAM(msg);
            break;
        default:
            ROS_ERROR_STREAM("UNDEFINED MSG TYPE");
    }
}

/* Returns the current language from the knowledge base*/
std::string getMissionLanguage()
{
    strcpy(key, "phraseLanguage");
    culturalKnowledgeBase.getValue(key, &cultureKeyValue);
    return cultureKeyValue.alphanumericValue;
}

/* Fetches the utility phrase from the culture knowledge base using the id and language */
std::string getUtilityPhrase(std::string phraseId, std::string language)
{
    std::string phraseKey = "utilityPhrase" + language + phraseId;
    strcpy(key, phraseKey.c_str());
    culturalKnowledgeBase.getValue(key, &cultureKeyValue);
    return cultureKeyValue.alphanumericValue;
}

/* Returns true if ch isn't an empty space character*/
static bool isNotSpace(unsigned char ch) {
    return !std::isspace(ch);
}

/* Trims whitespaces inplace */
static inline void trim(std::string &s) {
    // Trim leading spaces
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), isNotSpace));
    
    // Trim trailing spaces
    s.erase(std::find_if(s.rbegin(), s.rend(), isNotSpace).base(), s.end());
}

/* Returns the value of a key from the configuration file. */
std::string getValueFromConfig(const std::string &key)
{
    std::string value;
    std::ifstream configFile(ros::package::getPath(ROS_PACKAGE_NAME) + "/behaviorController/config/behaviorControllerConfiguration.ini");
    if (!configFile.is_open()) {
        throw std::runtime_error("Failed to open configuration file.");
    }

    std::string configLineRead;

    while (std::getline(configFile, configLineRead)) {
        std::istringstream iss(configLineRead);
        std::string paramKey, paramValue;
        iss >> paramKey;

        trim(paramKey);
        std::getline(iss, paramValue);
        trim(paramValue);

        if (paramKey == key) {
            value = paramValue;
            break;
        }
    }
    configFile.close();
    if (value.empty()) {
        throw std::runtime_error("Failed to retrieve value for key: " + key);
    }
    return value;
}

/* Returns true if a topic is available */
static bool isTopicAvailable(std::string topic)
{
    ros::master::V_TopicInfo masterTopics;
    ros::master::getTopics(masterTopics);

    // Iterate through the topics to check if the topic is available
    for (const auto &topicEntry : masterTopics) {
        if (topicEntry.name == topic) {
            return true;
        }
    }
    return false;
}

/* Returns true if all the topics in a list are available*/
bool checkTopics(std::vector<std::string> &topicsList)
{
    bool success = true;
    for (std::string topic : topicsList) {
        if (!isTopicAvailable(topic)) {
            success = false;
            ROS_ERROR_STREAM("[" << topic << "] NOT FOUND");
        }
    }
    return success;
}

/* Returns true if all the services in a list are available*/
bool checkServices(std::vector<std::string> &servicesList)
{
    bool success = true;
    for (std::string service : servicesList) {
        if (!ros::service::exists(service, false)) {
            success = false;
            ROS_ERROR_STREAM("[" << service << "] NOT FOUND");
        }
    }
    return success;
}

/* 
    Stores the result of a nodes execution in the paramteter server.
    To be used by the test node.
*/
static void storeResult(std::string key, int value=-1)
{
    std::string testParameterPath = "/behaviorControllerTest/";
    ros::param::set((testParameterPath + key), value);
}

/****************************** */
