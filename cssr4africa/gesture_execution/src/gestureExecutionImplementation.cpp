/* actuatorTestImplementation.cpp
*
* Author: Adedayo Akinade
* Date: March 28, 2024
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



#include "gesture_execution/gestureExecution.h"

#define MIN_BOW_ANGLE           0   
#define MAX_BOW_ANGLE           45

#define MIN_NOD_ANGLE           0
#define MAX_NOD_ANGLE           45

#define MIN_GESTURE_DURATION    0
#define MAX_GESTURE_DURATION    10000

#define MIN_POINT_DURATION      0
#define MAX_POINT_DURATION      10000

#define MIN_BOW_DURATION        0
#define MAX_BOW_DURATION        10000

#define MIN_NOD_DURATION        0
#define MAX_NOD_DURATION        10000

// These coordinates are defined by the envelope of the robot. This is how wide the robot can reach to point
#define MIN_X_COORDINATE        -5.0
#define MAX_X_COORDINATE        5.0

#define MIN_Y_COORDINATE        -5.0
#define MAX_Y_COORDINATE        5.0

#define MIN_Z_COORDINATE        -5.0
#define MAX_Z_COORDINATE        5.0

// Home positions for the robot
std::vector<double> rArmHeadHomePosition = {1.7410, -0.09664, 0.09664, 1.6981, -0.05679, -0.2, 0.012271}; // SHoulder pitch, shoulder roll, elbow roll, elbow yaw, wrist yaw, head pitch, head yaw
std::vector<double> rArmHomePosition = {1.7410, -0.09664, 0.09664, 1.6981, -0.05679};
std::vector<double> lArmHeadHomePosition = {1.7625, 0.09970, -0.1334, -1.7150,  0.06592, -0.2, 0.012271};
std::vector<double> lArmHomePosition = {1.7625, 0.09970, -0.1334, -1.7150,  0.06592};
std::vector<double> legHomePosition = {-0.0107, -0.00766, 0.03221};
std::vector<double> headHomePosition = {-0.2, 0.012271};   // Head pitch and yaw

// Stores the states of the joints
extern std::vector<double> legJointStates;
extern std::vector<double> headJointStates;
extern std::vector<double> rArmJointStates;
extern std::vector<double> lArmJointStates;


void initialize_server(string *implementationPlatform, string *interpolationType, string *gestureDescriptorsFileName, string *simulatorTopicsFileName, string *robotTopicsFileName, string *verboseModeInput, bool* verboseMode){
    /* Read gesture execution configuration */
    read_gesture_execution_configuration(implementationPlatform, interpolationType, gestureDescriptorsFileName, simulatorTopicsFileName, robotTopicsFileName, verboseModeInput);

    // read_gesture_execution_input(&gesture_type, &gesture_id, &robot_location_input, &debug_value);
    if (*verboseModeInput == "true"){
        *verboseMode = true;
    }
    else if (*verboseModeInput == "false"){
        *verboseMode = false;
    }
    else{
        printf("Verbose value in input file not supported. Supported verboseMode values are: true and false\n");
        prompt_and_exit(1);
    }

    if (*verboseMode){
        printf("Gesture execution node configuration:\n");
        printf("\tImplementation platform is %s\n", implementationPlatform->c_str());
        printf("\tInterpolation type is %s\n", interpolationType->c_str());
        printf("\tGestures Descriptors file is %s\n", gestureDescriptorsFileName->c_str());
        printf("\tSimulator Topics file is %s\n", simulatorTopicsFileName->c_str());
        printf("\tRobot Topics file is %s\n", robotTopicsFileName->c_str());
        printf("--------------------------------------------------------------------------------------------------------------------------------------------------\n");

    }
}

/* Go to the home position of all joints */
void go_to_home(std::string actuator, std::string topicsFilename, int interpolation, bool debug){
    // ros::Duration(0.5).sleep(); // Wait for one second to ensure that the joint states are updated
    std::vector<double> actuatorState;
    std::vector<double> actuatorHomePosition;
    ControlClientPtr actuatorClient;
    std::vector<std::string> actuatorJointNames;
    std::string actuatorTopic = extract_topic(actuator, topicsFilename);
    int numberOfJoints;
    double homeDuration = 2.0;

    if(actuator == "RArm"){
        actuatorState = rArmJointStates;
        actuatorHomePosition = rArmHomePosition;
        actuatorJointNames = {"RShoulderPitch", "RShoulderRoll", "RElbowRoll", "RElbowYaw", "RWristYaw"};
        actuatorClient = create_client(actuatorTopic);
        numberOfJoints = actuatorJointNames.size();
    }
    else if(actuator == "LArm"){
        actuatorState = lArmJointStates;
        actuatorHomePosition = lArmHomePosition;
        actuatorJointNames = {"LShoulderPitch", "LShoulderRoll", "LElbowRoll", "LElbowYaw", "LWristYaw"};
        actuatorClient = create_client(actuatorTopic);
        numberOfJoints = actuatorJointNames.size();
    }
    else if(actuator == "Leg"){
        actuatorState = legJointStates;
        actuatorHomePosition = legHomePosition;
        actuatorJointNames = {"HipPitch", "HipRoll", "KneePitch"};
        actuatorClient = create_client(actuatorTopic);
        numberOfJoints = actuatorJointNames.size();
    }
    else if(actuator == "Head"){
        actuatorState = headJointStates;
        actuatorHomePosition = headHomePosition;
        actuatorJointNames = {"HeadPitch", "HeadYaw"};
        actuatorClient = create_client(actuatorTopic);
        numberOfJoints = actuatorJointNames.size();
    }

    if (debug){
        printf("Actuator state:\n");
        for(int i = 0; i < actuatorState.size(); i++){
            printf("{%s: %f ", actuatorJointNames[i].c_str(), actuatorState[i]);
        }
        printf("}\n");
    }

    std::vector<std::vector<double>> positions_t;
    std::vector<std::vector<double>> velocities_t;
    std::vector<std::vector<double>> accelerations_t;
    std::vector<double> duration_t;

    compute_trajectory(actuatorState, actuatorHomePosition, actuatorState.size(), homeDuration, positions_t, velocities_t, accelerations_t, duration_t);

    if(interpolation == BIOLOGICAL_MOTION){
        move_to_position_BM(actuatorClient, actuatorJointNames, duration_t, homeDuration, "home", positions_t, velocities_t, accelerations_t);
    }
    else{
        move_to_position(actuatorClient, actuatorJointNames, homeDuration, "home", actuatorHomePosition);
    }


}


/* Read the gesture execution configuration */
void read_gesture_execution_configuration(string* platform, string* interpolation, string* gestureDescriptor, string* simulatorTopics, string* robotTopics, string* verboseMode){
    bool debug_mode = false;   // used to turn debug message on  

    std::string config_file = "gestureExecutionConfiguration.ini";    // data filename
    std::string config_path;                                          // data path
    std::string config_path_and_file;                                 // data path and filename
     
    std::string platformKey = "platform";                           // platform key 
    std::string interpolationKey = "interpolation";                 // interpolation key key
    std::string gestureDescriptorsKey = "gestureDescriptors";       // gesture descriptors key
    std::string simulatorTopicsKey = "simulatorTopics";             // simulator topics key
    std::string robotTopicsKey = "robotTopics";                     // robot topics key
    std::string verboseModeKey = "verboseMode";                     // verbose mode key

//     platform                    robot
// interpolation               linear
// gestureDescriptors          gestureDescriptors.dat
// simulatorTopics             simulatorTopics.dat
// robotTopics                 pepperTopics.dat
// verboseMode                 false

    std::string platformValue;                                      // platform key 
    std::string interpolationValue;                                  // interpolation key key
    std::string gestureDescriptorsValue;                            // gesture descriptors key
    std::string simulatorTopicsValue;                               // simulator topics key
    std::string robotTopicsValue;                                   // robot topics key
    std::string verboseModeValue;                                   // verbose mode key

    // Construct the full path of the configuration file
    #ifdef ROS
        config_path = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        data_path = "..";
    #endif

    // set configuration path
    config_path += "/config/";
    config_path_and_file = config_path;
    config_path_and_file += config_file;

    if (debug_mode) printf("COnfig file is %s\n", config_path_and_file.c_str());

    // Open configuration file
    std::ifstream data_if(config_path_and_file.c_str());
    if (!data_if.is_open()){
        printf("Unable to open the data file %s\n", config_path_and_file.c_str());
        prompt_and_exit(1);
    }

    std::string dataLineRead;  // variable to read the line in the file
    // Get key-value pairs from the configuration file
    while(std::getline(data_if, dataLineRead)){
        std::istringstream iss(dataLineRead);
        std::string paramKey, paramValue;
        iss >> paramKey;
        trim(paramKey);
        std::getline(iss, paramValue);
        iss >> paramValue;
        trim(paramValue);
        // printf("paramKey: %s; paramValue: %s\n", paramKey.c_str(), paramValue.c_str());
        
        if (paramKey == platformKey){ 
            boost::algorithm::to_lower(paramValue); // modifies string to lower case
            platformValue = paramValue;
            *platform = paramValue;
        }
        
        else if (paramKey == interpolationKey){ 
            interpolationValue = paramValue;
            boost::algorithm::to_lower(paramValue); // modifies string to lower case
            *interpolation = paramValue;
        }

        else if (paramKey == gestureDescriptorsKey){ 
            gestureDescriptorsValue = paramValue;
            *gestureDescriptor = paramValue;
        }

        else if (paramKey == simulatorTopicsKey){ 
            simulatorTopicsValue = paramValue;
            *simulatorTopics = paramValue;
        }

        else if (paramKey == robotTopicsKey){ 
            robotTopicsValue = paramValue;
            *robotTopics = paramValue;
        }

        else if (paramKey == verboseModeKey){ 
            boost::algorithm::to_lower(paramValue); // modifies string to lower case
            verboseModeValue = paramValue;
            *verboseMode = paramValue;
        }
    }
    data_if.close();
}

/* Read gesture descriptors */
void read_gesture_descriptors(string gesture_descriptors_file, string* gesture_type, string* gesture_id, int* number_of_waypoints, string* joint_angles){
    bool debug_mode = false;   // used to turn debug message on  

    std::string data_file = gesture_descriptors_file;    // data filename
    std::string data_path;                              // data path
    std::string data_path_and_file;                     // data path and filename
     
    std::string gestureTypeKey = "type";                // gesture type key
    std::string gestureIDKey = "ID";                    // gesture ID key
    std::string wayPointKey = "wayPoint";               // way point key
    std::string jointAngles = "jointAngles";            // joint angles key

    std::string gestureTypeValue;                       // gesture type value
    std::string gestureIDValue;                         // gesture ID value
    std::string wayPointValue;                          // way point value
    std::string jointAnglesValue;                       // joint angles value

    // Construct the full path of the configuration file
    #ifdef ROS
        data_path = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        data_path = "..";
    #endif

    // set configuration path
    data_path += "/data/";
    data_path_and_file = data_path;
    data_path_and_file += data_file;

    if (debug_mode) printf("Data file is %s\n", data_path_and_file.c_str());

    // Open descriptors file
    std::ifstream data_if(data_path_and_file.c_str());
    if (!data_if.is_open()){
        printf("Unable to open the data file %s\n", data_path_and_file.c_str());
        prompt_and_exit(1);
    }

    std::string dataLineRead;  // variable to read the line in the file
    // Get key-value pairs from the configuration file
    while(std::getline(data_if, dataLineRead)){
        std::istringstream iss(dataLineRead);
        std::string paramKey, paramValue;
        iss >> paramKey;
        trim(paramKey);
        std::getline(iss, paramValue);
        iss >> paramValue;
        trim(paramValue);

        // printf("paramKey: %s; paramValue: %s\n", paramKey.c_str(), paramValue.c_str());
        
        if (paramKey == gestureTypeKey){ 
            gestureTypeValue = paramValue;
            *gesture_type = paramValue;
        }
        
        else if (paramKey == gestureIDKey){ 
            gestureIDValue = paramValue;
            *gesture_id = paramValue;
        }

        else if (paramKey == wayPointKey){ 
            wayPointValue = paramValue;
            *number_of_waypoints = std::stoi(paramValue);
        }

        else if (paramKey == jointAngles){ 
            jointAnglesValue = paramValue;
            *joint_angles = paramValue;
        }
    }
    data_if.close();
}

/* Read the gesture execution input */
void read_gesture_execution_input(string* gesture_type, string* gesture_id, string* robot_location_input, string* debug){
    bool debug_mode = false;   // used to turn debug message on  

    std::string data_file = "gestureExecutionInput.dat";    // data filename
    std::string data_path;                                  // data path
    std::string data_path_and_file;                         // data path and filename
     
    std::string gestureTypeKey = "type";                     // gesture type key 
    std::string gestureIDKey = "ID";                         // gesture ID key
    std::string robotLocationKey = "robotLocation";         // robot location key
    std::string debugKey = "debug";                          // debug key

    std::string gestureTypeValue;                            // gesture type value
    std::string gestureIDValue;                              // gesture ID value
    std::string robotLocationValue;                          // robot location value
    std::string debugValue;                                  // debug value

    // Construct the full path of the configuration file
    #ifdef ROS
        data_path = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        data_path = "..";
    #endif

    // set configuration path
    data_path += "/data/";
    data_path_and_file = data_path;
    data_path_and_file += data_file;

    if (debug_mode) printf("Data file is %s\n", data_path_and_file.c_str());

    // Open configuration file
    std::ifstream data_if(data_path_and_file.c_str());
    if (!data_if.is_open()){
        printf("Unable to open the data file %s\n", data_path_and_file.c_str());
        prompt_and_exit(1);
    }

    std::string dataLineRead;  // variable to read the line in the file
    // Get key-value pairs from the configuration file
    while(std::getline(data_if, dataLineRead)){
        std::istringstream iss(dataLineRead);
        std::string paramKey, paramValue;
        iss >> paramKey;
        trim(paramKey);
        std::getline(iss, paramValue);
        iss >> paramValue;
        trim(paramValue);

        // printf("paramKey: %s; paramValue: %s\n", paramKey.c_str(), paramValue.c_str());
        
        if (paramKey == gestureTypeKey){ 
            gestureTypeValue = paramValue;
            *gesture_type = paramValue;
        }
        
        else if (paramKey == gestureIDKey){ 
            gestureIDValue = paramValue;
            *gesture_id = paramValue;
        }

        else if (paramKey == robotLocationKey){ 
            robotLocationValue = paramValue;
            *robot_location_input = paramValue;
        }

        else if (paramKey == debugKey){ 
            debugValue = paramValue;
            *debug = paramValue;
        }
    }
    data_if.close();
}

/* Modified Execute pointing gesture -- ID 01 */
int pointing_gesture(float point_x, float point_y, float point_z, int gesture_duration, string topics_file, int interpolation, bool debug){
    if (debug){
        printf("Executing pointing gesture\n");
    }
    double elbow_x       = 0.0;
    double elbow_y       = 0.0;
    double elbow_z       = 0.0;
    double point_angle   = 45;
    int pointing_arm    = LEFT_ARM;
    float l_1           = UPPER_ARM_LENGTH;
    float l_2           = 0.0;
    float x_s           = SHOULDER_OFFSET_X;
    float y_s           = SHOULDER_OFFSET_Y;
    float z_s           = SHOULDER_OFFSET_Z + TORSO_HEIGHT;

    // Head coordinates
    double head_x       = -38.0;
    double head_y       = 0.0;
    double head_z       = 169.9 + TORSO_HEIGHT;
    double l_head_1     = 112.051;
    double l_head_2     = 0.0;
    double camera_x     = 0.0;
    double camera_y     = 0.0;
    double camera_z     = 0.0;
    // float x_s           = 0.0;
    // float y_s           = 0.0;
    // float z_s           = TORSO_HEIGHT;
    double duration     = gesture_duration/1000.0;
    // duration = 2.0;

    if(point_y < 0){
        pointing_arm = RIGHT_ARM;
        y_s = -y_s;
    }

    double distance = sqrt(pow((point_x - x_s), 2) + pow((point_y - y_s), 2) + pow((point_z - z_s), 2));
    l_2 = distance - l_1;

    elbow_x = ((l_1 * point_x) + (l_2 * x_s))/(l_1 + l_2);
    elbow_y = ((l_1 * point_y) + (l_2 * y_s))/(l_1 + l_2);
    // -(SHOULDER_OFFSET_Y+ELBOW_OFFSET_Y+HAND_OFFSET_Y)
    elbow_z = ((l_1 * point_z) + (l_2 * z_s))/(l_1 + l_2);
    elbow_z = elbow_z - TORSO_HEIGHT;

    if (debug){
        printf("L1: %.3f, L2: %.3f\n", l_1, l_2);
        printf("Distance: %.3f, sum l1_l2: %.3f\n", distance, l_1 + l_2);
        printf("Pointing coordinates: %.3f, %.3f, %.3f\n", point_x, point_y, point_z);
        printf("Elbow coordinates: %.3f, %.3f, %.3f\n", elbow_x, elbow_y, elbow_z);
        printf("Pointing arm: %d\n", pointing_arm);
    }

    distance = sqrt(pow((point_x - head_x), 2) + pow((point_y - head_y), 2) + pow((point_z - head_z), 2));
    l_head_2 = distance - l_head_1;

    camera_x = ((l_head_1 * point_x) + (l_head_2 * head_x))/(l_head_1 + l_head_2);
    camera_y = ((l_head_1 * point_y) + (l_head_2 * head_y))/(l_head_1 + l_head_2);
    camera_z = ((l_head_1 * point_z) + (l_head_2 * head_z))/(l_head_1 + l_head_2);
    camera_z = camera_z;

    if(debug){
        printf("L1: %.3f, L2: %.3f\n", l_head_1, l_head_2);
        printf("Distance: %.3f, sum l1_l2: %.3f\n", distance, l_head_1 + l_head_2);
        printf("Pointing coordinates: %.3f, %.3f, %.3f\n", point_x, point_y, point_z);
        printf("Camera coordinates: %.3f, %.3f, %.3f\n", camera_x, camera_y, camera_z);
    }

    double shoulderPitch, shoulderRoll, elbowYaw, elbowRoll, wristYaw;
    double shoulderPitchMax;
    double shoulderPitchMin;
    double shoulderRollMax;
    double shoulderRollMin;

    double headYaw, headPitch;

    if(pointing_arm == RIGHT_ARM){
        shoulderPitchMin = MIN_RSHOULDER_PITCH;
        shoulderPitchMax = MAX_RSHOULDER_PITCH;
        shoulderRollMin = MIN_RSHOULDER_ROLL;
        shoulderRollMax = MAX_RSHOULDER_ROLL;
    }
    else if(pointing_arm == LEFT_ARM){
        shoulderPitchMin = MIN_LSHOULDER_PITCH;
        shoulderPitchMax = MAX_LSHOULDER_PITCH;
        shoulderRollMin = MIN_LSHOULDER_ROLL;
        shoulderRollMax = MAX_LSHOULDER_ROLL;
    }

    // inverseKinematicsArm(pointing_arm, elbow_x, elbow_y, elbow_z);
    // getArmAngles(pointing_arm, point_x, point_y, point_z, 0.0, 0.0, 0.0, &shoulderPitch, &shoulderRoll); // POinting forward left arm
    getArmAngles(pointing_arm, elbow_x, elbow_y, elbow_z, 0.0, 0.0, 0.0, &shoulderPitch, &shoulderRoll); // POinting forward left arm

    getHeadAngles(camera_x, camera_y, camera_z, &headYaw, &headPitch);
    // 33.26, -304, 120.17
    if(debug){
        printf("Shoulder Pitch: %f, Shoulder Roll: %f\n", shoulderPitch, shoulderRoll);
        printf("Head Yaw: %f, Head Pitch: %f\n", headYaw, headPitch);
    }

    if(shoulderPitch < shoulderPitchMin || shoulderPitch > shoulderPitchMax){
        if(debug)
            printf("Shoulder pitch value out of range\n");
        return 0;
    }
    if(shoulderRoll < shoulderRollMin || shoulderRoll > shoulderRollMax){
        if(debug)
            printf("Shoulder roll value out of range\n");
        return 0; // gesture not executed successfully
    }
    if(isnan(shoulderPitch) || isnan(shoulderRoll)){
        if(debug)
            printf("Invalid shoulder pitch or shoulder roll value\n");
        return 0; // gesture not executed successfully
    }


    std::string armTopic;
    std::string headTopic = extract_topic("Head", topics_file);
    std::string handTopic;

    if(pointing_arm == RIGHT_ARM){
        armTopic = extract_topic("RArm", topics_file);
        printf("Arm Topic: %s\n", armTopic.c_str());
        handTopic = extract_topic("RHand", topics_file);
        // elbowYaw = 0.6969355558937891;   // Default value
        // elbowRoll = 0.8999453212889488;     // Default value
        elbowYaw = 2.0857;
        elbowRoll = 0.0;
        wristYaw = -0.05679;
        go_to_home("RArm", topics_file, interpolation, debug);
        rHand(handTopic, "open");
        // rArmHeadPointing(armTopic, headTopic, shoulderPitch, shoulderRoll, elbowYaw, elbowRoll, wristYaw, headPitch, headYaw, duration, interpolation, debug);
        headPointing(headTopic, headPitch, headYaw, duration, interpolation, debug);
        rArmPointing(armTopic, shoulderPitch, shoulderRoll, elbowYaw, elbowRoll, wristYaw, duration, interpolation, debug);
        go_to_home("Head", topics_file, interpolation, debug);
        go_to_home("RArm", topics_file, interpolation, debug);
        rHand(handTopic, "home");
    }
    else if(pointing_arm == LEFT_ARM){
        armTopic = extract_topic("LArm", topics_file);
        printf("Arm Topic: %s\n", armTopic.c_str());
        handTopic = extract_topic("LHand", topics_file);
        // elbowYaw = -2.426597396781068;  // Default value
        // elbowRoll = -1.347202561275251; // Default value
        elbowYaw = -1.5620;
        elbowRoll = -0.0;
        wristYaw = 0.06592;
        go_to_home("LArm", topics_file, interpolation, debug);
        lHand(handTopic, "open");
        headPointing(headTopic, headPitch, headYaw, duration, interpolation, debug);
        lArmPointing(armTopic, shoulderPitch, shoulderRoll, elbowYaw, elbowRoll, wristYaw, duration, interpolation, debug);
        go_to_home("Head", topics_file, interpolation, debug);
        go_to_home("LArm", topics_file, interpolation, debug);
        lHand(handTopic, "home");
    }

    return 1;       // gesture executed successfully
}

// ---------------------------------------------------------------------------
/*  Modified Execute pointing gesture -- ID 01 */
/* int pointing_gesture(float point_x, float point_y, float point_z, int gesture_duration, string topics_file, int interpolation, bool debug){
    if (debug){
        printf("Executing pointing gesture\n");
    }
    double elbow_x       = 0.0;
    double elbow_y       = 0.0;
    double elbow_z       = 0.0;
    double point_angle   = 45;
    int pointing_arm    = LEFT_ARM;
    float l_1           = UPPER_ARM_LENGTH;
    float l_2           = 0.0;
    float x_s           = SHOULDER_OFFSET_X;
    float y_s           = SHOULDER_OFFSET_Y;
    float z_s           = SHOULDER_OFFSET_Z + TORSO_HEIGHT;
    // float x_s           = 0.0;
    // float y_s           = 0.0;
    // float z_s           = TORSO_HEIGHT;
    double duration     = gesture_duration/1000.0;
    // duration = 2.0;

    if(point_y < 0){
        pointing_arm = RIGHT_ARM;
        y_s = -y_s;
    }

    double distance = sqrt(pow((point_x - x_s), 2) + pow((point_y - y_s), 2) + pow((point_z - z_s), 2));
    l_2 = distance - l_1;

    elbow_x = ((l_1 * point_x) + (l_2 * x_s))/(l_1 + l_2);
    elbow_y = ((l_1 * point_y) + (l_2 * y_s))/(l_1 + l_2);
    // -(SHOULDER_OFFSET_Y+ELBOW_OFFSET_Y+HAND_OFFSET_Y)
    elbow_z = ((l_1 * point_z) + (l_2 * z_s))/(l_1 + l_2);
    elbow_z = elbow_z - TORSO_HEIGHT;

    if (debug){
        printf("L1: %.3f, L2: %.3f\n", l_1, l_2);
        printf("Distance: %.3f, sum l1_l2: %.3f\n", distance, l_1 + l_2);
        printf("Pointing coordinates: %.3f, %.3f, %.3f\n", point_x, point_y, point_z);
        printf("Elbow coordinates: %.3f, %.3f, %.3f\n", elbow_x, elbow_y, elbow_z);
        printf("Pointing arm: %d\n", pointing_arm);
    }

    double shoulderPitch, shoulderRoll, elbowYaw, elbowRoll, wristYaw;
    double shoulderPitchMax;
    double shoulderPitchMin;
    double shoulderRollMax;
    double shoulderRollMin;

    if(pointing_arm == RIGHT_ARM){
        shoulderPitchMin = MIN_RSHOULDER_PITCH;
        shoulderPitchMax = MAX_RSHOULDER_PITCH;
        shoulderRollMin = MIN_RSHOULDER_ROLL;
        shoulderRollMax = MAX_RSHOULDER_ROLL;
    }
    else if(pointing_arm == LEFT_ARM){
        shoulderPitchMin = MIN_LSHOULDER_PITCH;
        shoulderPitchMax = MAX_LSHOULDER_PITCH;
        shoulderRollMin = MIN_LSHOULDER_ROLL;
        shoulderRollMax = MAX_LSHOULDER_ROLL;
    }

    // inverseKinematicsArm(pointing_arm, elbow_x, elbow_y, elbow_z);
    // getArmAngles(pointing_arm, point_x, point_y, point_z, 0.0, 0.0, 0.0, &shoulderPitch, &shoulderRoll); // POinting forward left arm
    getArmAngles(pointing_arm, elbow_x, elbow_y, elbow_z, 0.0, 0.0, 0.0, &shoulderPitch, &shoulderRoll); // POinting forward left arm
    // 33.26, -304, 120.17
    if(debug)
        printf("Shoulder Pitch: %f, Shoulder Roll: %f\n", shoulderPitch, shoulderRoll);

    if(shoulderPitch < shoulderPitchMin || shoulderPitch > shoulderPitchMax){
        if(debug)
            printf("Shoulder pitch value out of range\n");
        return 0;
    }
    if(shoulderRoll < shoulderRollMin || shoulderRoll > shoulderRollMax){
        if(debug)
            printf("Shoulder roll value out of range\n");
        return 0; // gesture not executed successfully
    }
    if(isnan(shoulderPitch) || isnan(shoulderRoll)){
        if(debug)
            printf("Invalid shoulder pitch or shoulder roll value\n");
        return 0; // gesture not executed successfully
    }

    std::string armTopic;

    if(pointing_arm == RIGHT_ARM){
        armTopic = extract_topic("RArm", topics_file);
        printf("Arm Topic: %s\n", armTopic.c_str());
        // elbowYaw = 0.6969355558937891;   // Default value
        // elbowRoll = 0.8999453212889488;     // Default value
        elbowYaw = 2.0857;
        elbowRoll = 0.0;
        wristYaw = -0.05679;
        go_to_home("RArm", topics_file, interpolation, debug);
        rArmPointing(armTopic, shoulderPitch, shoulderRoll, elbowYaw, elbowRoll, wristYaw, duration, interpolation, debug);
        go_to_home("RArm", topics_file, interpolation, debug);
    }
    else if(pointing_arm == LEFT_ARM){
        armTopic = extract_topic("LArm", topics_file);
        printf("Arm Topic: %s\n", armTopic.c_str());
        // elbowYaw = -2.426597396781068;  // Default value
        // elbowRoll = -1.347202561275251; // Default value
        elbowYaw = -1.5620;
        elbowRoll = -0.0;
        wristYaw = 0.06592;
        go_to_home("LArm", topics_file, interpolation, debug);
        lArmPointing(armTopic, shoulderPitch, shoulderRoll, elbowYaw, elbowRoll, wristYaw, duration, interpolation, debug);
        go_to_home("LArm", topics_file, interpolation, debug);
    }

    return 1;       // gesture executed successfully
} */
// -------------------------------------------------------------------------------------

/* Modified Execute pointing gesture -- ID 01 */
/* int pointing_gesture_new(float point_x, float point_y, float point_z, int gesture_duration, string topics_file, int interpolation, bool debug){
    if (debug){
        printf("Executing pointing gesture\n");
    }
    double elbow_x       = 0.0;
    double elbow_y       = 0.0;
    double elbow_z       = 0.0;
    double point_angle   = 45;
    int pointing_arm    = LEFT_ARM;
    float l_1           = UPPER_ARM_LENGTH;
    float l_2           = 0.0;
    float x_s           = SHOULDER_OFFSET_X;
    float y_s           = SHOULDER_OFFSET_Y;
    float z_s           = SHOULDER_OFFSET_Z + TORSO_HEIGHT;

    // Head coordinates
    double head_x       = -38.0;
    double head_y       = 0.0;
    double head_z       = 169.9 + TORSO_HEIGHT;
    double l_head_1     = 112.051;
    double l_head_2     = 0.0;
    double camera_x     = 0.0;
    double camera_y     = 0.0;
    double camera_z     = 0.0;
    // float x_s           = 0.0;
    // float y_s           = 0.0;
    // float z_s           = TORSO_HEIGHT;
    double duration     = gesture_duration/1000.0;
    // duration = 2.0;

    if(point_y < 0){
        pointing_arm = RIGHT_ARM;
        y_s = -y_s;
    }

    double distance = sqrt(pow((point_x - x_s), 2) + pow((point_y - y_s), 2) + pow((point_z - z_s), 2));
    l_2 = distance - l_1;

    elbow_x = ((l_1 * point_x) + (l_2 * x_s))/(l_1 + l_2);
    elbow_y = ((l_1 * point_y) + (l_2 * y_s))/(l_1 + l_2);
    // -(SHOULDER_OFFSET_Y+ELBOW_OFFSET_Y+HAND_OFFSET_Y)
    elbow_z = ((l_1 * point_z) + (l_2 * z_s))/(l_1 + l_2);
    elbow_z = elbow_z - TORSO_HEIGHT;

    if (debug){
        printf("L1: %.3f, L2: %.3f\n", l_1, l_2);
        printf("Distance: %.3f, sum l1_l2: %.3f\n", distance, l_1 + l_2);
        printf("Pointing coordinates: %.3f, %.3f, %.3f\n", point_x, point_y, point_z);
        printf("Elbow coordinates: %.3f, %.3f, %.3f\n", elbow_x, elbow_y, elbow_z);
        printf("Pointing arm: %d\n", pointing_arm);
    }

    distance = sqrt(pow((point_x - head_x), 2) + pow((point_y - head_y), 2) + pow((point_z - head_z), 2));
    l_head_2 = distance - l_head_1;

    camera_x = ((l_head_1 * point_x) + (l_head_2 * head_x))/(l_head_1 + l_head_2);
    camera_y = ((l_head_1 * point_y) + (l_head_2 * head_y))/(l_head_1 + l_head_2);
    camera_z = ((l_head_1 * point_z) + (l_head_2 * head_z))/(l_head_1 + l_head_2);
    camera_z = camera_z - TORSO_HEIGHT;

    if(debug){
        printf("L1: %.3f, L2: %.3f\n", l_head_1, l_head_2);
        printf("Distance: %.3f, sum l1_l2: %.3f\n", distance, l_head_1 + l_head_2);
        printf("Pointing coordinates: %.3f, %.3f, %.3f\n", point_x, point_y, point_z);
        printf("Camera coordinates: %.3f, %.3f, %.3f\n", camera_x, camera_y, camera_z);
    }

    double shoulderPitch, shoulderRoll, elbowYaw, elbowRoll, wristYaw;
    double shoulderPitchMax;
    double shoulderPitchMin;
    double shoulderRollMax;
    double shoulderRollMin;

    double headYaw, headPitch;

    if(pointing_arm == RIGHT_ARM){
        shoulderPitchMin = MIN_RSHOULDER_PITCH;
        shoulderPitchMax = MAX_RSHOULDER_PITCH;
        shoulderRollMin = MIN_RSHOULDER_ROLL;
        shoulderRollMax = MAX_RSHOULDER_ROLL;
    }
    else if(pointing_arm == LEFT_ARM){
        shoulderPitchMin = MIN_LSHOULDER_PITCH;
        shoulderPitchMax = MAX_LSHOULDER_PITCH;
        shoulderRollMin = MIN_LSHOULDER_ROLL;
        shoulderRollMax = MAX_LSHOULDER_ROLL;
    }

    // inverseKinematicsArm(pointing_arm, elbow_x, elbow_y, elbow_z);
    // getArmAngles(pointing_arm, point_x, point_y, point_z, 0.0, 0.0, 0.0, &shoulderPitch, &shoulderRoll); // POinting forward left arm
    getArmAngles(pointing_arm, elbow_x, elbow_y, elbow_z, 0.0, 0.0, 0.0, &shoulderPitch, &shoulderRoll); // POinting forward left arm

    getHeadAngles(camera_x, camera_y, camera_z, &headYaw, &headPitch);
    // 33.26, -304, 120.17
    if(debug){
        printf("Shoulder Pitch: %f, Shoulder Roll: %f\n", shoulderPitch, shoulderRoll);
        printf("Head Yaw: %f, Head Pitch: %f\n", headYaw, headPitch);
    }

    if(shoulderPitch < shoulderPitchMin || shoulderPitch > shoulderPitchMax){
        if(debug)
            printf("Shoulder pitch value out of range\n");
        return 0;
    }
    if(shoulderRoll < shoulderRollMin || shoulderRoll > shoulderRollMax){
        if(debug)
            printf("Shoulder roll value out of range\n");
        return 0; // gesture not executed successfully
    }
    if(isnan(shoulderPitch) || isnan(shoulderRoll)){
        if(debug)
            printf("Invalid shoulder pitch or shoulder roll value\n");
        return 0; // gesture not executed successfully
    }


    std::string armTopic;

    if(pointing_arm == RIGHT_ARM){
        armTopic = extract_topic("RArm", topics_file);
        printf("Arm Topic: %s\n", armTopic.c_str());
        // elbowYaw = 0.6969355558937891;   // Default value
        // elbowRoll = 0.8999453212889488;     // Default value
        elbowYaw = 2.0857;
        elbowRoll = 0.0;
        wristYaw = -0.05679;
        go_to_home("RArm", topics_file, interpolation, debug);
        rArmPointingOld(armTopic, shoulderPitch, shoulderRoll, elbowYaw, elbowRoll, wristYaw, duration, interpolation, debug);
        // rArmPointing(armTopic, shoulderPitch, shoulderRoll, elbowYaw, elbowRoll, wristYaw, headPitch, headYaw, duration, interpolation, debug);
        go_to_home("RArm", topics_file, interpolation, debug);
    }
    else if(pointing_arm == LEFT_ARM){
        armTopic = extract_topic("LArm", topics_file);
        printf("Arm Topic: %s\n", armTopic.c_str());
        // elbowYaw = -2.426597396781068;  // Default value
        // elbowRoll = -1.347202561275251; // Default value
        elbowYaw = -1.5620;
        elbowRoll = -0.0;
        wristYaw = 0.06592;
        go_to_home("LArm", topics_file, interpolation, debug);
        lArmPointingOld(armTopic, shoulderPitch, shoulderRoll, elbowYaw, elbowRoll, wristYaw, duration, interpolation, debug);
        // lArmPointing(armTopic, shoulderPitch, shoulderRoll, elbowYaw, elbowRoll, wristYaw, headPitch, headYaw, duration, interpolation, debug);
        go_to_home("LArm", topics_file, interpolation, debug);
    }

    return 1;       // gesture executed successfully
} */


/* Modified Execute bowing gesture -- ID 02 */
int bowing_gesture(int bow_angle, int gesture_duration, string topics_file, int interpolation, bool debug){
    if (debug){
        printf("Executing bowing gesture\n");
    }

    double duration = gesture_duration/1000.0;

    // Check parameters are within limits

    std::string legTopic = extract_topic("Leg", topics_file);
    if(debug)
        printf("Leg Topic: %s\n", legTopic.c_str());
    // leg(legTopic, bow_angle, duration, debug);
    go_to_home("Leg", topics_file, interpolation, debug);
    legBowing(legTopic, bow_angle, gesture_duration, interpolation, debug);
    go_to_home("Leg", topics_file, interpolation, debug);

    return 1;       // gesture executed successfully
}

/* Modified Execute nodding gesture -- ID 03 */
int nodding_gesture(int bow_angle, int gesture_duration, string topics_file, int interpolation, bool debug){
    if (debug){
        printf("Executing nodding gesture\n");
    }
    
    double duration = gesture_duration/1000.0;
    // Check parameters are within limits

    std::string headTopic = extract_topic("Head", topics_file);
    if(debug)
        printf("Leg Topic: %s\n", headTopic.c_str());
    // go_to_home("Head", topics_file, interpolation, debug);
    // headNodding(headTopic, bow_angle, gesture_duration, interpolation, debug);
    // go_to_home("Head", topics_file, interpolation, debug);
    head(headTopic, bow_angle, gesture_duration, debug);

    return 1;       // gesture executed successfully
}

/* Extract topic names for the respective simulator or physical robot */
string extract_topic(string key, string topic_file_name){
    bool debug = false;   // used to turn debug message on
    
    std::string topic_path;                                   // topic filename path
    std::string topic_path_and_file;                          // topic with path and file 

    std::string topic_value = "";                             // topic value

    // Construct the full path of the topic file
    #ifdef ROS
        topic_path = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        topic_path = "..";
    #endif

    // set topic path    
    topic_path += "/data/";
    topic_path_and_file = topic_path;
    topic_path_and_file += topic_file_name;

    if (debug) printf("Topic file is %s\n", topic_path_and_file.c_str());

    // Open topic file
    std::ifstream topic_if(topic_path_and_file.c_str());
    if (!topic_if.is_open()){
        printf("Unable to open the topic file %s\n", topic_path_and_file.c_str());
        prompt_and_exit(1);
    }

    std::string topicLineRead;   // variable to read the line in the file
    // Get key-value pairs from the topic file
    while(std::getline(topic_if, topicLineRead)){
        std::istringstream iss(topicLineRead);
        std::string paramKey, paramValue;
        iss >> paramKey;
        trim(paramKey);
        std::getline(iss, paramValue);
        iss >> paramValue;
        trim(paramValue);
        if (paramKey == key) {
            topic_value = paramValue;
            break;
        }
    }
    topic_if.close();

    // verify the topic_value is not empty
    if (topic_value == ""){
        printf("Unable to find a valid topic.\n");
        prompt_and_exit(1);
    }
    return topic_value;
}

ControlClientPtr create_client(const std::string& TopicName) {
    ControlClientPtr actionClient(new ControlClient(TopicName, true));
    int maxIterations = 10;

    for (int iterations = 0; iterations < maxIterations; ++iterations) {
        if (actionClient->waitForServer(ros::Duration(5.0))) {
            return actionClient;
        }
        ROS_DEBUG("Waiting for the %s controller to come up", TopicName.c_str());
    }

    throw std::runtime_error("Error creating action client for " + TopicName + " controller: Server not available");
}

void compute_trajectory(std::vector<double> startPosition, std::vector<double> endPosition, int numberOfJoints,
                        double trajectoryDuration, std::vector<std::vector<double>>& positions, 
                        std::vector<std::vector<double>>& velocities, std::vector<std::vector<double>>& accelerations, 
                        std::vector<double>& durations){
    double t_t = 0;
    std::vector<double> x_t;
    std::vector<double> v_t;
    std::vector<double> a_t;
    std::vector<double> duration_t;
    double a, v, x;
    double time_step = 1;

    // Clear the existing values
    for(int i = 0; i < positions.size(); i++){
        positions[i].clear();
        velocities[i].clear();
        accelerations[i].clear();
    }
    positions.clear();
    velocities.clear();
    accelerations.clear();

    // Calculate the trajectory

    while(t_t < trajectoryDuration){
        for(int i = 0; i < numberOfJoints; i++){     // Create a trajectory for each joint (5 joints for the arm)
            x = startPosition[i] + (endPosition[i] - startPosition[i]) * ((10 * (pow(t_t/trajectoryDuration, 3))) - (15 * (pow(t_t/trajectoryDuration, 4))) + (6 * (pow(t_t/trajectoryDuration, 5))));
            x_t.push_back(x);
            // printf("%f, ", x_t[i]);
            v = ((endPosition[i] - startPosition[i])/trajectoryDuration) * ((30 * (pow(t_t/trajectoryDuration, 2))) - (60 * (pow(t_t/trajectoryDuration, 3))) + (30 * (pow(t_t/trajectoryDuration, 4))));
            v_t.push_back(v);
            // printf("%f, ", v_t[i]);
            a = ((endPosition[i] - startPosition[i])/(trajectoryDuration*trajectoryDuration)) * ((60 * (pow(t_t/trajectoryDuration, 1))) - (180 * (pow(t_t/trajectoryDuration, 2))) + (120 * (pow(t_t/trajectoryDuration, 3))));
            a_t.push_back(a);

        // p   rintf("%f, ", a_t[i]);
        }
        positions.push_back(x_t);
        velocities.push_back(v_t);
        accelerations.push_back(a_t);
        durations.push_back(t_t);
        t_t = t_t + time_step;
        x_t.clear();
        v_t.clear();
        a_t.clear();
    }
    t_t = trajectoryDuration;
    for(int i = 0; i < numberOfJoints; i++){
        x = startPosition[i] + (endPosition[i] - startPosition[i]) * ((10 * (pow(t_t/trajectoryDuration, 3))) - (15 * (pow(t_t/trajectoryDuration, 4))) + (6 * (pow(t_t/trajectoryDuration, 5))));
        x_t.push_back(x);
        // printf("%f, ", x_t[i]);
        v = ((endPosition[i] - startPosition[i])/trajectoryDuration) * ((30 * (pow(t_t/trajectoryDuration, 2))) - (60 * (pow(t_t/trajectoryDuration, 3))) + (30 * (pow(t_t/trajectoryDuration, 4))));
        v_t.push_back(v);
        // printf("%f, ", v_t[i]);
        a = ((endPosition[i] - startPosition[i])/(trajectoryDuration*trajectoryDuration)) * ((60 * (pow(t_t/trajectoryDuration, 1))) - (180 * (pow(t_t/trajectoryDuration, 2))) + (120 * (pow(t_t/trajectoryDuration, 3))));
        a_t.push_back(a);
        // printf("%f, ", a_t[i]);
    }
    positions.push_back(x_t);
    velocities.push_back(v_t);
    accelerations.push_back(a_t);
    durations.push_back(t_t);

    return;
}

void move_to_position(ControlClientPtr& client, const std::vector<std::string>& jointNames, double duration, 
                        const std::string& positionName, std::vector<double> positions){
    
    control_msgs::FollowJointTrajectoryGoal goal;
    trajectory_msgs::JointTrajectory& trajectory = goal.trajectory;
    trajectory.joint_names = jointNames;
    trajectory.points.resize(1);

    trajectory.points[0].positions = positions;
    trajectory.points[0].time_from_start = ros::Duration(duration);

    client->sendGoal(goal);
    client->waitForResult(ros::Duration(duration)); // Adjust the timeout as needed
    return;
}

void move_to_position_BM(ControlClientPtr& client, const std::vector<std::string>& jointNames, std::vector<double> duration, double gesture_duration,
                        const std::string& positionName, std::vector<std::vector<double>> positions, std::vector<std::vector<double>> velocities, std::vector<std::vector<double>> accelerations){
    
    control_msgs::FollowJointTrajectoryGoal goal;
    trajectory_msgs::JointTrajectory& trajectory = goal.trajectory;
    trajectory.joint_names = jointNames;
    trajectory.points.resize(positions.size());


    for(int i = 0; i < positions.size(); i++){
        trajectory.points[i].positions = positions[i];
        trajectory.points[i].velocities = velocities[i];
        trajectory.points[i].accelerations = accelerations[i];
        trajectory.points[i].time_from_start = ros::Duration(duration[i]);
    }

    client->sendGoal(goal);
    client->waitForResult(ros::Duration(gesture_duration)); // Adjust the timeout as needed
    return;
}

void move_to_position_BM_new(ControlClientPtr& armClient, ControlClientPtr& headClient, const std::vector<std::string>& armJointNames, const std::vector<std::string>& headJointNames, std::vector<double> duration, 
                        const std::string& positionName, std::vector<std::vector<double>> armPositions, std::vector<std::vector<double>> headPositions, 
                        std::vector<std::vector<double>> armVelocities, std::vector<std::vector<double>> headVelocities, 
                        std::vector<std::vector<double>> armAccelerations, std::vector<std::vector<double>> headAccelerations){
    
    control_msgs::FollowJointTrajectoryGoal armGoal;
    trajectory_msgs::JointTrajectory& armTrajectory = armGoal.trajectory;
    armTrajectory.joint_names = armJointNames;
    armTrajectory.points.resize(armPositions.size());


    for(int i = 0; i < armPositions.size(); i++){
        armTrajectory.points[i].positions = armPositions[i];
        armTrajectory.points[i].velocities = armVelocities[i];
        armTrajectory.points[i].accelerations = armAccelerations[i];
        armTrajectory.points[i].time_from_start = ros::Duration(duration[i]);
    }

    control_msgs::FollowJointTrajectoryGoal headGoal;
    trajectory_msgs::JointTrajectory& headTrajectory = headGoal.trajectory;
    headTrajectory.joint_names = headJointNames;
    headTrajectory.points.resize(headPositions.size());


    for(int i = 0; i < headPositions.size(); i++){
        headTrajectory.points[i].positions = headPositions[i];
        headTrajectory.points[i].velocities = headVelocities[i];
        headTrajectory.points[i].accelerations = headAccelerations[i];
        headTrajectory.points[i].time_from_start = ros::Duration(duration[i]);
    }

    armClient->sendGoal(armGoal);
    headClient->sendGoal(headGoal);
    armClient->waitForResult(ros::Duration(10.0)); // Adjust the timeout as needed
}

// generate duration by taking the velocity max, min and home position (t = (max - min) / velocity)
std::vector<std::vector<double>> calculateDuration(std::vector<double> homePosition, std::vector<double> maxPosition, std::vector<double> minPosition, std::vector<std::vector<double>> velocity){
    
    // Initialize the duration vector similar to the velocity vector
    std::vector<std::vector<double>> duration(velocity.size(), std::vector<double>(velocity[0].size(), 0.0));
    
    // Calculate the duration for each joint check if the velocity is 0 or not
    for (int i = 0; i < homePosition.size(); ++i){
        // Calculate the duration for the first part of the trajectory
        duration[i][0] = std::fabs(minPosition[i] - homePosition[i]) / velocity[i][0];
        
        // Calculate the duration for the second part of the trajectory
        duration[i][1] = std::fabs(maxPosition[i] - minPosition[i]) / velocity[i][1];
        
        // Calculate the duration for the third part of the trajectory
        duration[i][2] = std::fabs(homePosition[i] - maxPosition[i]) / velocity[i][2];   
    }

    return duration;
}

void headPointing(std::string headTopic, double headPitch, double headYaw, double gestureDuration, int interpolation, bool debug){
    ControlClientPtr headClient = create_client(headTopic);
    std::vector<std::string> headJointNames = {"HeadPitch", "HeadYaw"};
    int numberOfJoints = headJointNames.size();
    
    // positions for each joint
    // std::vector<double> homePosition = {1.7410, -0.09664, 0.09664, 1.6981, -0.05679};
    std::vector<double> headPosition = {headPitch, headYaw};
    // -0.353, -0.930, -1.347, -2.427, -0.057

    std::vector<std::vector<double>> positions_t_head;
    std::vector<std::vector<double>> velocities_t_head;
    std::vector<std::vector<double>> accelerations_t_head;
    std::vector<double> duration_t_head;

    if(interpolation == BIOLOGICAL_MOTION){
        // ros::Duration(5.0).sleep();
        compute_trajectory(headHomePosition, headPosition, numberOfJoints, 1.0, positions_t_head, velocities_t_head, accelerations_t_head, duration_t_head);
        // Move the joint
        move_to_position_BM(headClient, headJointNames, duration_t_head, 1.0, "point", positions_t_head, velocities_t_head, accelerations_t_head);
    }
    else if(interpolation == LINEAR_INTERPOLATION){
        move_to_position(headClient, headJointNames, 1.0, "point", headPosition);
    }
}

void headNodding(std::string headTopic, int nod_angle, int gesture_duration, int interpolation, bool debug) {
    ControlClientPtr headClient = create_client(headTopic);
    std::vector<std::string> jointNames = {"HeadPitch", "HeadYaw"};
    std::vector<double> nodPosition;
    int numberOfJoints = jointNames.size();
    
    // Maximum and minimum positions for each joint


    // std::vector<double> maxPosition = {0.1226, 1.04896};
    // std::vector<double> minPosition = {-0.4534, -1.0367};
    std::vector<double> maxPosition = {0.1226, 0};
    std::vector<double> minPosition = {-0.4534, 0};
    std::vector<double> homePosition = {-0.2, 0.00};
    

    double max_angle = 45.0;
    double set_angle = double(nod_angle); 
    double interpolation_factor = max_angle / set_angle;
    printf("Interpolation factor: %f\n", interpolation_factor);

    double duration_value = double(gesture_duration) / 2000;

    std::vector<std::vector<double>> positions_t;
    std::vector<std::vector<double>> velocities_t;
    std::vector<std::vector<double>> accelerations_t;
    std::vector<double> duration_t;

    nodPosition = {-0.2, 0.00};
    nodPosition[0] = (headHomePosition[0] + maxPosition[0]) / interpolation_factor;
    // nodPosition = maxPosition;
    nodPosition[0] = minPosition[0];

    if(interpolation == BIOLOGICAL_MOTION){
        compute_trajectory(minPosition, maxPosition, numberOfJoints, duration_value, positions_t, velocities_t, accelerations_t, duration_t);
        move_to_position_BM(headClient, jointNames, duration_t, gesture_duration, "nod", positions_t, velocities_t, accelerations_t);
    }
    else if(interpolation == LINEAR_INTERPOLATION){
        move_to_position(headClient, jointNames, duration_value, "nod", nodPosition);
    }
}

void head(std::string headTopic, int nod_angle, int gesture_duration, bool debug) {
    ControlClientPtr headClient = create_client(headTopic);
    std::vector<std::string> jointNames = {"HeadPitch", "HeadYaw"};
    std::vector<double> position(2, 0.0);
    
    // Maximum and minimum positions for each joint
    // std::vector<double> maxPosition = {0.4451, 2.0857};
    // std::vector<double> minPosition = {-0.7068, -2.0857};
    std::vector<double> homePosition = {-0.2, 0.012271};

    std::vector<double> maxPosition = {0.1226, 1.04896};
    std::vector<double> minPosition = {-0.4534, -1.0367};
    // std::vector<double> homePosition = {-0.2, 0.012271};
    
    std::vector<std::vector<double>> velocities = {{1.5, 1.5, 1.5},{1.2, 1.2, 1.2}};
    std::vector<std::vector<double>> duration = calculateDuration(homePosition, maxPosition, minPosition, velocities);
    
    ROS_INFO_STREAM("----------[START HEAD CONTROL TEST]-----------");

    double max_angle = 22.50;
    double set_angle = double(nod_angle); 
    double interpolation_factor = 2 * max_angle / set_angle;
    printf("Interpolation factor: %f\n", interpolation_factor);

    double duration_value = double(gesture_duration) / 2000;

    // For each joint, move to the maximum position, then to the minimum position, then to the mid-range position
    for (int i = 0; i < 1; ++i) {
        // ROS_INFO_STREAM("[START] " << jointNames[i] << " test.");

        ROS_INFO_STREAM("Moving to the Minimum position");
        position[i] = minPosition[i] / interpolation_factor;
        move_to_position(headClient, jointNames, duration_value, "min", position);

        ROS_INFO_STREAM("Moving to the Maximum position");
        position[i] = maxPosition[i] / interpolation_factor;
        move_to_position(headClient, jointNames, duration_value, "max", position);

        // ROS_INFO_STREAM("Moving to the required position");
        // position[i] = (maxPosition[i] + minPosition[i]) / interpolation_factor;
        // move_to_position(headClient, jointNames, duration_value, "mid", position);

        // ROS_INFO_STREAM("[END] " << jointNames[i] << " test.");
    }

    // ROS_INFO_STREAM("[PUT DOWN HEAD] Moving to the Home position");
    double homeDuration = 1.0;
    move_to_position(headClient, jointNames, duration_value, "home", homePosition);

    // End of test 
    // ROS_INFO_STREAM("----------[END HEAD CONTROL TEST]-----------");
}


void rArmHeadPointing(std::string rightArmTopic, std::string headTopic, double shoulderPitch, double shoulderRoll, double elbowYaw, double elbowRoll, double wristYaw, double headPitch, double headYaw, double gestureDuration, int interpolation, bool debug){
    ControlClientPtr rightArmClient = create_client(rightArmTopic);
    ControlClientPtr headClient = create_client(headTopic);
    std::vector<std::string> jointNames = {"RShoulderPitch", "RShoulderRoll",  "RElbowRoll", "RElbowYaw", "RWristYaw", "HeadPitch", "HeadYaw"};
    std::vector<std::string> headJointNames = {"HeadPitch", "HeadYaw"};
    int numberOfJoints = jointNames.size();
    
    // positions for each joint
    // std::vector<double> homePosition = {1.7410, -0.09664, 0.09664, 1.6981, -0.05679};
    std::vector<double> pointPosition = {shoulderPitch, shoulderRoll, elbowRoll, elbowYaw, wristYaw};
    std::vector<double> headPosition = {headPitch, headYaw};
    // -0.353, -0.930, -1.347, -2.427, -0.057

    std::vector<std::vector<double>> positions_t;
    std::vector<std::vector<double>> velocities_t;
    std::vector<std::vector<double>> accelerations_t;
    std::vector<double> duration_t;

    std::vector<std::vector<double>> positions_t_head;
    std::vector<std::vector<double>> velocities_t_head;
    std::vector<std::vector<double>> accelerations_t_head;
    std::vector<double> duration_t_head;

    if(interpolation == BIOLOGICAL_MOTION){
        // ros::Duration(5.0).sleep();
        compute_trajectory(rArmHomePosition, pointPosition, numberOfJoints, gestureDuration, positions_t, velocities_t, accelerations_t, duration_t);
        compute_trajectory(headHomePosition, headPosition, 2, gestureDuration, positions_t_head, velocities_t_head, accelerations_t_head, duration_t_head);
        for(int i = 0; i < positions_t_head.size(); i++){
            for(int j = 0; j < positions_t_head[i].size(); j++){
                printf("%f, ", positions_t_head[i][j]);
            }
            printf("\n");
        }
        // Move the joint
        // move_to_position_BM(headClient, headJointNames, duration_t_head, gestureDuration, "point", positions_t_head, velocities_t_head, accelerations_t_head);
        // ros::Duration(2.0).sleep();
        move_to_position_BM(rightArmClient, jointNames, duration_t, gestureDuration, "point", positions_t, velocities_t, accelerations_t);
    }
    else if(interpolation == LINEAR_INTERPOLATION){
        move_to_position(headClient, headJointNames, gestureDuration, "point", headPosition);
        move_to_position(rightArmClient, jointNames, gestureDuration, "point", pointPosition);
    }
}

void rArmPointingNew(std::string rightArmTopic, double shoulderPitch, double shoulderRoll, double elbowYaw, double elbowRoll, double wristYaw, double headPitch, double headYaw, double gestureDuration, int interpolation, bool debug){
    ControlClientPtr rightArmClient = create_client(rightArmTopic);
    ControlClientPtr headClient = create_client("/pepper_dcm/Head_controller/follow_joint_trajectory");
    std::vector<std::string> jointNames = {"RShoulderPitch", "RShoulderRoll",  "RElbowRoll", "RElbowYaw", "RWristYaw", "HeadPitch", "HeadYaw"};
    std::vector<std::string> headJointNames = {"HeadPitch", "HeadYaw"};
    int numberOfJoints = jointNames.size();
    
    // positions for each joint
    // std::vector<double> homePosition = {1.7410, -0.09664, 0.09664, 1.6981, -0.05679};
    std::vector<double> pointPosition = {shoulderPitch, shoulderRoll, elbowRoll, elbowYaw, wristYaw};
    std::vector<double> headPosition = {headPitch, headYaw};
    // -0.353, -0.930, -1.347, -2.427, -0.057

    std::vector<std::vector<double>> positions_t;
    std::vector<std::vector<double>> velocities_t;
    std::vector<std::vector<double>> accelerations_t;
    std::vector<double> duration_t;

    std::vector<std::vector<double>> positions_t_head;
    std::vector<std::vector<double>> velocities_t_head;
    std::vector<std::vector<double>> accelerations_t_head;
    std::vector<double> duration_t_head;

    if(interpolation == BIOLOGICAL_MOTION){
        // ros::Duration(5.0).sleep();
        compute_trajectory(rArmHomePosition, pointPosition, numberOfJoints, gestureDuration, positions_t, velocities_t, accelerations_t, duration_t);
        compute_trajectory(headHomePosition, headPosition, 2, gestureDuration, positions_t_head, velocities_t_head, accelerations_t_head, duration_t_head);
        // Move the joint
        move_to_position_BM_new(rightArmClient, headClient, jointNames, headJointNames, duration_t, 
                            "point", positions_t, velocities_t, accelerations_t, 
                            positions_t_head, velocities_t_head, accelerations_t_head);
    }
    else if(interpolation == LINEAR_INTERPOLATION){
        move_to_position(rightArmClient, jointNames, gestureDuration, "point", pointPosition);
    }
}

void rArmPointing(std::string rightArmTopic, double shoulderPitch, double shoulderRoll, double elbowYaw, double elbowRoll, double wristYaw, double gestureDuration, int interpolation, bool debug){
    ControlClientPtr rightArmClient = create_client(rightArmTopic);
    std::vector<std::string> jointNames = {"RShoulderPitch", "RShoulderRoll",  "RElbowRoll", "RElbowYaw", "RWristYaw"};
    std::vector<double> position(5, 0.0);
    int numberOfJoints = jointNames.size();
    
    // positions for each joint
    // std::vector<double> homePosition = {1.7410, -0.09664, 0.09664, 1.6981, -0.05679};
    std::vector<double> pointPosition = {shoulderPitch, shoulderRoll, elbowRoll, elbowYaw, wristYaw};
    // -0.353, -0.930, -1.347, -2.427, -0.057

    std::vector<std::vector<double>> positions_t;
    std::vector<std::vector<double>> velocities_t;
    std::vector<std::vector<double>> accelerations_t;
    std::vector<double> duration_t;

    if(interpolation == BIOLOGICAL_MOTION){
        // ros::Duration(5.0).sleep();
        compute_trajectory(rArmHomePosition, pointPosition, numberOfJoints, gestureDuration, positions_t, velocities_t, accelerations_t, duration_t);
        // Move the joint
        move_to_position_BM(rightArmClient, jointNames, duration_t, gestureDuration, "point", positions_t, velocities_t, accelerations_t);
    }
    else if(interpolation == LINEAR_INTERPOLATION){
        move_to_position(rightArmClient, jointNames, gestureDuration, "point", pointPosition);
    }
}

// Return the right arm to the home position from a particular point position defined by the joint angles
void rArmPointingHome(std::string rightArmTopic, double shoulderPitch, double shoulderRoll, double elbowYaw, double elbowRoll, double wristYaw, double gestureDuration, int interpolation, bool debug){
    ControlClientPtr rightArmClient = create_client(rightArmTopic);
    std::vector<std::string> jointNames = {"RShoulderPitch", "RShoulderRoll",  "RElbowRoll", "RElbowYaw", "RWristYaw"};
    std::vector<double> position(5, 0.0);
    int numberOfJoints = jointNames.size();
    
    // positions for each joint
    // std::vector<double> homePosition = {1.7410, -0.09664, 0.09664, 1.6981, -0.05679};
    std::vector<double> pointPosition = {shoulderPitch, shoulderRoll, elbowRoll, elbowYaw, wristYaw};
    // -0.353, -0.930, -1.347, -2.427, -0.057

    std::vector<std::vector<double>> positions_t;
    std::vector<std::vector<double>> velocities_t;
    std::vector<std::vector<double>> accelerations_t;
    std::vector<double> duration_t;

    if(interpolation == BIOLOGICAL_MOTION){
        compute_trajectory(pointPosition, rArmHomePosition, numberOfJoints, gestureDuration, positions_t, velocities_t, accelerations_t, duration_t);
        // Move the joint
        move_to_position_BM(rightArmClient, jointNames, duration_t, gestureDuration, "home", positions_t, velocities_t, accelerations_t); 
    }
    else if(interpolation == LINEAR_INTERPOLATION){
        move_to_position(rightArmClient, jointNames, gestureDuration, "home", rArmHomePosition);
    }
}

void rHand(std::string rightHandTopic, std::string state){
    ControlClientPtr rightHandClient = create_client(rightHandTopic);
    std::vector<std::string> jointNames = {"RHand"};
    
    // Maximum and minimum positions for each joint
    std::vector<double> maxPosition = {1.0};
    std::vector<double> minPosition = {0.0};
    std::vector<double> homePosition = {0.66608};
    double velocity = 2.0;

    if(state == "open"){
        move_to_position(rightHandClient, jointNames, 0.5, "open", maxPosition);
    }
    else if(state == "close"){
        move_to_position(rightHandClient, jointNames, 0.5, "close", minPosition);
    }
    else if(state == "home"){
        move_to_position(rightHandClient, jointNames, 0.5, "home", homePosition);
    }
}

void lHand(std::string leftHandTopic, std::string state){
    ControlClientPtr leftHandClient = create_client(leftHandTopic);
    std::vector<std::string> jointNames = {"LHand"};
    std::vector<double> position(1, 0.0);
    
    // Maximum and minimum positions for each joint
    std::vector<double> maxPosition = {1.0};
    std::vector<double> minPosition = {0.0};
    std::vector<double> homePosition = {0.6695};
    double velocity = 2.0;

    if(state == "open"){
        move_to_position(leftHandClient, jointNames, 0.5, "open", maxPosition);
    }
    else if(state == "close"){
        move_to_position(leftHandClient, jointNames, 0.5, "close", minPosition);
    }
    else if(state == "home"){
        move_to_position(leftHandClient, jointNames, 0.5, "home", homePosition);
    }
}

void lArmPointingNew(std::string leftArmTopic, double shoulderPitch, double shoulderRoll, double elbowYaw, double elbowRoll, double wristYaw, double headPitch, double headYaw, double gestureDuration, int interpolation, bool debug){
    ControlClientPtr leftArmClient = create_client(leftArmTopic);
    std::vector<std::string> jointNames = {"LShoulderPitch", "LShoulderRoll", "LElbowRoll", "LElbowYaw", "LWristYaw", "HeadPitch", "HeadYaw"};
    int numberOfJoints = jointNames.size();

    // positions for each joint
    std::vector<double> homePosition = {1.7410, -0.09664, 0.09664, 1.6981, -0.05679};
    std::vector<double> pointPosition = {shoulderPitch, shoulderRoll, elbowRoll, elbowYaw, wristYaw, headPitch, headYaw};
    printf("JOint angles: %.3f, %.3f, %.3f, %.3f, %.3f\n", shoulderPitch, shoulderRoll, elbowRoll, elbowYaw, wristYaw);
 
    std::vector<std::vector<double>> positions_t;
    std::vector<std::vector<double>> velocities_t;
    std::vector<std::vector<double>> accelerations_t;
    std::vector<double> duration_t;

    if(interpolation == BIOLOGICAL_MOTION){
        compute_trajectory(lArmHeadHomePosition, pointPosition, numberOfJoints, gestureDuration, positions_t, velocities_t, accelerations_t, duration_t);
        // Move the joint
        // move_to_position_BM(leftArmClient, jointNames, duration_t, "side", positions_t, velocities_t, accelerations_t);
    }
    else if(interpolation == LINEAR_INTERPOLATION){
        move_to_position(leftArmClient, jointNames, gestureDuration, "side", pointPosition);
    }
}

void lArmHeadPointing(std::string leftArmTopic, std::string headTopic, double shoulderPitch, double shoulderRoll, double elbowYaw, double elbowRoll, double wristYaw, double headPitch, double headYaw, double gestureDuration, int interpolation, bool debug){
    ControlClientPtr leftArmClient = create_client(leftArmTopic);
    ControlClientPtr headClient = create_client(headTopic);
    std::vector<std::string> jointNames = {"LShoulderPitch", "LShoulderRoll", "LElbowRoll", "LElbowYaw", "LWristYaw"};
    std::vector<std::string> headJointNames = {"HeadPitch", "HeadYaw"};
    int numberOfJoints = jointNames.size();

    // positions for each joint
    std::vector<double> homePosition = {1.7410, -0.09664, 0.09664, 1.6981, -0.05679};
    std::vector<double> pointPosition = {shoulderPitch, shoulderRoll, elbowRoll, elbowYaw, wristYaw};
    std::vector<double> headPosition = {headPitch, headYaw};
    printf("JOint angles: %.3f, %.3f, %.3f, %.3f, %.3f\n", shoulderPitch, shoulderRoll, elbowRoll, elbowYaw, wristYaw);
 
    std::vector<std::vector<double>> positions_t;
    std::vector<std::vector<double>> velocities_t;
    std::vector<std::vector<double>> accelerations_t;
    std::vector<double> duration_t;

    std::vector<std::vector<double>> positions_t_head;
    std::vector<std::vector<double>> velocities_t_head;
    std::vector<std::vector<double>> accelerations_t_head;
    std::vector<double> duration_t_head;

    if(interpolation == BIOLOGICAL_MOTION){
        compute_trajectory(lArmHomePosition, pointPosition, numberOfJoints, gestureDuration, positions_t, velocities_t, accelerations_t, duration_t);
        compute_trajectory(headHomePosition, headPosition, 2, gestureDuration, positions_t_head, velocities_t_head, accelerations_t_head, duration_t_head);
        // Move the joint
        move_to_position_BM(headClient, headJointNames, duration_t_head, gestureDuration, "point", positions_t_head, velocities_t_head, accelerations_t_head);
        move_to_position_BM(leftArmClient, jointNames, duration_t, gestureDuration, "point", positions_t, velocities_t, accelerations_t);
    }
    else if(interpolation == LINEAR_INTERPOLATION){
        move_to_position(headClient, headJointNames, gestureDuration, "point", headPosition);
        move_to_position(leftArmClient, jointNames, gestureDuration, "point", pointPosition);
    }
}

void lArmPointing(std::string leftArmTopic, double shoulderPitch, double shoulderRoll, double elbowYaw, double elbowRoll, double wristYaw, double gestureDuration, int interpolation, bool debug){
    ControlClientPtr leftArmClient = create_client(leftArmTopic);
    std::vector<std::string> jointNames = {"LShoulderPitch", "LShoulderRoll", "LElbowRoll", "LElbowYaw", "LWristYaw"};
    int numberOfJoints = jointNames.size();

    // positions for each joint
    std::vector<double> homePosition = {1.7410, -0.09664, 0.09664, 1.6981, -0.05679};
    std::vector<double> pointPosition = {shoulderPitch, shoulderRoll, elbowRoll, elbowYaw, wristYaw};
    printf("JOint angles: %.3f, %.3f, %.3f, %.3f, %.3f\n", shoulderPitch, shoulderRoll, elbowRoll, elbowYaw, wristYaw);
 
    std::vector<std::vector<double>> positions_t;
    std::vector<std::vector<double>> velocities_t;
    std::vector<std::vector<double>> accelerations_t;
    std::vector<double> duration_t;

    if(interpolation == BIOLOGICAL_MOTION){
        compute_trajectory(lArmHomePosition, pointPosition, numberOfJoints, gestureDuration, positions_t, velocities_t, accelerations_t, duration_t);
        // Move the joint
        move_to_position_BM(leftArmClient, jointNames, duration_t, gestureDuration, "side", positions_t, velocities_t, accelerations_t);
    }
    else if(interpolation == LINEAR_INTERPOLATION){
        move_to_position(leftArmClient, jointNames, gestureDuration, "side", pointPosition);
    }
}

// Return the left arm to the home position from a particular point position defined by the joint angles
void lArmPointingHome(std::string leftArmTopic, double shoulderPitch, double shoulderRoll, double elbowYaw, double elbowRoll, double wristYaw, double gestureDuration, int interpolation, bool debug){
    ControlClientPtr leftArmClient = create_client(leftArmTopic);
    std::vector<std::string> jointNames = {"LShoulderPitch", "LShoulderRoll", "LElbowRoll", "LElbowYaw", "LWristYaw"};
    int numberOfJoints = jointNames.size();

    // positions for each joint
    // std::vector<double> homePosition = {1.7410, -0.09664, 0.09664, 1.6981, -0.05679};
    std::vector<double> pointPosition = {shoulderPitch, shoulderRoll, elbowRoll, elbowYaw, wristYaw};
    // printf("JOint angles: %.3f, %.3f, %.3f, %.3f, %.3f\n", shoulderPitch, shoulderRoll, elbowRoll, elbowYaw, wristYaw);
 
    std::vector<std::vector<double>> positions_t;
    std::vector<std::vector<double>> velocities_t;
    std::vector<std::vector<double>> accelerations_t;
    std::vector<double> duration_t;

    if(interpolation == BIOLOGICAL_MOTION){
        compute_trajectory(pointPosition, lArmHomePosition, numberOfJoints, gestureDuration, positions_t, velocities_t, accelerations_t, duration_t);
        // Move the joint
        move_to_position_BM(leftArmClient, jointNames, duration_t, gestureDuration, "side", positions_t, velocities_t, accelerations_t);
    }
    else if(interpolation == LINEAR_INTERPOLATION){
        move_to_position(leftArmClient, jointNames, gestureDuration, "side", lArmHomePosition);
        // moveToPosition(rightArmClient, jointNames, homeDuration, "side", rndPosition);
    }
}

void legBowing(std::string legTopic, int bow_angle, int gesture_duration, int interpolation, bool debug){
    ControlClientPtr leg_client = create_client(legTopic);
    std::vector<std::string> jointNames = {"HipPitch", "HipRoll", "KneePitch"};
    std::vector<double> bowPosition;
    int numberOfJoints = jointNames.size();
    
    // Maximum and minimum positions for each joint
    std::vector<double> maxPosition = {1.0385,   0.5149,   0.5149}; // 45 degrees
    std::vector<double> minPosition = {-1.0385, -0.5149 , -0.5149}; // -45 degrees
    std::vector<double> homePosition = {-0.0107, -0.00766, 0.03221};    // 0 degrees

    // ROS_INFO_STREAM("----------[START LEG CONTROL TEST]-----------");

    double max_angle = 45.0;
    double set_angle = double(bow_angle); 
    double interpolation_factor = max_angle / set_angle;
    // printf("Interpolation factor: %f\n", interpolation_factor);

    double duration_value = double(gesture_duration) / 1000;

    std::vector<std::vector<double>> positions_t;
    std::vector<std::vector<double>> velocities_t;
    std::vector<std::vector<double>> accelerations_t;
    std::vector<double> duration_t;

    bowPosition = {-0.0107, -0.00766, 0.03221};
    bowPosition[0] = (homePosition[0] + minPosition[0]) / interpolation_factor;

    if(interpolation == BIOLOGICAL_MOTION){
        compute_trajectory(legHomePosition, bowPosition, numberOfJoints, duration_value, positions_t, velocities_t, accelerations_t, duration_t);
        // Move the joint
        move_to_position_BM(leg_client, jointNames, duration_t, gesture_duration, "bow", positions_t, velocities_t, accelerations_t);
        
    }
    else if(interpolation == LINEAR_INTERPOLATION){
        move_to_position(leg_client, jointNames, duration_value, "bow", bowPosition);
    }
}

void leg(std::string legTopic, int bow_angle, int gesture_duration, bool debug){
    ControlClientPtr leg_client = create_client(legTopic);
    std::vector<std::string> jointNames = {"HipPitch", "HipRoll", "KneePitch"};
    std::vector<double> position(3, 0.0);
    
    
    // Maximum and minimum positions for each joint
    std::vector<double> maxPosition = {1.0385,   0.5149,   0.5149}; // 45 degrees
    std::vector<double> minPosition = {-1.0385, -0.5149 , -0.5149}; // -45 degrees
    std::vector<double> homePosition = {-0.0107, -0.00766, 0.03221};    // 0 degrees

    std::vector<std::vector<double>> velocities = {{0.5, 0.5, 0.5},{0.5, 0.5, 0.5},{0.5, 0.5, 0.5}};
    std::vector<std::vector<double>> duration = calculateDuration(homePosition, maxPosition, minPosition, velocities);


    // ROS_INFO_STREAM("----------[START LEG CONTROL TEST]-----------");

    double max_angle = 45.0;
    double set_angle = double(bow_angle); 
    double interpolation_factor = max_angle / set_angle;
    printf("Interpolation factor: %f\n", interpolation_factor);

    double duration_value = double(gesture_duration) / 1000;

    // For each joint, move to the maximum position, then to the minimum position, then to the mid-range position
    for (int i = 0; i < 1; ++i) {
        // ROS_INFO_STREAM("[START] " << jointNames[i] << " test.");

        // ROS_INFO_STREAM("Moving to the Minimum position");
        // position[i] = minPosition[i];
        // move_to_position(leg_client, jointNames, duration[i][0], "min", position);

        // ROS_INFO_STREAM("Moving to the Maximum position");
        // position[i] = maxPosition[i];
        // move_to_position(leg_client, jointNames, duration[i][1], "max", position);

        // ROS_INFO_STREAM("Moving to the Mid-range position");
        // position[i] = (maxPosition[i] + minPosition[i]) / 2.0;
        // move_to_position(leg_client, jointNames, duration[i][2], "mid", position);

        ROS_INFO_STREAM("Moving to the required position");
        position[i] = (homePosition[i] + minPosition[i]) / interpolation_factor;
        printf("Duration: %f\n", duration_value);
        move_to_position(leg_client, jointNames, duration_value, "mid", position);

        // ROS_INFO_STREAM("[END] " << jointNames[i] << " test.");
    }
    printf("Position: {%f, %f, %f}\n", position[0], position[1], position[2]);

    // calc_velocity(homePosition, maxPosition, minPosition, duration);

    // ROS_INFO_STREAM("[PUT DOWN LEG] Moving to the Home position");
    double homeDuration = 2.0;
    move_to_position(leg_client, jointNames, homeDuration, "home", homePosition);

    // End of test
    // ROS_INFO_STREAM("----------[END LEG CONTROL TEST]-----------");
}

/* Utility Functions */
void prompt_and_exit(int status){
    printf("Press any key to exit ... \n");
    getchar();
    exit(status);
}

void prompt_and_continue(){
    printf("Press X to quit or Press any key to continue...\n");
    char gotChar = getchar();
    if ((gotChar == 'X') || (gotChar == 'x')){
        printf("Exiting ...\n");
       exit(0);
    }
}