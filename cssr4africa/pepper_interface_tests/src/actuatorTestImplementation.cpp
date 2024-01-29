/* actuatorTestImplementation.cpp
*
* Author: Yohannes Tadesse Haile and Mihirteab Taye Hordofa 
* Date: January 11, 2024
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


# include "pepper_interface_tests/actuatorTest.h"

ControlClientPtr createClient(const std::string& TopicName) {
    ControlClientPtr actionClient(new ControlClient(TopicName, true));
    int maxIterations = 5;

    for (int iterations = 0; iterations < maxIterations; ++iterations) {
        if (actionClient->waitForServer(ros::Duration(5.0))) {
            return actionClient;
        }
        ROS_DEBUG("Waiting for the %s controller to come up", TopicName.c_str());
    }

    throw std::runtime_error("Error creating action client for " + TopicName + " controller: Server not available");
}

/* Extract topic names for the respective simulator or physical robot */
std::string extractTopic(std::string key){
    bool debug = false;   // used to turn debug message on
    
    std::string configFileName = "actuatorTestConfiguration.ini";  // configuration filename
    std::string configPath;                                        // configuration path
    std::string configPathFile;                                    // configuration path and filename
    
    std::string platformKey = "platform";                          // platform key 
    std::string robotTopicKey = "robottopics";                     // robot topic key
    std::string simulatorTopicKey = "simulatortopics";             // simulator topic key

    std::string platformValue;                                     // platform value
    std::string robotTopicValue;                                   // robot topic value
    std::string simulatorTopicValue;                               // simulator topic value
    std::string mode;                                              // mode value
    
    std::string topicFileName;                                         // topic filename
    std::string topicPath;                                         // topic filename path
    std::string topicPathFile;                                     // topic with path and file 

    std::string topic_value = "";                                  // topic value

    // Construct the full path of the configuration file
    #ifdef ROS
        configPath = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        configPath = "..";
    #endif

    // set configuration path
    configPath += "/config/";
    configPathFile = configPath;
    configPathFile += configFileName;

    if (debug) printf("Config file is %s\n", configPathFile.c_str());

    // Open configuration file
    std::ifstream configFile(configPathFile.c_str());
    if (!configFile.is_open()){
        printf("Unable to open the config file %s\n", configPathFile.c_str());
        promptAndExit(1);
    }

    std::string configLineRead;  // variable to read the line in the file
    // Get key-value pairs from the configuration file
    while(std::getline(configFile, configLineRead)){
        std::istringstream iss(configLineRead);
        std::string paramKey, paramValue;
        iss >> paramKey;
        trim(paramKey);
        std::getline(iss, paramValue);
        iss >> paramValue;
        trim(paramValue);

        // To lower case
        transform(paramKey.begin(), paramKey.end(), paramKey.begin(), ::tolower);
        transform(paramValue.begin(), paramValue.end(), paramValue.begin(), ::tolower);

        if (paramKey == platformKey){ platformValue = paramValue;}
        
        else if (paramKey == robotTopicKey){ robotTopicValue = paramValue;}

        else if (paramKey == simulatorTopicKey){ simulatorTopicValue = paramValue;}

    }
    configFile.close();

    // set the topic file based on the config extracted above
    if (platformValue == "simulator") { topicFileName = "simulatorTopics.dat"; }
    else if (platformValue == "robot") { topicFileName = "pepperTopics.dat"; }
    
    if (debug) printf("Topic file: %s\n", topicFileName.c_str());

    // Construct the full path of the topic file
    #ifdef ROS
        topicPath = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        topicPath = "..";
    #endif

    // set topic path    
    topicPath += "/data/";
    topicPathFile = topicPath;
    topicPathFile += topicFileName;

    if (debug) printf("Topic file is %s\n", topicPathFile.c_str());

    // Open topic file
    std::ifstream topicFile(topicPathFile.c_str());
    if (!topicFile.is_open()){
        printf("Unable to open the topic file %s\n", topicPathFile.c_str());
        promptAndExit(1);
    }

    std::string topicLineRead;   // variable to read the line in the file
    // Get key-value pairs from the topic file
    while(std::getline(topicFile, topicLineRead)){
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
    topicFile.close();

    // verify the topic_value is not empty
    if (topic_value == ""){
        printf("Unable to find a valid topic.\n");
        promptAndExit(1);
    }
    return topic_value;
}

// Extract the mode to run the tests
std::string extractMode(){
    bool debug = false;   // used to turn debug message on
    
    std::string configFileName = "actuatorTestConfiguration.ini";  // configuration filename
    std::string configPath;                                  // configuration path
    std::string configPathFile;                         // configuration path and filename
    
    std::string modeKey = "mode";                             // mode key 

    std::string modeValue;                                    // mode value
    
    // Construct the full path of the configuration file
    #ifdef ROS
        configPath = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        configPath = "..";
    #endif

    // set configuration path
    configPath += "/config/";
    configPathFile = configPath;
    configPathFile += configFileName;

    if (debug) printf("Config file is %s\n", configPathFile.c_str());

    // Open configuration file
    std::ifstream configFile(configPathFile.c_str());
    if (!configFile.is_open()){
        printf("Unable to open the config file %s\n", configPathFile.c_str());
        promptAndExit(1);
    }

    std::string configLineRead;  // variable to read the line in the file
    // Get key-value pairs from the configuration file
    while(std::getline(configFile, configLineRead)){
        std::istringstream iss(configLineRead);
        std::string paramKey, paramValue;
        iss >> paramKey;
        trim(paramKey);
        std::getline(iss, paramValue);
        iss >> paramValue;
        trim(paramValue);

        // To lower case
        transform(paramKey.begin(), paramKey.end(), paramKey.begin(), ::tolower);
        transform(paramValue.begin(), paramValue.end(), paramValue.begin(), ::tolower);

        if (paramKey == modeKey){ modeValue = paramValue;}
    }
    configFile.close();

    // verify the modeValue is not empty
    if (modeValue == ""){
        printf("Unable to find a valid mode.\n");
        promptAndExit(1);
    }
    return modeValue;
}

/* Extract the expected tests to run for the respective actuator or sensor tests */
std::vector<std::string> extractTests(std::string test){
    bool debug = false;   // used to turn debug message on
    
    std::string inputFileName;                                  // input filename
    std::string inputPath;                                  // input path
    std::string inputPathFile;                         // input path and filename
    
    std::vector<std::string> testName;
    std::string flag;

    if (test == "actuator"){
        inputFileName = "actuatorTestInput.ini";
    }
    else if (test == "sensor"){
        inputFileName = "sensorTestInput.ini";
    }
    else {
        printf("unable to identify the test.\n");
    }

    // Construct the full path of the input file
    #ifdef ROS
        inputPath = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
        std::cout<<inputPath<<std::endl;
    #else
        inputPath = "..";
    #endif
    
    inputPath += "/config/";
    inputPathFile = inputPath;
    inputPathFile += inputFileName;

    if (debug) printf("Input file is %s\n", inputPathFile.c_str());

    // Open input file
    std::ifstream inputFile(inputPathFile.c_str());
    if (!inputFile.is_open()){
        printf("Unable to open the input file %s\n", inputPathFile.c_str());
        promptAndExit(1);
    }

    std::string inpLineRead;  // variable to read the line in the file
    
    std::string paramKey, paramValue;
    // Get key-value pairs from the input file
    while(std::getline(inputFile, inpLineRead)){
        std::istringstream iss(inpLineRead);
    
        iss >> paramKey;
        trim(paramKey);
        std::getline(iss, paramValue);
        iss >> paramValue;
        
        trim(paramValue); // trim whitespace
        transform(paramValue.begin(), paramValue.end(), paramValue.begin(), ::tolower); // convert to lower case

        if (paramValue == "true"){ testName.push_back(paramKey);}
    }
    inputFile.close();

    return testName;
}

/* Helper Functions */
void promptAndExit(int status){
    printf("Press any key to continue ... \n");
    getchar();
    exit(status);
}

void promptAndContinue(){
    printf("Press any key to proceed ...\n");
    getchar();
}

void moveToPosition(ControlClientPtr& client, const std::vector<std::string>& jointNames, double duration, 
                        const std::string& positionName, std::vector<double> positions){
    
    control_msgs::FollowJointTrajectoryGoal goal;
    trajectory_msgs::JointTrajectory& trajectory = goal.trajectory;
    trajectory.joint_names = jointNames;
    trajectory.points.resize(1);

    trajectory.points[0].positions = positions;
    trajectory.points[0].time_from_start = ros::Duration(duration);

    client->sendGoal(goal);
    client->waitForResult(ros::Duration(10.0)); // Adjust the timeout as needed
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

void head(ros::NodeHandle& nh, const std::string headTopic) {
    ControlClientPtr headClient = createClient(headTopic);
    std::vector<std::string> jointNames = {"HeadPitch", "HeadYaw"};
    std::vector<double> position(2, 0.0);
    
    // Maximum and minimum positions for each joint
    std::vector<double> maxPosition = {0.4451, 2.0857};
    std::vector<double> minPosition = {-0.7068, -2.0857};
    std::vector<double> homePosition = {-0.2, 0.012271};
    
    std::vector<std::vector<double>> velocities = {{1.5, 1.5, 1.5},{1.2, 1.2, 1.2}};
    std::vector<std::vector<double>> duration = calculateDuration(homePosition, maxPosition, minPosition, velocities);
    
    ROS_INFO_STREAM("----------[START HEAD CONTROL TEST]-----------");

    // For each joint, move to the maximum position, then to the minimum position, then to the mid-range position
    for (int i = 0; i < jointNames.size(); ++i) {
        ROS_INFO_STREAM("[START] " << jointNames[i] << " test.");

        ROS_INFO_STREAM("Moving to the Minimum position");
        position[i] = minPosition[i];
        moveToPosition(headClient, jointNames, duration[i][0], "min", position);

        ROS_INFO_STREAM("Moving to the Maximum position");
        position[i] = maxPosition[i];
        moveToPosition(headClient, jointNames, duration[i][1], "max", position);

        ROS_INFO_STREAM("Moving to the Mid-range position");
        position[i] = (maxPosition[i] + minPosition[i]) / 2.0;
        moveToPosition(headClient, jointNames, duration[i][2], "mid", position);

        ROS_INFO_STREAM("[END] " << jointNames[i] << " test.");
    }

    ROS_INFO_STREAM("[PUT DOWN HEAD] Moving to the Home position");
    double homeDuration = 2.0;
    moveToPosition(headClient, jointNames, homeDuration, "home", homePosition);

    // End of test 
    ROS_INFO_STREAM("----------[END HEAD CONTROL TEST]-----------");
}

void rArm(ros::NodeHandle& nh, std::string rightArmTopic){
    ControlClientPtr rightArmClient = createClient(rightArmTopic);
    std::vector<std::string> jointNames = {"RShoulderPitch", "RShoulderRoll",  "RElbowRoll", "RElbowYaw", "RWristYaw"};
    std::vector<double> position(5, 0.0);
    
    // Maximum and minimum positions for each joint
    std::vector<double> maxPosition = {2.0857,  -0.0087,  1.5620,  2.0857,  1.8239};
    std::vector<double> minPosition = {-2.0857, -1.5620 , 0.0087, -2.0857, -1.5620};
    std::vector<double> homePosition = {1.7410, -0.09664, 0.09664, 1.6981, -0.05679};
  
    std::vector<std::vector<double>> velocity = {{1.5, 1.5, 0.1}, {1.2, 0.8, 0.15},{0.1, 0.8, 1.2}, {2.0, 1.5, 0.2}, {1.8, 1.8, 1.8}};
    std::vector<std::vector<double>> duration = calculateDuration(homePosition, maxPosition, minPosition, velocity);

    ROS_INFO_STREAM("----------[START RIGHT ARM CONTROL TEST]-----------");

    // For each joint, move to the maximum position, then to the minimum position, then to the mid-range position
    for (int i = 0; i < jointNames.size(); ++i) {
        ROS_INFO_STREAM("[START] " << jointNames[i] << " test.");

        ROS_INFO_STREAM("Moving to the Minimum position");
        position[i] = minPosition[i];
        moveToPosition(rightArmClient, jointNames, duration[i][0], "min", position);

        ROS_INFO_STREAM("Moving to the Maximum position");
        position[i] = maxPosition[i];
        moveToPosition(rightArmClient, jointNames, duration[i][1], "max", position);

        ROS_INFO_STREAM("Moving to the Mid-range position");
        position[i] = (maxPosition[i] + minPosition[i]) / 2.0;
        moveToPosition(rightArmClient, jointNames, duration[i][2], "mid", position);

        ROS_INFO_STREAM("[END] " << jointNames[i] << " test.");
    }

    ROS_INFO_STREAM("[PUT DOWN RIGHT ARM] Moving to the Home position");
    double homeDuration = 2.0;
    moveToPosition(rightArmClient, jointNames, homeDuration, "home", homePosition);

    // End of test 
    ROS_INFO_STREAM("----------[END RIGHT ARM CONTROL TEST]-----------");
}

void rHand(ros::NodeHandle& nh, std::string rightHandTopic){
    ControlClientPtr rightHandClient = createClient(rightHandTopic);
    std::vector<std::string> jointNames = {"RHand"};
    std::vector<double> position(1, 0.0);
    
    // Maximum and minimum positions for each joint
    std::vector<double> maxPosition = {1.0};
    std::vector<double> minPosition = {0.0};
    std::vector<double> homePosition = {0.66608};
    double velocity = 2.0;

    double duration = std::fabs(maxPosition[0] - minPosition[0]) / velocity;

    ROS_INFO_STREAM("----------[START RIGHT HAND CONTROL TEST]-----------");

    // For each joint, move to the maximum position, then to the minimum position, then to the mid-range position
    for (int i = 0; i < jointNames.size(); ++i) {
        ROS_INFO_STREAM("[START] " << jointNames[i] << " test.");

        ROS_INFO_STREAM("Moving to the Minimum position");
        position[i] = minPosition[i];
        moveToPosition(rightHandClient, jointNames, duration, "min", position);

        ROS_INFO_STREAM("Moving to the Maximum position");
        position[i] = maxPosition[i];
        moveToPosition(rightHandClient, jointNames, duration, "max", position);

        ROS_INFO_STREAM("Moving to the Mid-range position");
        position[i] = (maxPosition[i] + minPosition[i]) / 2.0;
        moveToPosition(rightHandClient, jointNames, duration, "mid", position);

        ROS_INFO_STREAM("[END] " << jointNames[i] << " test.");
    }

    // calc_velocity(homePosition, maxPosition, minPosition, duration);

    ROS_INFO_STREAM("[PUT DOWN RIGHT HAND] Moving to the Home position");
    moveToPosition(rightHandClient, jointNames, duration, "home", homePosition);

    // End of test 
    ROS_INFO_STREAM("----------[END RIGHT HAND CONTROL TEST]-----------");
}


void lArm(ros::NodeHandle& nh, std::string leftArmTopic){
    ControlClientPtr leftArmClient = createClient(leftArmTopic);
    std::vector<std::string> jointNames = {"LShoulderPitch", "LShoulderRoll", "LElbowRoll", "LElbowYaw", "LWristYaw"};
    std::vector<double> position(5, 0.0);
    
    // Maximum and minimum positions for each joint
    std::vector<double> maxPosition = {2.0857,  0.0087,  -1.5620, -2.0857,  -1.8239};
    std::vector<double> minPosition = {-2.0857, 1.5620 , -0.0087,  2.0857,   1.8239};
    std::vector<double> homePosition = {1.7625, 0.09970, -0.1334, -1.7150,  0.06592};

    std::vector<std::vector<double>> velocities = {{1.5, 1.5, 0.1},{1.2, 0.8, 0.15},{0.1, 0.9, 1.2},{2.1, 1.5, 0.2},{1.8, 1.8, 1.9}};
    std::vector<std::vector<double>> duration = calculateDuration(homePosition, maxPosition, minPosition, velocities);

    ROS_INFO_STREAM("----------[START LEFT ARM CONTROL TEST]-----------");

    // For each joint, move to the maximum position, then to the minimum position, then to the mid-range position
    for (int i = 0; i < jointNames.size(); ++i) {
        ROS_INFO_STREAM("[START] " << jointNames[i] << " test.");

        ROS_INFO_STREAM("Moving to the Minimum position");
        position[i] = minPosition[i];
        moveToPosition(leftArmClient, jointNames, duration[i][0], "min", position);

        ROS_INFO_STREAM("Moving to the Maximum position");
        position[i] = maxPosition[i];
        moveToPosition(leftArmClient, jointNames, duration[i][1], "max", position);

        ROS_INFO_STREAM("Moving to the Mid-range position");
        position[i] = (maxPosition[i] + minPosition[i]) / 2.0;
        moveToPosition(leftArmClient, jointNames, duration[i][2], "mid", position);

        ROS_INFO_STREAM("[END] " << jointNames[i] << " test.");
    }

    // calc_velocity(homePosition, maxPosition, minPosition, duration);

    ROS_INFO_STREAM("[PUT DOWN LEFT ARM] Moving to the Home position");
    double homeDuration = 2.0;
    moveToPosition(leftArmClient, jointNames, homeDuration, "home", homePosition);

    // End of test
    ROS_INFO_STREAM("----------[END LEFT ARM CONTROL TEST]-----------");
}

void lHand(ros::NodeHandle& nh, std::string leftHandTopic){
    ControlClientPtr leftHandClient = createClient(leftHandTopic);
    std::vector<std::string> jointNames = {"LHand"};
    std::vector<double> position(1, 0.0);
    
    // Maximum and minimum positions for each joint
    std::vector<double> maxPosition = {1.0};
    std::vector<double> minPosition = {0.0};
    std::vector<double> homePosition = {0.6695};

    double velocity = 2.0;
    double duration = std::fabs(maxPosition[0] - minPosition[0]) / velocity;

    ROS_INFO_STREAM("----------[START LEFT HAND CONTROL TEST]-----------");

    // For each joint, move to the maximum position, then to the minimum position, then to the mid-range position
    for (int i = 0; i < jointNames.size(); ++i) {
        ROS_INFO_STREAM("[START] " << jointNames[i] << " test.");

        ROS_INFO_STREAM("Moving to the Minimum position");
        position[i] = minPosition[i];
        moveToPosition(leftHandClient, jointNames, duration, "min", position);

        ROS_INFO_STREAM("Moving to the Maximum position");
        position[i] = maxPosition[i];
        moveToPosition(leftHandClient, jointNames, duration, "max", position);

        ROS_INFO_STREAM("Moving to the Mid-range position");
        position[i] = (maxPosition[i] + minPosition[i]) / 2.0;
        moveToPosition(leftHandClient, jointNames, duration, "mid", position);

        ROS_INFO_STREAM("[END] " << jointNames[i] << " test.");
    }

    // calc_velocity(homePosition, maxPosition, minPosition, duration);

    ROS_INFO_STREAM("[PUT DOWN LEFT HAND] Moving to the Home position");
    moveToPosition(leftHandClient, jointNames, duration, "home", homePosition);

    // End of test
    ROS_INFO_STREAM("----------[END LEFT HAND CONTROL TEST]-----------");
}

void leg(ros::NodeHandle& nh, std::string legTopic){
    ControlClientPtr legClient = createClient(legTopic);
    std::vector<std::string> jointNames = {"HipPitch", "HipRoll", "KneePitch"};
    std::vector<double> position(3, 0.0);
    
    
    // Maximum and minimum positions for each joint
    std::vector<double> maxPosition = {1.0385,   0.5149,   0.5149};
    std::vector<double> minPosition = {-1.0385, -0.5149 , -0.5149};
    std::vector<double> homePosition = {-0.0107, -0.00766, 0.03221};

    std::vector<std::vector<double>> velocities = {{0.5, 0.5, 0.5},{0.5, 0.5, 0.5},{0.5, 0.5, 0.5}};
    std::vector<std::vector<double>> duration = calculateDuration(homePosition, maxPosition, minPosition, velocities);


    ROS_INFO_STREAM("----------[START LEG CONTROL TEST]-----------");

    // For each joint, move to the maximum position, then to the minimum position, then to the mid-range position
    for (int i = 0; i < jointNames.size(); ++i) {
        ROS_INFO_STREAM("[START] " << jointNames[i] << " test.");

        ROS_INFO_STREAM("Moving to the Minimum position");
        position[i] = minPosition[i];
        moveToPosition(legClient, jointNames, duration[i][0], "min", position);

        ROS_INFO_STREAM("Moving to the Maximum position");
        position[i] = maxPosition[i];
        moveToPosition(legClient, jointNames, duration[i][1], "max", position);

        ROS_INFO_STREAM("Moving to the Mid-range position");
        position[i] = (maxPosition[i] + minPosition[i]) / 2.0;
        moveToPosition(legClient, jointNames, duration[i][2], "mid", position);

        ROS_INFO_STREAM("[END] " << jointNames[i] << " test.");
    }

    // calc_velocity(homePosition, maxPosition, minPosition, duration);

    ROS_INFO_STREAM("[PUT DOWN LEG] Moving to the Home position");
    double homeDuration = 2.0;
    moveToPosition(legClient, jointNames, homeDuration, "home", homePosition);

    // End of test
    ROS_INFO_STREAM("----------[END LEG CONTROL TEST]-----------");
}

// Function to publish a velocity command to a joint
void publishVelocity(ros::Publisher &pub, geometry_msgs::Twist &msg, ros::Rate &rate, double duration) {
    ros::Time startTime = ros::Time::now();
    ros::Duration waitTime = ros::Duration(duration); 
    ros::Time endTime = startTime + waitTime;
    // Publish the trajectory for 1 seconds
    while(ros::ok() && ros::Time::now() < endTime) {
        pub.publish(msg);
        rate.sleep();
    }
}

void wheels(ros::NodeHandle& nh, std::string wheelTopic){
   
   // Create a publisher to publish geometry_msgs::Twist messages on the /pepper/cmd_vel topic
   ros::Publisher pub = nh.advertise<geometry_msgs::Twist>(wheelTopic, 1000, true);

   // Set the publishing rate to 50 Hz
   ros::Rate rate(50); 

   // Create a Twist message object
   geometry_msgs::Twist msg;
   
   ROS_INFO_STREAM("-------[START WHEEL CONTROL TEST]--------");
   /* [1] THIS SECTION PUBLISHES A LINEAR VELOCITY ON THE CMD VEL TOPIC */
   ROS_INFO_STREAM("[LINEAR VELOCITY START] Publishing linear velocity on the cmd vel started.");
   
   // Initialize the message with 0 linear velocity
   ROS_INFO_STREAM("[ZERO VELOCITY] Publishing 0 velocity value.");
   msg.linear.x = 0.0;

   // Publish 0 velocity
   publishVelocity(pub, msg, rate, 1);

   // Publish a fixed positive linear velocity
   ROS_INFO_STREAM("[POSITIVE VELOCITY] Publishing a fixed positive velocity value");
   msg.linear.x = 0.05;

   // Publish the positive velocity 
   publishVelocity(pub, msg, rate, 4);

   // Reset linear velocity to 0
   ROS_INFO_STREAM("[ZERO VELOCITY] Publishing 0 velocity value.");
   msg.linear.x = 0.0;

   // Publish 0 velocity 
   publishVelocity(pub, msg, rate, 2);

   // Publish a fixed negative linear velocity
   ROS_INFO_STREAM("[NEGATIVE VELOCITY] Publishing a fixed negative velocity value");
   msg.linear.x = -0.05;

   // Publish the negative velocity 
   publishVelocity(pub, msg, rate, 4);

   // Reset linear velocity to 0
   ROS_INFO_STREAM("[ZERO VELOCITY] Publishing 0 velocity value.");
   msg.linear.x = 0.0;

   // Publish 0 velocity 
   publishVelocity(pub, msg, rate, 4);
   
   ROS_INFO_STREAM("[LINEAR VELOCITY END] Publishing linear velocity ended.");
   
   /* [2] THIS SECTION PUBLISHES AN ANGULAR VELOCITY ON THE CMD VEL TOPIC */
   ROS_INFO_STREAM("[ANGULAR VELOCITY START] Publishing angular velocity on the cmd vel started.");
   
   // Initialize the message with 0 angular velocity
   ROS_INFO_STREAM("[ZERO VELOCITY] Publishing 0 velocity value.");
   msg.angular.z = 0.0;

   // Publish 0 velocity 
   publishVelocity(pub, msg, rate, 2);

   // Publish a fixed positive angular velocity
   ROS_INFO_STREAM("[POSITIVE VELOCITY] Publishing a fixed positive velocity value");
   msg.angular.z = 0.3925;

   // Publish the positive velocity 
   publishVelocity(pub, msg, rate, 4);

   // Reset angular velocity to 0
   ROS_INFO_STREAM("[ZERO VELOCITY] Publishing 0 velocity value.");
   msg.angular.z = 0.0;

   // Publish 0 velocity 
   publishVelocity(pub, msg, rate, 1);

   // Publish a fixed negative angular velocity
   ROS_INFO_STREAM("[NEGATIVE VELOCITY] Publishing a fixed negative velocity value");
   msg.angular.z = -0.3925;

   // Publish the negative velocity 
   publishVelocity(pub, msg, rate, 4);

   // Reset angular velocity to 0
   ROS_INFO_STREAM("[ZERO VELOCITY] Publishing 0 velocity value.");
   msg.angular.z = 0.0;

   // Publish 0 velocity 
   publishVelocity(pub, msg, rate, 4);
   
   ROS_INFO_STREAM("[ANGULAR VELOCITY END] Publishing angular velocity ended.");
    
   // Print success message
   ROS_INFO_STREAM("[SUCCESS] Wheel control test completed.");
   ROS_INFO_STREAM("                                       ");
}