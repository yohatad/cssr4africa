# include "pepper_interface_tests/actuatorTest.h"

ControlClientPtr createclient(const std::string& controller_name) {
    ControlClientPtr actionClient(new ControlClient(controller_name, true));
    int maxIterations = 5;

    for (int iterations = 0; iterations < maxIterations; ++iterations) {
        if (actionClient->waitForServer(ros::Duration(5.0))) {
            return actionClient;
        }
        ROS_DEBUG("Waiting for the %s controller to come up", controller_name.c_str());
    }

    throw std::runtime_error("Error creating action client for " + controller_name + " controller: Server not available");
}

/* Extract topic names for the respective simulator or physical robot */
std::string extract_topic(std::string key){
    bool debug = false;   // used to turn debug message on
    
    std::string conf_file = "actuatorTestConfiguration.ini";  // configuration filename
    std::string config_path;                                  // configuration path
    std::string config_path_and_file;                         // configuration path and filename
    
    std::string platformKey = "platform";                     // platform key 
    std::string robotTopicKey = "robottopics";                // robot topic key
    std::string simulatorTopicKey = "simulatortopics";        // simulator topic key

    std::string platformValue;                                // platform value
    std::string robotTopicValue;                              // robot topic value
    std::string simulatorTopicValue;                          // simulator topic value
    std::string mode;                                         // mode value
    
    std::string topic_file;                                   // topic filename
    std::string topic_path;                                   // topic filename path
    std::string topic_path_and_file;                          // topic with path and file 

    std::string topic_value = "";                             // topic value

    // Construct the full path of the configuration file
    #ifdef ROS
        config_path = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        config_path = "..";
    #endif

    // set configuration path
    config_path += "/config/";
    config_path_and_file = config_path;
    config_path_and_file += conf_file;

    if (debug) printf("Config file is %s\n", config_path_and_file.c_str());

    // Open configuration file
    std::ifstream conf_if(config_path_and_file.c_str());
    if (!conf_if.is_open()){
        printf("Unable to open the config file %s\n", config_path_and_file.c_str());
        prompt_and_exit(1);
    }

    std::string configLineRead;  // variable to read the line in the file
    // Get key-value pairs from the configuration file
    while(std::getline(conf_if, configLineRead)){
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
    conf_if.close();

    // set the topic file based on the config extracted above
    if (platformValue == "simulator") { topic_file = "simulatorTopics.dat"; }
    else if (platformValue == "robot") { topic_file = "pepperTopics.dat"; }
    
    if (debug) printf("Topic file: %s\n", topic_file.c_str());

    // Construct the full path of the topic file
    #ifdef ROS
        topic_path = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        topic_path = "..";
    #endif

    // set topic path    
    topic_path += "/data/";
    topic_path_and_file = topic_path;
    topic_path_and_file += topic_file;

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

// Extract the mode to run the tests
std::string extract_mode(){
    bool debug = false;   // used to turn debug message on
    
    std::string conf_file = "actuatorTestConfiguration.ini";  // configuration filename
    std::string config_path;                                  // configuration path
    std::string config_path_and_file;                         // configuration path and filename
    
    std::string modeKey = "mode";                             // mode key 

    std::string modeValue;                                    // mode value
    
    // Construct the full path of the configuration file
    #ifdef ROS
        config_path = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        config_path = "..";
    #endif

    // set configuration path
    config_path += "/config/";
    config_path_and_file = config_path;
    config_path_and_file += conf_file;

    if (debug) printf("Config file is %s\n", config_path_and_file.c_str());

    // Open configuration file
    std::ifstream conf_if(config_path_and_file.c_str());
    if (!conf_if.is_open()){
        printf("Unable to open the config file %s\n", config_path_and_file.c_str());
        prompt_and_exit(1);
    }

    std::string configLineRead;  // variable to read the line in the file
    // Get key-value pairs from the configuration file
    while(std::getline(conf_if, configLineRead)){
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
    conf_if.close();

    // verify the modeValue is not empty
    if (modeValue == ""){
        printf("Unable to find a valid mode.\n");
        prompt_and_exit(1);
    }
    return modeValue;
}

/* Extract the expected tests to run for the respective actuator or sensor tests */
std::vector<std::string> extract_tests(std::string test){
    bool debug = false;   // used to turn debug message on
    
    std::string inp_file;                                  // input filename
    std::string inp_path;                                  // input path
    std::string inp_path_and_file;                         // input path and filename
    
    std::vector<std::string> test_name;
    std::string flag;

    if (test == "actuator"){
        inp_file = "actuatorTestInput.ini";
    }
    else if (test == "sensor"){
        inp_file = "sensorTestInput.ini";
    }
    else {
        printf("unable to identify the test.\n");
    }

    // Construct the full path of the input file
    #ifdef ROS
        inp_path = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
        std::cout<<inp_path<<std::endl;
    #else
        inp_path = "..";
    #endif
    
    inp_path += "/config/";
    inp_path_and_file = inp_path;
    inp_path_and_file += inp_file;

    if (debug) printf("Input file is %s\n", inp_path_and_file.c_str());

    // Open input file
    std::ifstream inp_if(inp_path_and_file.c_str());
    if (!inp_if.is_open()){
        printf("Unable to open the input file %s\n", inp_path_and_file.c_str());
        prompt_and_exit(1);
    }

    std::string inpLineRead;  // variable to read the line in the file
    
    std::string paramKey, paramValue;
    // Get key-value pairs from the input file
    while(std::getline(inp_if, inpLineRead)){
        std::istringstream iss(inpLineRead);
    
        iss >> paramKey;
        trim(paramKey);
        std::getline(iss, paramValue);
        iss >> paramValue;
        
        trim(paramValue); // trim whitespace
        transform(paramValue.begin(), paramValue.end(), paramValue.begin(), ::tolower); // convert to lower case

        if (paramValue == "true"){ test_name.push_back(paramKey);}
    }
    inp_if.close();

    return test_name;
}

/* Helper Functions */
void prompt_and_exit(int status){
    printf("Press any key to continue ... \n");
    getchar();
    exit(status);
}

void prompt_and_continue(){
    printf("Press any key to proceed ...\n");
    getchar();
}

void moveToPosition(ControlClientPtr& client, const std::vector<std::string>& joint_names, double duration, 
                        const std::string& position_name, std::vector<double> positions){
    
    control_msgs::FollowJointTrajectoryGoal goal;
    trajectory_msgs::JointTrajectory& trajectory = goal.trajectory;
    trajectory.joint_names = joint_names;
    trajectory.points.resize(1);

    trajectory.points[0].positions = positions;
    trajectory.points[0].time_from_start = ros::Duration(duration);

    client->sendGoal(goal);
    client->waitForResult(ros::Duration(10.0)); // Adjust the timeout as needed
}

// generate duration by taking the velocity max, min and home position (t = (max - min) / velocity)
std::vector<std::vector<double>> calc_duration(std::vector<double> home_position, std::vector<double> max_position, std::vector<double> min_position, std::vector<std::vector<double>> velocity){
    
    // Initialize the duration vector similar to the velocity vector
    std::vector<std::vector<double>> duration(velocity.size(), std::vector<double>(velocity[0].size(), 0.0));
    
    // Calculate the duration for each joint check if the velocity is 0 or not
    for (int i = 0; i < home_position.size(); ++i){
        // Calculate the duration for the first part of the trajectory
        if (velocity[i][0] != 0){
            duration[i][0] = std::fabs(min_position[i] - home_position[i]) / velocity[i][0];
        }
        else{
            duration[i][0] = 3.0;
        }
        // Calculate the duration for the second part of the trajectory
        if (velocity[i][1] != 0){
            duration[i][1] = std::fabs(max_position[i] - min_position[i]) / velocity[i][1];
        }
        else{
            duration[i][1] = 3.0;
        }
        // Calculate the duration for the third part of the trajectory
        if (velocity[i][2] != 0){
            duration[i][2] = std::fabs(home_position[i] - max_position[i]) / velocity[i][2];
        }
        else{
            duration[i][2] = 3.0;
        }
    }

    return duration;
}

void head(ros::NodeHandle& nh, const std::string controller_name) {
    ControlClientPtr headClient = createclient(controller_name);
    std::vector<std::string> joint_names = {"HeadPitch", "HeadYaw"};
    std::vector<double> position(2, 0.0);
    
    // Maximum and minimum positions for each joint
    std::vector<double> max_position = {0.4451, 2.0857};
    std::vector<double> min_position = {-0.7068, -2.0857};
    std::vector<double> home_position = {-0.2, 0.012271};
    
    std::vector<std::vector<double>> velocities = {{1.5, 1.5, 1.5},{1.2, 1.2, 1.2}};
    std::vector<std::vector<double>> duration = calc_duration(home_position, max_position, min_position, velocities);
    
    ROS_INFO_STREAM("----------[START HEAD CONTROL TEST]-----------");

    // For each joint, move to the maximum position, then to the minimum position, then to the mid-range position
    for (int i = 0; i < joint_names.size(); ++i) {
        ROS_INFO_STREAM("[START] " << joint_names[i] << " test.");

        ROS_INFO_STREAM("Moving to the Minimum position");
        position[i] = min_position[i];
        moveToPosition(headClient, joint_names, duration[i][0], "min", position);

        ROS_INFO_STREAM("Moving to the Maximum position");
        position[i] = max_position[i];
        moveToPosition(headClient, joint_names, duration[i][1], "max", position);

        ROS_INFO_STREAM("Moving to the Mid-range position");
        position[i] = (max_position[i] + min_position[i]) / 2.0;
        moveToPosition(headClient, joint_names, duration[i][2], "mid", position);

        ROS_INFO_STREAM("[END] " << joint_names[i] << " test.");
    }

    ROS_INFO_STREAM("[PUT DOWN HEAD] Moving to the Home position");
    double home_duration = 2.0;
    moveToPosition(headClient, joint_names, home_duration, "home", home_position);

    // End of test 
    ROS_INFO_STREAM("----------[END HEAD CONTROL TEST]-----------");
}

void rArm(ros::NodeHandle& nh, std::string controller_name){
    ControlClientPtr rightArmClient = createclient(controller_name);
    std::vector<std::string> joint_names = {"RShoulderPitch", "RShoulderRoll",  "RElbowRoll", "RElbowYaw", "RWristYaw"};
    std::vector<double> position(5, 0.0);
    
    // Maximum and minimum positions for each joint
    std::vector<double> max_position = {2.0857,  -0.0087,  1.5620,  2.0857,  1.8239};
    std::vector<double> min_position = {-2.0857, -1.5620 , 0.0087, -2.0857, -1.5620};
    std::vector<double> home_position = {1.7410, -0.09664, 0.09664, 1.6981, -0.05679};
  
    std::vector<std::vector<double>> velocity = {{1.5, 1.5, 0.1}, {1.2, 0.8, 0.15},{0.1, 0.8, 1.2}, {2.0, 1.5, 0.2}, {1.8, 1.8, 1.8}};
    std::vector<std::vector<double>> duration = calc_duration(home_position, max_position, min_position, velocity);

    ROS_INFO_STREAM("----------[START RIGHT ARM CONTROL TEST]-----------");

    // For each joint, move to the maximum position, then to the minimum position, then to the mid-range position
    for (int i = 0; i < joint_names.size(); ++i) {
        ROS_INFO_STREAM("[START] " << joint_names[i] << " test.");

        ROS_INFO_STREAM("Moving to the Minimum position");
        position[i] = min_position[i];
        moveToPosition(rightArmClient, joint_names, duration[i][0], "min", position);

        ROS_INFO_STREAM("Moving to the Maximum position");
        position[i] = max_position[i];
        moveToPosition(rightArmClient, joint_names, duration[i][1], "max", position);

        ROS_INFO_STREAM("Moving to the Mid-range position");
        position[i] = (max_position[i] + min_position[i]) / 2.0;
        moveToPosition(rightArmClient, joint_names, duration[i][2], "mid", position);

        ROS_INFO_STREAM("[END] " << joint_names[i] << " test.");
    }

    ROS_INFO_STREAM("[PUT DOWN RIGHT ARM] Moving to the Home position");
    double home_duration = 2.0;
    moveToPosition(rightArmClient, joint_names, home_duration, "home", home_position);

    // End of test 
    ROS_INFO_STREAM("----------[END RIGHT ARM CONTROL TEST]-----------");
}

void rHand(ros::NodeHandle& nh, std::string controller_name){
    ControlClientPtr rightHandClient = createclient(controller_name);
    std::vector<std::string> joint_names = {"RHand"};
    std::vector<double> position(1, 0.0);
    
    // Maximum and minimum positions for each joint
    std::vector<double> max_position = {1.0};
    std::vector<double> min_position = {0.0};
    std::vector<double> home_position = {0.66608};
    double velocity = 2.0;

    double duration = std::fabs(max_position[0] - min_position[0]) / velocity;

    ROS_INFO_STREAM("----------[START RIGHT HAND CONTROL TEST]-----------");

    // For each joint, move to the maximum position, then to the minimum position, then to the mid-range position
    for (int i = 0; i < joint_names.size(); ++i) {
        ROS_INFO_STREAM("[START] " << joint_names[i] << " test.");

        ROS_INFO_STREAM("Moving to the Minimum position");
        position[i] = min_position[i];
        moveToPosition(rightHandClient, joint_names, duration, "min", position);

        ROS_INFO_STREAM("Moving to the Maximum position");
        position[i] = max_position[i];
        moveToPosition(rightHandClient, joint_names, duration, "max", position);

        ROS_INFO_STREAM("Moving to the Mid-range position");
        position[i] = (max_position[i] + min_position[i]) / 2.0;
        moveToPosition(rightHandClient, joint_names, duration, "mid", position);

        ROS_INFO_STREAM("[END] " << joint_names[i] << " test.");
    }

    // calc_velocity(home_position, max_position, min_position, duration);

    ROS_INFO_STREAM("[PUT DOWN RIGHT HAND] Moving to the Home position");
    moveToPosition(rightHandClient, joint_names, duration, "home", home_position);

    // End of test 
    ROS_INFO_STREAM("----------[END RIGHT HAND CONTROL TEST]-----------");
}


void lArm(ros::NodeHandle& nh, std::string controller_name){
    ControlClientPtr leftArmClient = createclient(controller_name);
    std::vector<std::string> joint_names = {"LShoulderPitch", "LShoulderRoll", "LElbowRoll", "LElbowYaw", "LWristYaw"};
    std::vector<double> position(5, 0.0);
    
    // Maximum and minimum positions for each joint
    std::vector<double> max_position = {2.0857,  0.0087,  -1.5620, -2.0857,  -1.8239};
    std::vector<double> min_position = {-2.0857, 1.5620 , -0.0087,  2.0857,   1.8239};
    std::vector<double> home_position = {1.7625, 0.09970, -0.1334, -1.7150,  0.06592};

    std::vector<std::vector<double>> velocities = {{1.5, 1.5, 0.1},{1.2, 0.8, 0.15},{0.1, 0.9, 1.2},{2.1, 1.5, 0.2},{1.8, 1.8, 1.9}};
    std::vector<std::vector<double>> duration = calc_duration(home_position, max_position, min_position, velocities);

    ROS_INFO_STREAM("----------[START LEFT ARM CONTROL TEST]-----------");

    // For each joint, move to the maximum position, then to the minimum position, then to the mid-range position
    for (int i = 0; i < joint_names.size(); ++i) {
        ROS_INFO_STREAM("[START] " << joint_names[i] << " test.");

        ROS_INFO_STREAM("Moving to the Minimum position");
        position[i] = min_position[i];
        moveToPosition(leftArmClient, joint_names, duration[i][0], "min", position);

        ROS_INFO_STREAM("Moving to the Maximum position");
        position[i] = max_position[i];
        moveToPosition(leftArmClient, joint_names, duration[i][1], "max", position);

        ROS_INFO_STREAM("Moving to the Mid-range position");
        position[i] = (max_position[i] + min_position[i]) / 2.0;
        moveToPosition(leftArmClient, joint_names, duration[i][2], "mid", position);

        ROS_INFO_STREAM("[END] " << joint_names[i] << " test.");
    }

    // calc_velocity(home_position, max_position, min_position, duration);

    ROS_INFO_STREAM("[PUT DOWN LEFT ARM] Moving to the Home position");
    double home_duration = 2.0;
    moveToPosition(leftArmClient, joint_names, home_duration, "home", home_position);

    // End of test
    ROS_INFO_STREAM("----------[END LEFT ARM CONTROL TEST]-----------");
}

void lHand(ros::NodeHandle& nh, std::string controller_name){
    ControlClientPtr leftHandClient = createclient(controller_name);
    std::vector<std::string> joint_names = {"LHand"};
    std::vector<double> position(1, 0.0);
    
    // Maximum and minimum positions for each joint
    std::vector<double> max_position = {1.0};
    std::vector<double> min_position = {0.0};
    std::vector<double> home_position = {0.6695};

    double velocity = 2.0;
    double duration = std::fabs(max_position[0] - min_position[0]) / velocity;

    ROS_INFO_STREAM("----------[START LEFT HAND CONTROL TEST]-----------");

    // For each joint, move to the maximum position, then to the minimum position, then to the mid-range position
    for (int i = 0; i < joint_names.size(); ++i) {
        ROS_INFO_STREAM("[START] " << joint_names[i] << " test.");

        ROS_INFO_STREAM("Moving to the Minimum position");
        position[i] = min_position[i];
        moveToPosition(leftHandClient, joint_names, duration, "min", position);

        ROS_INFO_STREAM("Moving to the Maximum position");
        position[i] = max_position[i];
        moveToPosition(leftHandClient, joint_names, duration, "max", position);

        ROS_INFO_STREAM("Moving to the Mid-range position");
        position[i] = (max_position[i] + min_position[i]) / 2.0;
        moveToPosition(leftHandClient, joint_names, duration, "mid", position);

        ROS_INFO_STREAM("[END] " << joint_names[i] << " test.");
    }

    // calc_velocity(home_position, max_position, min_position, duration);

    ROS_INFO_STREAM("[PUT DOWN LEFT HAND] Moving to the Home position");
    moveToPosition(leftHandClient, joint_names, duration, "home", home_position);

    // End of test
    ROS_INFO_STREAM("----------[END LEFT HAND CONTROL TEST]-----------");
}

void leg(ros::NodeHandle& nh, std::string controller_name){
    ControlClientPtr legClient = createclient(controller_name);
    std::vector<std::string> joint_names = {"HipPitch", "HipRoll", "KneePitch"};
    std::vector<double> position(3, 0.0);
    
    
    // Maximum and minimum positions for each joint
    std::vector<double> max_position = {1.0385,   0.5149,   0.5149};
    std::vector<double> min_position = {-1.0385, -0.5149 , -0.5149};
    std::vector<double> home_position = {-0.0107, -0.00766, 0.03221};

    std::vector<std::vector<double>> velocities = {{0.5, 0.5, 0.5},{0.5, 0.5, 0.5},{0.5, 0.5, 0.5}};
    std::vector<std::vector<double>> duration = calc_duration(home_position, max_position, min_position, velocities);


    ROS_INFO_STREAM("----------[START LEG CONTROL TEST]-----------");

    // For each joint, move to the maximum position, then to the minimum position, then to the mid-range position
    for (int i = 0; i < joint_names.size(); ++i) {
        ROS_INFO_STREAM("[START] " << joint_names[i] << " test.");

        ROS_INFO_STREAM("Moving to the Minimum position");
        position[i] = min_position[i];
        moveToPosition(legClient, joint_names, duration[i][0], "min", position);

        ROS_INFO_STREAM("Moving to the Maximum position");
        position[i] = max_position[i];
        moveToPosition(legClient, joint_names, duration[i][1], "max", position);

        ROS_INFO_STREAM("Moving to the Mid-range position");
        position[i] = (max_position[i] + min_position[i]) / 2.0;
        moveToPosition(legClient, joint_names, duration[i][2], "mid", position);

        ROS_INFO_STREAM("[END] " << joint_names[i] << " test.");
    }

    // calc_velocity(home_position, max_position, min_position, duration);

    ROS_INFO_STREAM("[PUT DOWN LEG] Moving to the Home position");
    double home_duration = 2.0;
    moveToPosition(legClient, joint_names, home_duration, "home", home_position);

    // End of test
    ROS_INFO_STREAM("----------[END LEG CONTROL TEST]-----------");
}

// Function to publish a velocity command to a joint
void publish_velocity(ros::Publisher &pub, geometry_msgs::Twist &msg, ros::Rate &rate, double duration) {
    ros::Time startTime = ros::Time::now();
    ros::Duration waitTime = ros::Duration(duration); 
    ros::Time endTime = startTime + waitTime;
    // Publish the trajectory for 1 seconds
    while(ros::ok() && ros::Time::now() < endTime) {
        pub.publish(msg);
        rate.sleep();
    }
}

void wheels(ros::NodeHandle& nh){
    // find the respective topic
   std::string topic_name = extract_topic("Wheels");
   
   // Create a publisher to publish geometry_msgs::Twist messages on the /pepper/cmd_vel topic
   ros::Publisher pub = nh.advertise<geometry_msgs::Twist>(topic_name, 1000, true);

   // Set the publishing rate to 50 Hz
   ros::Rate rate(50); 

   int i = 1; // create iterator variable
   // Create a Twist message object
   geometry_msgs::Twist msg;
   
   ROS_INFO_STREAM("-------[START WHEEL CONTROL TEST]--------");
   /* [1] THIS SECTION PUBLISHES A LINEAR VELOCITY ON THE CMD VEL TOPIC */
   ROS_INFO_STREAM("[LINEAR VELOCITY START] Publishing linear velocity on the cmd vel started.");
   
   // Initialize the message with 0 linear velocity
   ROS_INFO_STREAM("[ZERO VELOCITY] Publishing 0 velocity value.");
   msg.linear.x = 0.0;

   // Publish 0 velocity
   publish_velocity(pub, msg, rate, 1);

   // Publish a fixed positive linear velocity
   ROS_INFO_STREAM("[POSITIVE VELOCITY] Publishing a fixed positive velocity value");
   msg.linear.x = 0.05;

   // Publish the positive velocity 
   publish_velocity(pub, msg, rate, 4);

   // Reset linear velocity to 0
   ROS_INFO_STREAM("[ZERO VELOCITY] Publishing 0 velocity value.");
   msg.linear.x = 0.0;

   // Publish 0 velocity 
   publish_velocity(pub, msg, rate, 2);

   // Publish a fixed negative linear velocity
   ROS_INFO_STREAM("[NEGATIVE VELOCITY] Publishing a fixed negative velocity value");
   msg.linear.x = -0.05;

   // Publish the negative velocity 
   publish_velocity(pub, msg, rate, 4);

   // Reset linear velocity to 0
   ROS_INFO_STREAM("[ZERO VELOCITY] Publishing 0 velocity value.");
   msg.linear.x = 0.0;

   // Publish 0 velocity 
   publish_velocity(pub, msg, rate, 4);
   
   ROS_INFO_STREAM("[LINEAR VELOCITY END] Publishing linear velocity ended.");
   
   /* [2] THIS SECTION PUBLISHES AN ANGULAR VELOCITY ON THE CMD VEL TOPIC */
   ROS_INFO_STREAM("[ANGULAR VELOCITY START] Publishing angular velocity on the cmd vel started.");
   
   // Initialize the message with 0 angular velocity
   ROS_INFO_STREAM("[ZERO VELOCITY] Publishing 0 velocity value.");
   msg.angular.z = 0.0;

   // Publish 0 velocity 
   publish_velocity(pub, msg, rate, 2);

   // Publish a fixed positive angular velocity
   ROS_INFO_STREAM("[POSITIVE VELOCITY] Publishing a fixed positive velocity value");
   msg.angular.z = 0.3925;

   // Publish the positive velocity 
   publish_velocity(pub, msg, rate, 4);

   // Reset angular velocity to 0
   ROS_INFO_STREAM("[ZERO VELOCITY] Publishing 0 velocity value.");
   msg.angular.z = 0.0;

   // Publish 0 velocity 
   publish_velocity(pub, msg, rate, 1);

   // Publish a fixed negative angular velocity
   ROS_INFO_STREAM("[NEGATIVE VELOCITY] Publishing a fixed negative velocity value");
   msg.angular.z = -0.3925;

   // Publish the negative velocity 
   publish_velocity(pub, msg, rate, 4);

   // Reset angular velocity to 0
   ROS_INFO_STREAM("[ZERO VELOCITY] Publishing 0 velocity value.");
   msg.angular.z = 0.0;

   // Publish 0 velocity 
   publish_velocity(pub, msg, rate, 4);
   
   ROS_INFO_STREAM("[ANGULAR VELOCITY END] Publishing angular velocity ended.");
    
   // Print success message
   ROS_INFO_STREAM("[SUCCESS] Wheel control test completed.");
   ROS_INFO_STREAM("                                       ");
}