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


# include "overt_attention/overtAttention.h"

// Home positions for the robot head
std::vector<double> head_home_position = {0.0, 0.0};   // Head pitch and yaw

// Joint states of the robot - updated by subscribing to the /sensor_msgs/joint_states topic
std::vector<double> head_joint_states = {0.0, 0.0};

// Coordinates of the robot in the world (x, y, z, theta) - updated by subscribing to the /robotLocalization/pose topic
std::vector<double> robot_pose = {0.0, 0.0, 0.0};

// Head joint angles for the robot control during scanning or social attention
// double attention_head_pitch = 0.0;
// double attention_head_yaw = 0.0;
std::vector<double> attention_head_pitch;
std::vector<double> attention_head_yaw;

std::vector<int> face_labels;                               // Stores the labels of the detected faces
std::vector<int> viewed_face_labels;                        // Stores the labels of the detected faces
int last_seen_label = -1;                                   // Stores the label of the last seen face
int current_label = -1;                                     // Stores the label of the current face

std::vector<double> gaze_angles;                           // Stores the gaze angles of the detected faces

double previous_attention_head_pitch = 0.0;
double previous_attention_head_yaw = 0.0;

double angle_of_sound = 0.0;                        // Stores the angle of the sound source
double previous_angle_of_sound = 0.0;                       // Stores the previous angle of sound

bool face_detected = false;                         // Stores the status of face detection
bool sound_detected = false;                        // Stores the status of sound detection

// Variables for the attention mode set
int attention_mode = ATTENTION_MODE_SOCIAL;        // Stores the attention mode currently set. Default is -1 on initialization
double location_x = 0.0;                            // Stores the x-coordinate of the location to pay attention to
double location_y = 0.0;                            // Stores the y-coordinate of the location to pay attention to
double location_z = 0.0;                            // Stores the z-coordinate of the location to pay attention to
bool location_attended_to = false;                  // Stores the status if the location request has been attended to once

// Variables for the attention activation set
int system_activation_status = -1;                  // Stores the activation status of the attention system. Default is -1 on initialization

// Variable for social attention done once
bool social_attention_done = true;
int sound_count = 0;

// Variable for checking if engagement is found - 0 is neutral, 1 is success, -1 is failure
int engagement_status = ENGAGEMENT_STATUS_NEUTRAL;                          // Stores the engagement status of the robot

std::vector<int> scanning_angles = {-45, 0, 45};         // Scanning angles for the robot head
size_t scanning_index = 0;                               // Index for the scanning angles
bool forward_scanning = true;                            // Flag to indicate the direction of scanning

std::vector<int> seeking_angles = {-90, 90};         // Scanning angles for the robot head
size_t seeking_index = 0;                               // Index for the scanning angles
bool forward_seeking = true;                            // Flag to indicate the direction of scanning

// Message to publish the engagement status
std_msgs::Float64 overt_attention_engagement_status_msg;

// // Publisher for the velocity commands
// ros::Publisher velocity_publisher;

// // Declare the publisher of the /overt_attention/engagement_status topic
// ros::Publisher overt_attention_engagement_status_pub;

/* 
 *   Function to read the the robot pose from an input file
 * @param:
 *   robot_pose_input: vector to store the robot pose
 *
 * @return:
 *    None
 */
void read_robot_pose_input(std::vector<double>& robot_pose_input){
    bool debug_mode = false;   // used to turn debug message on

    std::string data_file = "robotPose.dat";    // data filename
    std::string data_path;                                          // data path
    std::string data_path_and_file;                                 // data path and filename

    std::string x_key = "x";                                         // x key
    std::string y_key = "y";                                         // y key
    std::string theta_key = "theta";                                 // theta key

    std::string x_value;                                             // x value
    std::string y_value;                                             // y value
    std::string z_value;                                             // z value
    std::string theta_value;                                         // theta value

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

    // Open data file
    std::ifstream data_if(data_path_and_file.c_str());
    if (!data_if.is_open()){
        printf("Unable to open the data file %s\n", data_path_and_file.c_str());
        prompt_and_exit(1);
    }

    std::string data_line_read;  // variable to read the line in the file
    // Get key-value pairs from the data file
    while(std::getline(data_if, data_line_read)){
        std::istringstream iss(data_line_read);
        std::string param_key;
        std::string param_value;
        iss >> param_key;
        trim(param_key);
        std::getline(iss, param_value);
        iss >> param_value;
        trim(param_value);

        // Read the x value of the robot pose
        if (param_key == x_key){
            x_value = param_value;
            robot_pose_input[0] = std::stod(param_value);
        }

        // Read the y value of the robot pose
        else if (param_key == y_key){
            y_value = param_value;
            robot_pose_input[1] = std::stod(param_value);
        }

        // Read the theta value of the robot pose
        else if (param_key == theta_key){
            theta_value = param_value;
            robot_pose_input[2] = std::stod(param_value);
        }
    }
    // Close the data file
    data_if.close();

    if (debug_mode){
        printf("Robot location input:\n");
        printf("\tX: %f\n", robot_pose_input[0]);
        printf("\tY: %f\n", robot_pose_input[1]);
        printf("\tTheta: %f\n", robot_pose_input[2]);
    }
}

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
ControlClientPtr create_client(const std::string& topic_name) {
    // Create a new action client
    ControlClientPtr actionClient(new ControlClient(topic_name, true));
    int max_iterations = 10;        // maximum number of iterations to wait for the server to come up

    for (int iterations = 0; iterations < max_iterations; ++iterations) {
        if (actionClient->waitForServer(ros::Duration(5.0))) {
            return actionClient;   // return the action client if the server is available
        }
        ROS_DEBUG("Waiting for the %s controller to come up", topic_name.c_str());
    }
    // Throw an exception if the server is not available and client creation fails
    throw std::runtime_error("Error creating action client for " + topic_name + " controller: Server not available");
}

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

    std::string topic_line_read;   // variable to read the line in the file
    // Get key-value pairs from the topic file
    while(std::getline(topic_if, topic_line_read)){
        std::istringstream iss(topic_line_read);
        std::string param_key;
        std::string param_value;
        iss >> param_key;
        trim(param_key);
        std::getline(iss, param_value);
        iss >> param_value;
        trim(param_value);
        if (param_key == key) {                     // if the key is found
            topic_value = param_value;              // set the topic value
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

/* Print the overt attention configuration */
void print_configuration(string platform, string camera, int realignment_threshold, int x_offset_to_head_yaw, int y_offset_to_head_pitch, string simulator_topics, string robot_topics, string topics_filename, bool debug_mode){
    printf("Platform: %s\n", platform.c_str());
    printf("Camera: %s\n", camera.c_str());
    printf("Realignment Threshold: %d\n", realignment_threshold);
    printf("X Offset to Head Yaw: %d\n", x_offset_to_head_yaw);
    printf("Y Offset to Head Pitch: %d\n", y_offset_to_head_pitch);
    printf("Simulator Topics: %s\n", simulator_topics.c_str());
    printf("Robot Topics: %s\n", robot_topics.c_str());
    printf("Topics Filename: %s\n", topics_filename.c_str());
    printf("Debug Mode: %s\n", debug_mode ? "true" : "false");
}

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
int read_configuration_file(string* platform, string* camera, int* realignment_threshold, int* x_offset_to_head_yaw, int* y_offset_to_head_pitch, string* simulator_topics, string* robot_topics, string* topics_filename, bool* debug_mode){
    std::string config_file = "overtAttentionConfiguration.ini";        // data filename
    std::string config_path;                                            // data path
    std::string config_path_and_file;                                   // data path and filename
     
    std::string platform_key = "platform";                              // platform key 
    std::string camera_key = "camera";                                  // camera key
    std::string realignment_threshold_key = "realignmentThreshold";     // realignment threshold key
    std::string x_offset_to_head_yaw_key = "xOffsetToHeadYaw";          // x offset to head yaw key
    std::string y_offset_to_head_pitch_key = "yOffsetToHeadPitch";      // y offset to head pitch key
    std::string simulator_topics_key = "simulatorTopics";               // simulator topics key
    std::string robot_topics_key = "robotTopics";                       // robot topics key
    std::string verbose_mode_key = "verboseMode";                       // verbose mode key

    std::string platform_value;                                         // platform value 
    std::string camera_value;                                           // camera value
    std::string realignment_threshold_value;                            // realignment threshold value
    std::string x_offset_to_head_yaw_value;                             // x offset to head yaw value
    std::string y_offset_to_head_pitch_value;                           // y offset to head pitch value
    std::string simulator_topics_value;                                 // simulator topics value
    std::string robot_topics_value;                                     // robot topics value
    std::string verbose_mode_value;                                     // verbose mode value

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

    // Open configuration file
    std::ifstream data_if(config_path_and_file.c_str());
    if (!data_if.is_open()){
        printf("Unable to open the data file %s\n", config_path_and_file.c_str());
        return 1;
    }

    std::string data_line_read;  // variable to read the line in the file
    // Get key-value pairs from the configuration file
    while(std::getline(data_if, data_line_read)){
        std::istringstream iss(data_line_read);
        std::string param_key, param_value;
        iss >> param_key;
        trim(param_key);
        std::getline(iss, param_value);
        iss >> param_value;
        trim(param_value);
        // printf("paramKey: %s; paramValue: %s\n", paramKey.c_str(), paramValue.c_str());
        
        if (param_key == platform_key){ 
            boost::algorithm::to_lower(param_value); // modifies string to lower case
            platform_value = param_value;
            *platform = param_value;
            if(platform_value != "robot" && platform_value != "simulator"){
                printf("Platform value not supported. Supported values are: robot and simulator\n");
                return 1;
            }
        }
        
        else if (param_key == camera_key){ 
            boost::algorithm::to_lower(param_value); // modifies string to lower case
            camera_value = param_value;
            *camera = param_value;
            if(camera_value != "frontcamera" && camera_value != "stereocamera"){
                printf("Camera value not supported. Supported values are: frontcamera and stereocamera\n");
                return 1;
            }
        }

        else if (param_key == realignment_threshold_key){ 
            realignment_threshold_value = param_value;
            *realignment_threshold = std::stoi(param_value);
        }

        else if (param_key == x_offset_to_head_yaw_key){ 
            x_offset_to_head_yaw_value = param_value;
            *x_offset_to_head_yaw = std::stoi(param_value);
        }

        else if (param_key == y_offset_to_head_pitch_key){ 
            y_offset_to_head_pitch_value = param_value;
            *y_offset_to_head_pitch = std::stoi(param_value);
        }

        else if (param_key == simulator_topics_key){ 
            simulator_topics_value = param_value;
            *simulator_topics = param_value;
        }

        else if (param_key == robot_topics_key){ 
            robot_topics_value = param_value;
            *robot_topics = param_value;
        }

        else if (param_key == verbose_mode_key){ 
            boost::algorithm::to_lower(param_value); // modifies string to lower case
            verbose_mode_value = param_value;
            if(verbose_mode_value == "true"){
                *debug_mode = true;
            }
            else if(verbose_mode_value == "false"){
                *debug_mode = false;
            }
            else{
                printf("Verbose mode value not supported. Supported values are: true and false\n");
                return 1;
            }
        }
    }
    data_if.close();

    if(*platform == "" || *camera == "" || *simulator_topics == "" || *robot_topics == ""){
        printf("Unable to find a valid configuration. Verify you have values in the configuration.\n");
        return 1;
    }

    if (platform_value == "robot"){
        *topics_filename = *robot_topics;
    }
    else if(platform_value == "simulator"){
        *topics_filename = *simulator_topics;
    }

    return 0;
}

void extractConfig(std::map<std::string, std::string>& configMap) {
    std::string conf_file = "overtAttentionConfiguration.ini"; // configuration filename
    std::string config_path;                                  // configuration path
    std::string config_path_and_file;                         // configuration path and filename

    std::string topic_file;                                   // topic filename
    std::string topic_path;                                   // topic filename path
    std::string topic_path_and_file;                          // topic with path and file
    
    // Construct the full path of the configuration file
    #ifdef ROS
        config_path = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
        topic_path = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        config_path = "..";
        topic_path = "..";
    #endif

    // set configuration path
    config_path += "/config/";
    config_path_and_file = config_path;
    config_path_and_file += conf_file;

    if (DEBUG) {
        cout<<"Config file path is: "<<config_path_and_file<<endl;
    }

    std::ifstream file(config_path_and_file);
    if (!file.is_open()){
        cout<<__FUNCTION__<<": Unable to open the configuration file "<<config_path_and_file<<endl;
        
        prompt_and_exit(1);
    }

    std::string line;

    while (std::getline(file, line)) {
        std::istringstream is_line(line);
        std::string key;
        if (std::getline(is_line, key, ' ')) {
            std::string value;
            if (std::getline(is_line, value)) {
                // Trim leading and trailing whitespaces from the value
                value = std::regex_replace(value, std::regex("^\\s+|\\s+$"), "");
                configMap[key] = value;
                // std::cout << key << ": " << value << std::endl;
            }
        }
    }

    // Determine which topics file to read based on platform
    std::string topicsFile = (configMap["platform"] == "simulator") ? configMap["simulatorTopics"] : configMap["robotTopics"];

    // set topic path    
    topic_path += "/data/";
    topic_path_and_file = topic_path;
    topic_path_and_file += topicsFile;

    if (DEBUG) {
        cout<<"Topic file path is: "<<topic_path_and_file<<endl;
    }

    // Read topics file and get camera value
    std::ifstream topicsFilestream(topic_path_and_file);
    if (!topicsFilestream.is_open()){
        cout<<__FUNCTION__<<": Unable to open the topic file "<<topic_path_and_file<<endl;
        
        prompt_and_exit(1);
    }
    std::map<std::string, std::string> topicsMap;

    while (std::getline(topicsFilestream, line)) {
        // Skip comments
        if (line[0] == '#') continue;

        std::istringstream is_line(line);
        std::string key;
        if (std::getline(is_line, key, ' ')) {
            std::string value;
            if (std::getline(is_line, value)){
                value = std::regex_replace(value, std::regex("^\\s+|\\s+$"), ""); 
                topicsMap[key] = value;
                // std::cout << key << ": " << value << std::endl;
            }
        }
    }

    // Update camera value in configMap
    configMap["topic"] = topicsMap[configMap["camera"]];
}

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
void get_head_angles(double camera_x, double camera_y, double camera_z, double* head_yaw, double* head_pitch){
   double link_1 = -38.0;
   double link_2 = 169.9;
   double link_3 = 93.6;
   double link_4 = 61.6;

   // theta1
   *head_yaw = atan2(camera_y, (camera_x - link_1));
   // theta2
   *head_pitch = asin((link_2 - camera_z) / sqrt(pow(link_4,2) + pow(link_3,2))) + atan(link_4/link_3);

   // Check if the calculated angles fall within Pepper's range
   // if not set the angles to 0
   if (isnan(*head_yaw) || *head_yaw < -2.1 || *head_yaw > 2.1){
      *head_yaw = 0.0;
   }
   if (isnan(*head_pitch) || *head_pitch < -0.71 || *head_pitch > 0.638){
      *head_pitch = 0.0;
   }
}

/*  
 *   Function to compute the angle changes required refocus the robot head on a point it's FOV
 *
 *   @param:
 *       center_x: x coordinate of the point of interest
 *       center_y: y coordinate of the point of interest
 *       image_width: width of the original image
 *       image_height: height of the original image
 *       theta_v: the vertical FOV of the camera
 *       theta_h: the horizontal FOV of the camera
 *
 *   @return:
 *       AngleChange: the head_yaw and head_pitch angle changes required
 */

AngleChange get_angles_from_pixel(double center_x, double center_y, double image_width, double image_height, double theta_h, double theta_v){
    // Calculate the offsets from the image center
    double x_offset = center_x - image_width / 2.0;
    double y_offset = center_y - image_height / 2.0;
    
    // Calculate the proportion of the offset relative to the image dimensions
    double x_proportion = x_offset / image_width;
    double y_proportion = y_offset / image_height;
    
    // Calculate the angle changes
    double delta_yaw = x_proportion * theta_h * -1;
    double delta_pitch = y_proportion * theta_v;
    
    return {delta_yaw, delta_pitch};
}

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
void move_robot_head(std::string head_topic, double head_pitch, double head_yaw, double gesture_duration, bool debug){
    // Create a control client for the head
    ControlClientPtr head_client = create_client(head_topic);
    std::vector<std::string> head_joint_names = {"HeadPitch", "HeadYaw"};   // Set the joint names for the head to the specified joint names
    int number_of_joints = head_joint_names.size(); 
    
    // positions for each joint
    std::vector<double> head_position = {head_pitch, head_yaw};

    // Vectors to store the positions, velocities, accelerations and duration of the trajectory
    std::vector<std::vector<double>> positions_t_head;
    std::vector<std::vector<double>> velocities_t_head;
    std::vector<std::vector<double>> accelerations_t_head;
    std::vector<double> duration_t_head;

    move_to_position(head_client, head_joint_names, gesture_duration, head_position);
}

void move_robot_head_wheels(std::string head_topic, double head_pitch, double head_yaw, double gesture_duration, 
                            bool rotate_robot, double angle_radians, ros::Publisher velocity_publisher, bool debug){
    // Create a control client for the head
    ControlClientPtr head_client = create_client(head_topic);
    std::vector<std::string> head_joint_names = {"HeadPitch", "HeadYaw"};   // Set the joint names for the head to the specified joint names
    int number_of_joints = head_joint_names.size(); 
    
    // positions for each joint
    std::vector<double> head_position = {head_pitch, head_yaw};
    std::vector<double> head_current_position = head_joint_states;

    // Vectors to store the positions, velocities, accelerations and duration of the trajectory
    std::vector<std::vector<double>> positions_t_head;
    std::vector<std::vector<double>> velocities_t_head;
    std::vector<std::vector<double>> accelerations_t_head;
    std::vector<double> duration_t_head;

    compute_trajectory(head_joint_states, head_position, number_of_joints, gesture_duration, positions_t_head, velocities_t_head, accelerations_t_head, duration_t_head);

    // move_robot_head_wheels_to_position(head_client, head_joint_names, gesture_duration, head_position, rotate_robot, angle_radians, velocity_publisher, debug);
    move_robot_head_wheels_to_position_biological_motion(head_client, head_joint_names, gesture_duration, duration_t_head, positions_t_head, velocities_t_head, accelerations_t_head, rotate_robot, angle_radians, velocity_publisher, debug);
}

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
int location_attention(float camera_x, float camera_y, float camera_z, string topics_file, ros::Publisher velocity_publisher, bool debug){
    // Gesture duration in milliseconds
    double duration     = 0.5;

    // Variables to store the head angles
    double head_yaw; 
    double head_pitch;

    get_head_angles(camera_x, camera_y, camera_z, &head_yaw, &head_pitch);
    // 33.26, -304, 120.17
    if(debug){
        printf("Head Yaw: %f, Head Pitch: %f\n", head_yaw, head_pitch);
    }

    std::string head_topic;     // stores the head topic
    
    // Extract the topic for the head
    head_topic = extract_topic("Head", topics_file);

    move_robot_head(head_topic, head_pitch, head_yaw, duration, debug);

    return 1;       // attention executed successfully
}

int location_attention_old(float point_x, float point_y, float point_z, string topics_file, ros::Publisher velocity_publisher, bool debug){
    // Robot pose coordinates
    double robot_x      = robot_pose[0] * 1000;     // Convert the robot's x coordinate from meters to millimeters
    double robot_y      = robot_pose[1] * 1000;     // Convert the robot's y coordinate from meters to millimeters
    double robot_z      = 0;
    double robot_theta  = robot_pose[2];
    robot_theta         = radians(robot_theta);      // Convert the robot's orientation from degrees to radians

    
    // Head coordinates
    double head_x       = -38.0;
    double head_y       = 0.0;
    double head_z       = 169.9 + TORSO_HEIGHT;
    double l_head_1     = 93.6;
    double l_head_2     = 0.0;

    // Camera coordinates
    double camera_x     = 0.0;
    double camera_y     = 0.0;
    double camera_z     = 0.0;

    // Pointing coordinates
    double pointing_x    = 0.0;
    double pointing_y    = 0.0;
    double pointing_z    = 0.0;

    // Gesture duration in milliseconds
    double duration     = 1.0;

    bool pose_achievable= true;     // flag to check if the pose is achievable
    double rotation_angle = 0.0;    // angle to rotate the robot if the pose is not achievable

    /* Compute the pointing coordinates with respect to the robot pose in the environment */
    double relative_pointing_x = (point_x * 1000) - robot_x;        // Convert the pointing coordinates from meters to millimeters
    double relative_pointing_y = (point_y * 1000) - robot_y;        // Convert the pointing coordinates from meters to millimeters
    pointing_x = (relative_pointing_x * cos(-robot_theta)) - (relative_pointing_y * sin(-robot_theta));
    pointing_y = (relative_pointing_y * cos(-robot_theta)) + (relative_pointing_x * sin(-robot_theta));
    pointing_z = point_z * 1000; 

    /* Account for unreachable points in the cartesian space 
    (e.g. outside the robot's forward reach)
    Rotate the robot appropriately (by 90 degrees) if necessary */

    // Case 1: Pointing coordinates directly in front of the robot (+x direction): No rotation is needed, just choose arm
    if(pointing_x >= 0.0){
        pose_achievable = true;     // The pointing coordinates are reachable without rotating the robot
    }
    // Case 2: Pointing coordinates directly behind the robot (-x direction): 
    // Rotate the robot by 90 degrees left or right depending on the y coordinate of the pointing coordinates
    else if(pointing_x < 0.0){
        pose_achievable = false;
        double temp_var = 0.0;
        // Rotate 90 degrees clockwise and use right arm if the pointing coordinates are to the right of the robot (-y direction)
        if(pointing_y <= 0.0){
            rotation_angle = -90.0;
            // Realign the pointing coordinates considering the rotation
            temp_var = pointing_x;
            pointing_x = -pointing_y;
            pointing_y = temp_var;
        }
        // Rotate 90 degrees anticlockwise and use left arm if the pointing coordinates are to the left of the robot (+y direction)
        else if(pointing_y > 0.0){
            rotation_angle = 90.0;
            // Realign the pointing coordinates considering the rotation
            temp_var = pointing_x;
            pointing_x = pointing_y;
            pointing_y = -temp_var;
        }
    }

    double distance = sqrt(pow((point_x - head_x), 2) + pow((point_y - head_y), 2) + pow((point_z - head_z), 2));
    l_head_2 = distance - l_head_1;

    // Calculate the camera coordinates
    distance = sqrt(pow((pointing_x - head_x), 2) + pow((pointing_y - head_y), 2) + pow((pointing_z - head_z), 2));
    l_head_2 = distance - l_head_1;

    camera_x = ((l_head_1 * pointing_x) + (l_head_2 * head_x))/(l_head_1 + l_head_2);
    camera_y = ((l_head_1 * pointing_y) + (l_head_2 * head_y))/(l_head_1 + l_head_2);
    camera_z = ((l_head_1 * pointing_z) + (l_head_2 * head_z))/(l_head_1 + l_head_2);
    camera_z = camera_z + 61.6 - TORSO_HEIGHT;

    // Variables to store the head angles
    double head_yaw; 
    double head_pitch;

    get_head_angles(camera_x, camera_y, camera_z, &head_yaw, &head_pitch);
    // 33.26, -304, 120.17
    if(debug){
        printf("Head Yaw: %f, Head Pitch: %f\n", head_yaw, head_pitch);
    }

    std::string head_topic;     // stores the head topic
    
    // Extract the topic for the head
    head_topic = extract_topic("Head", topics_file);

    // Rotate the robot by 90 degrees if the pointing coordinates are unreachable
    if(!pose_achievable){
        rotate_robot(rotation_angle, velocity_publisher, debug);
    }

    move_robot_head(head_topic, head_pitch, head_yaw, duration, debug);

    // Return the robot to the original orientation if it was rotated
    if(!pose_achievable){
        rotate_robot(-rotation_angle, velocity_publisher, debug);
    }

    return 1;       // attention executed successfully
}

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
void rotate_robot(double angle_degrees, ros::Publisher velocity_publisher, bool debug){
    if (debug){
        printf("Rotating the robot by %.3f degrees\n", angle_degrees);
    }

    double angle_radians;                           // stores the angle in radians
    angle_radians = radians(angle_degrees);         // Convert angle from degrees to radians (function found in pepperKinematicsUtilities.h)

    // Declare a geometry_msgs::Twist message to send velocity commands to the robot
    geometry_msgs::Twist velocity_command;

    // Set publishing rate to 10 Hz
    ros::Rate loop_rate(10);

    // Set the linear velocities to zero and angular velocity to the angle in radian
    velocity_command.linear.x = 0.0;
    velocity_command.linear.y = 0.0;
    velocity_command.linear.z = 0.0;

    velocity_command.angular.x = 0.0;
    velocity_command.angular.y = 0.0;
    velocity_command.angular.z = angle_radians;

    // Publish the velocity command to the robot
    velocity_publisher.publish(velocity_command);

    // Sleep for the duration of the rotation
    loop_rate.sleep();
}

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
                        std::vector<std::vector<double>>& accelerations, std::vector<double>& durations){
    // Declare variables
    double time_t = 0;                      // stores the instantaneous time of the trajectory
    std::vector<double> positions_t;        // vector to store the positions of the trajectory
    std::vector<double> velocities_t;       // vector to store the velocities of the trajectory
    std::vector<double> accelerations_t;    // vector to store the accelerations of the trajectory
    std::vector<double> duration_t;         // vector to store the duration of the trajectory
    double acceleration;                    // stores the acceleration
    double velocity;                        // stores the velocity
    double position;                        // stores the position
    double time_step = 1;                   // Time step between each point in the trajectory

    // Clear the existing values
    for(int i = 0; i < positions.size(); i++){
        positions[i].clear();
        velocities[i].clear();
        accelerations[i].clear();
    }
    positions.clear();
    velocities.clear();
    accelerations.clear();

    // Compute the trajectory for each point in time
    while(time_t < trajectory_duration){
        for(int i = 0; i < number_of_joints; i++){     // Create a trajectory for each joint (5 joints for the arm)
            position = start_position[i] + (end_position[i] - start_position[i]) * ((10 * (pow(time_t/trajectory_duration, 3))) - (15 * (pow(time_t/trajectory_duration, 4))) + (6 * (pow(time_t/trajectory_duration, 5))));
            positions_t.push_back(position);

            velocity = ((end_position[i] - start_position[i])/trajectory_duration) * ((30 * (pow(time_t/trajectory_duration, 2))) - (60 * (pow(time_t/trajectory_duration, 3))) + (30 * (pow(time_t/trajectory_duration, 4))));
            velocities_t.push_back(velocity);

            acceleration = ((end_position[i] - start_position[i])/(trajectory_duration*trajectory_duration)) * ((60 * (pow(time_t/trajectory_duration, 1))) - (180 * (pow(time_t/trajectory_duration, 2))) + (120 * (pow(time_t/trajectory_duration, 3))));
            accelerations_t.push_back(acceleration);
        }
        // Store the computed trajectory
        positions.push_back(positions_t);
        velocities.push_back(velocities_t);
        accelerations.push_back(accelerations_t);
        durations.push_back(time_t);

        // Increment the time
        time_t = time_t + time_step;

        // Clear the vectors for the next iteration
        positions_t.clear();
        velocities_t.clear();
        accelerations_t.clear();
    }
    // Compute the trajectory for the last point in time
    time_t = trajectory_duration;
    for(int i = 0; i < number_of_joints; i++){          // Create a trajectory for each joint (5 joints for the arm)   
        position = start_position[i] + (end_position[i] - start_position[i]) * ((10 * (pow(time_t/trajectory_duration, 3))) - (15 * (pow(time_t/trajectory_duration, 4))) + (6 * (pow(time_t/trajectory_duration, 5))));
        positions_t.push_back(position);

        velocity = ((end_position[i] - start_position[i])/trajectory_duration) * ((30 * (pow(time_t/trajectory_duration, 2))) - (60 * (pow(time_t/trajectory_duration, 3))) + (30 * (pow(time_t/trajectory_duration, 4))));
        velocities_t.push_back(velocity);

        acceleration = ((end_position[i] - start_position[i])/(trajectory_duration*trajectory_duration)) * ((60 * (pow(time_t/trajectory_duration, 1))) - (180 * (pow(time_t/trajectory_duration, 2))) + (120 * (pow(time_t/trajectory_duration, 3))));
        accelerations_t.push_back(acceleration);
    }
    // Store the computed trajectory for the last point in time
    positions.push_back(positions_t);
    velocities.push_back(velocities_t);
    accelerations.push_back(accelerations_t);
    durations.push_back(time_t);

    return;
}

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
                        std::vector<double> positions){
    // Create a goal message
    control_msgs::FollowJointTrajectoryGoal goal;
    trajectory_msgs::JointTrajectory& trajectory = goal.trajectory;
    trajectory.joint_names = joint_names;                               // Set the joint names for the actuator to the specified joint names
    trajectory.points.resize(1);                                        // Set the number of points in the trajectory to 1

    trajectory.points[0].positions = positions;                         // Set the positions in the trajectory to the specified positions
    trajectory.points[0].time_from_start = ros::Duration(duration);     // Set the time from start of the trajectory to the specified duration

    // Send the goal to move the actuator to the specified position
    client->sendGoal(goal);
    client->waitForResult(ros::Duration(duration)); // Wait for the actuator to reach the specified position
}

void move_robot_head_wheels_to_position(ControlClientPtr& head_client, const std::vector<std::string>& joint_names, double duration, 
                        std::vector<double> positions, bool rotate_robot, double angle_radians, ros::Publisher velocity_publisher, bool debug){
    // Create a goal message
    control_msgs::FollowJointTrajectoryGoal goal;
    trajectory_msgs::JointTrajectory& trajectory = goal.trajectory;
    trajectory.joint_names = joint_names;                               // Set the joint names for the actuator to the specified joint names
    trajectory.points.resize(1);                                        // Set the number of points in the trajectory to 1

    trajectory.points[0].positions = positions;                         // Set the positions in the trajectory to the specified positions
    trajectory.points[0].time_from_start = ros::Duration(duration);     // Set the time from start of the trajectory to the specified duration

    // Configure parameters for rotating the robot
    geometry_msgs::Twist velocity_command;          // Declare a geometry_msgs::Twist message to send velocity commands to the robot

    ros::Rate loop_rate(10);                       // Set publishing rate to 10 Hz        

    // Set the linear velocities to zero and angular velocity to the angle in radian
    velocity_command.linear.x = 0.0;
    velocity_command.linear.y = 0.0;
    velocity_command.linear.z = 0.0;

    velocity_command.angular.x = 0.0;
    velocity_command.angular.y = 0.0;
    velocity_command.angular.z = angle_radians;

    // Send the goal to move the head to the specified position
    head_client->sendGoal(goal);

    // Rotate the robot by the specified angle
    if(rotate_robot){
        // Publish the velocity command to the robot
        velocity_publisher.publish(velocity_command);

        // // Sleep for the duration of the rotation
        loop_rate.sleep();
    }
    head_client->waitForResult(ros::Duration(duration)); // Wait for the actuator to reach the specified position
}

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
                                        std::vector<std::vector<double>> accelerations){
    // Create a goal message
    control_msgs::FollowJointTrajectoryGoal goal;
    trajectory_msgs::JointTrajectory& trajectory = goal.trajectory;
    trajectory.joint_names = joint_names;                // Set the joint names for the arm to the specified joint names
    trajectory.points.resize(positions.size());          // Set the number of points in the trajectory to the number of positions in the waypoints

    // Set the positions, velocities, accelerations and time from start for each point in the trajectory
    for(int i = 0; i < positions.size(); i++){
        trajectory.points[i].positions = positions[i];
        trajectory.points[i].velocities = velocities[i];
        trajectory.points[i].accelerations = accelerations[i];
        trajectory.points[i].time_from_start = ros::Duration(duration[i]);
    }

    // Send the goal to move the head to the specified position
    client->sendGoal(goal);
    client->waitForResult(ros::Duration(gesture_duration)); // Wait for the arm to reach the specified position
}

void move_robot_head_wheels_to_position_biological_motion(ControlClientPtr& client, const std::vector<std::string>& joint_names, 
                                        double gesture_duration, std::vector<double> duration, 
                                        std::vector<std::vector<double>> positions, std::vector<std::vector<double>> velocities, 
                                        std::vector<std::vector<double>> accelerations,
                                        bool rotate_robot, double angle_radians, ros::Publisher velocity_publisher, bool debug){
    // Create a goal message
    control_msgs::FollowJointTrajectoryGoal goal;
    trajectory_msgs::JointTrajectory& trajectory = goal.trajectory;
    trajectory.joint_names = joint_names;                // Set the joint names for the arm to the specified joint names
    trajectory.points.resize(positions.size());          // Set the number of points in the trajectory to the number of positions in the waypoints

    // Set the positions, velocities, accelerations and time from start for each point in the trajectory
    for(int i = 0; i < positions.size(); i++){
        trajectory.points[i].positions = positions[i];
        trajectory.points[i].velocities = velocities[i];
        trajectory.points[i].accelerations = accelerations[i];
        trajectory.points[i].time_from_start = ros::Duration(duration[i]);
    }

    // Configure parameters for rotating the robot
    geometry_msgs::Twist velocity_command;          // Declare a geometry_msgs::Twist message to send velocity commands to the robot

    ros::Rate loop_rate(5);                       // Set publishing rate to 10 Hz        

    // Set the linear velocities to zero and angular velocity to the angle in radian
    velocity_command.linear.x = 0.0;
    velocity_command.linear.y = 0.0;
    velocity_command.linear.z = 0.0;

    velocity_command.angular.x = 0.0;
    velocity_command.angular.y = 0.0;
    velocity_command.angular.z = angle_radians;

    // Send the goal to move the head to the specified position
    client->sendGoal(goal);

    // Rotate the robot by the specified angle
    if(rotate_robot){
        // Publish the velocity command to the robot
        velocity_publisher.publish(velocity_command);

        // Sleep for the duration of the rotation
        // loop_rate.sleep();
    }

    client->waitForResult(ros::Duration(gesture_duration)); // Wait for the actuator to reach the specified position
}

int social_attention(std::string topics_file, int realignment_threshold, ros::Publisher velocity_publisher, bool debug){
    std::string head_topic;     // stores the head topic
    
    // Extract the topic for the head
    head_topic = extract_topic("Head", topics_file);

    double control_head_pitch;
    double control_head_yaw;
    double current_attention_head_pitch = 0.0;
    double current_attention_head_yaw = 0.0;
    double realignment_threshold_radians = radians(realignment_threshold);
    bool realign_robot_base = false;
    double robot_rotation_angle = 0.0;

    int number_of_faces_to_look_at = NUMBER_OF_FACES_SOCIAL_ATTENTION;

    if(face_detected || sound_detected){
        // IMage control
        // control_head_pitch = attention_head_pitch + head_joint_states[0];
        // control_head_yaw = attention_head_yaw + head_joint_states[1];

        // // Sound control
        // control_head_pitch = 0.0;
        // control_head_yaw = angle_of_sound + head_joint_states[1];
        if(face_detected){
            if (face_labels.size() == 1){
                current_attention_head_pitch = attention_head_pitch[0];
                current_attention_head_yaw = attention_head_yaw[0];
                last_seen_label = -1;
            }
            else{
                for (int i = 0; i < face_labels.size(); i++){
                    if(face_labels[i] != last_seen_label){
                        current_attention_head_pitch = attention_head_pitch[i];
                        current_attention_head_yaw = attention_head_yaw[i];
                        last_seen_label = face_labels[i];
                        break;
                    }
                    if(i == face_labels.size() - 1){
                        last_seen_label = -1;
                    }
                    // current_label += 1;
                }
            }
        }

        control_head_pitch = head_joint_states[0];
        control_head_yaw = head_joint_states[1];

        if(face_detected && sound_detected){            // When a face is detected and sound is detected
            if (sound_count > 30)
            {
                cout << "Sound count: " << sound_count << endl;
                control_head_pitch = 0.0;
                control_head_yaw += angle_of_sound;
                sound_count = 0;
            } else {
                control_head_pitch += current_attention_head_pitch;
                control_head_yaw += current_attention_head_yaw;
            }
            
            // control_head_pitch = current_attention_head_pitch + head_joint_states[0];

            // if((current_attention_head_yaw == 0.0) && (angle_of_sound == 0.0)){
            //     control_head_yaw += current_attention_head_yaw;
            // }
            // else if((current_attention_head_yaw > 0.0) && (angle_of_sound > 0.0)){
            //     control_head_yaw += current_attention_head_yaw;
            // }
            // else if((current_attention_head_yaw < 0.0) && (angle_of_sound < 0.0)){
            //     control_head_yaw += current_attention_head_yaw;
            // }
            // else{
            //     control_head_yaw = head_joint_states[1];
            // }
        }

        else if(face_detected && !sound_detected){      // If a face is detected and sound is not detected
            control_head_pitch += current_attention_head_pitch;
            control_head_yaw += current_attention_head_yaw;
        }

        else if(!face_detected && sound_detected){      // If a face is not detected and sound is detected
            control_head_pitch = 0.0;
            control_head_yaw += angle_of_sound;
            sound_count = 0;
        }


        // if((attention_head_yaw == 0.0) && (angle_of_sound == 0.0)){
        //     control_head_yaw += 0.0;
        // }
        // else if((attention_head_yaw > 0.0) && (angle_of_sound > 0.0)){
        //     control_head_yaw += attention_head_yaw;
        // }
        // else if((attention_head_yaw < 0.0) && (angle_of_sound < 0.0)){
        //     control_head_yaw += attention_head_yaw;
        // }
        // else{
        //     control_head_yaw = head_joint_states[1];
        // }


        if(control_head_pitch >= MAX_HEAD_PITCH){
            control_head_pitch = MAX_HEAD_PITCH;
        }
        else if(control_head_pitch <= MIN_HEAD_PITCH){
            control_head_pitch = MIN_HEAD_PITCH;
        }

        if(control_head_yaw >= realignment_threshold_radians || control_head_yaw <= -realignment_threshold_radians){
            robot_rotation_angle = control_head_yaw;
            control_head_yaw = 0;
            realign_robot_base = true;
        }

        // move_robot_head(head_topic, control_head_pitch, control_head_yaw, 1.0, debug);
        move_robot_head_wheels(head_topic, control_head_pitch, control_head_yaw, 1.0, realign_robot_base, robot_rotation_angle, velocity_publisher, debug);
        ros::Duration(1).sleep();
        
        face_detected = false;
        sound_detected = false;

    }

    return 1;
}

// int social_attention_old(std::string topics_file, int realignment_threshold, ros::Publisher velocity_publisher, bool debug){
//     std::string head_topic;     // stores the head topic
    
//     // Extract the topic for the head
//     head_topic = extract_topic("Head", topics_file);

//     double control_head_pitch;
//     double control_head_yaw;
//     double realignment_threshold_radians = radians(realignment_threshold);
//     bool realign_robot_base = false;
//     double robot_rotation_angle = 0.0;

//     int number_of_faces_to_look_at = NUMBER_OF_FACES_SOCIAL_ATTENTION;

//     if(face_detected || sound_detected){
//         // IMage control
//         // control_head_pitch = attention_head_pitch + head_joint_states[0];
//         // control_head_yaw = attention_head_yaw + head_joint_states[1];

//         // // Sound control
//         // control_head_pitch = 0.0;
//         // control_head_yaw = angle_of_sound + head_joint_states[1];


//         control_head_pitch = 0.0;
//         control_head_yaw = head_joint_states[1];

//         if(face_detected && sound_detected){            // When a face is detected and sound is detected
//             control_head_pitch = attention_head_pitch[0] + head_joint_states[0];

//             if((attention_head_yaw == 0.0) && (angle_of_sound == 0.0)){
//                 control_head_yaw += attention_head_yaw[0];
//             }
//             else if((attention_head_yaw > 0.0) && (angle_of_sound > 0.0)){
//                 control_head_yaw += attention_head_yaw[0];
//             }
//             else if((attention_head_yaw < 0.0) && (angle_of_sound < 0.0)){
//                 control_head_yaw += attention_head_yaw[0];
//             }
//             else{
//                 control_head_yaw = head_joint_states[1];
//             }
//         }

//         else if(face_detected && !sound_detected){      // If a face is detected and sound is not detected
//             control_head_pitch = attention_head_pitch[0] + head_joint_states[0];
//             control_head_yaw += attention_head_yaw[0];
//         }

//         else if(!face_detected && sound_detected){      // If a face is not detected and sound is detected
//             control_head_pitch = 0.0;
//             control_head_yaw += angle_of_sound;
//         }


//         // if((attention_head_yaw == 0.0) && (angle_of_sound == 0.0)){
//         //     control_head_yaw += 0.0;
//         // }
//         // else if((attention_head_yaw > 0.0) && (angle_of_sound > 0.0)){
//         //     control_head_yaw += attention_head_yaw;
//         // }
//         // else if((attention_head_yaw < 0.0) && (angle_of_sound < 0.0)){
//         //     control_head_yaw += attention_head_yaw;
//         // }
//         // else{
//         //     control_head_yaw = head_joint_states[1];
//         // }


//         if(control_head_pitch >= MAX_HEAD_PITCH){
//             control_head_pitch = MAX_HEAD_PITCH;
//         }
//         else if(control_head_pitch <= MIN_HEAD_PITCH){
//             control_head_pitch = MIN_HEAD_PITCH;
//         }

//         if(control_head_yaw >= realignment_threshold_radians || control_head_yaw <= -realignment_threshold_radians){
//             robot_rotation_angle = control_head_yaw;
//             control_head_yaw = 0;
//             realign_robot_base = true;
//         }

//         // move_robot_head(head_topic, control_head_pitch, control_head_yaw, 1.0, debug);
//         move_robot_head_wheels(head_topic, control_head_pitch, control_head_yaw, 1.0, realign_robot_base, robot_rotation_angle, velocity_publisher, debug);
//         ros::Duration(1).sleep();
        
//         face_detected = false;
//         sound_detected = false;

//     }

//     return 1;
// }

int sound_attention(std::string topics_file, bool debug){
    std::string head_topic;     // stores the head topic
    
    // Extract the topic for the head
    head_topic = extract_topic("Head", topics_file);

    double control_head_pitch;
    double control_head_yaw;

    control_head_pitch = head_joint_states[0];
    control_head_yaw = angle_of_sound + head_joint_states[1];

    if(control_head_pitch >= MAX_HEAD_PITCH){
        control_head_pitch = MAX_HEAD_PITCH;
    }
    else if(control_head_pitch <= MIN_HEAD_PITCH){
        control_head_pitch = MIN_HEAD_PITCH;
    }

    move_robot_head(head_topic, control_head_pitch, control_head_yaw, 1.0, debug);
    ros::Duration(3).sleep();
    return 1;
}

int reactive_attention(std::string topics_file, bool debug){
    std::string head_topic;     // stores the head topic
    
    // Extract the topic for the head
    head_topic = extract_topic("Head", topics_file);

    double control_head_pitch;
    double control_head_yaw;

    control_head_pitch = attention_head_pitch[0] + head_joint_states[0];
    control_head_yaw = attention_head_yaw[0] + head_joint_states[1];

    if(control_head_pitch >= MAX_HEAD_PITCH){
        control_head_pitch = MAX_HEAD_PITCH;
    }
    else if(control_head_pitch <= MIN_HEAD_PITCH){
        control_head_pitch = MIN_HEAD_PITCH;
    }

    move_robot_head(head_topic, control_head_pitch, control_head_yaw, 1.0, debug);
    ros::Duration(1).sleep();
    return 1;
}

int get_next_scanning_angle() {
    int current_angle = scanning_angles[scanning_index]; // Get the current angle

    if (forward_scanning) {
        scanning_index++;
        if (scanning_index >= scanning_angles.size()) {
            scanning_index = scanning_angles.size() - 2;
            forward_scanning = false;
        }
    } 
    else {
        if (scanning_index == 0) {
            scanning_index = 1;
            forward_scanning = true;
        } 
        else {
            scanning_index--;
        }
    }

    return current_angle;
}

int scanning_attention(string topics_file, ros::Publisher velocity_publisher, bool debug){
    if(face_detected){
        // Change the attention mode to seek mode
        attention_mode = ATTENTION_MODE_SEEKING;
        face_detected = false;
        return 1;
    }

    int next_angle;
    double control_head_yaw;
    double control_head_pitch = 0.0;
    string head_topic = extract_topic("Head", topics_file);
    // if(sound_detected){
    //     control_head_yaw = angle_of_sound + head_joint_states[1];
    //     sound_detected = false;
    // }
    // else{
        next_angle = get_next_scanning_angle();
        control_head_yaw = radians(next_angle);
    // }

    if(control_head_yaw >= MAX_HEAD_YAW){
        control_head_yaw = MAX_HEAD_YAW;
    }
    else if(control_head_yaw <= MIN_HEAD_YAW){
        control_head_yaw = MIN_HEAD_YAW;
    }

    move_robot_head(head_topic, control_head_pitch, control_head_yaw, 1.0, debug);
    ros::Duration(1).sleep();
    return 1;
}

int get_next_seeking_angle() {
    int current_angle = seeking_angles[seeking_index]; // Get the current angle

    if (forward_seeking) {
        seeking_index++;
        if (seeking_index >= seeking_angles.size()) {
            seeking_index = seeking_angles.size() - 2;
            forward_seeking = false;
        }
    } 
    else {
        if (seeking_index == 0) {
            seeking_index = 1;
            forward_seeking = true;
        } 
        else {
            seeking_index--;
        }
    }

    return current_angle;
}

int seeking_attention(string topics_file, int realignment_threshold, ros::Publisher velocity_publisher, ros::Publisher overt_attention_engagement_status_pub, bool debug){
    if(face_detected){              // Change it to engagement based on gaze angle
        // Change the attention mode to social mode
        attention_mode = ATTENTION_MODE_SOCIAL;

        std::string head_topic;     // stores the head topic
    
        // Extract the topic for the head
        head_topic = extract_topic("Head", topics_file);

        double control_head_pitch;
        double control_head_yaw;
        double realignment_threshold_radians = radians(realignment_threshold);
        bool realign_robot_base = false;
        double robot_rotation_angle = 0.0;

        control_head_pitch = 0.0;
        control_head_yaw = head_joint_states[1];

        control_head_pitch = attention_head_pitch[0] + head_joint_states[0];
        control_head_yaw += attention_head_yaw[0];

        // Move the head/wheel to center on the face
        move_robot_head(head_topic, control_head_pitch, control_head_yaw, 1.0, debug);
        ros::Duration(1).sleep();
        
        face_detected = false;
        engagement_status = ENGAGEMENT_STATUS_ENGAGED;

        // std_msgs::Float64 overt_attention_engagement_status_msg;
        overt_attention_engagement_status_msg.data = engagement_status;
        overt_attention_engagement_status_pub.publish(overt_attention_engagement_status_msg);

        return 1;
    }

    int next_angle;
    double control_head_yaw;
    double control_head_pitch = 0.0;
    string head_topic = extract_topic("Head", topics_file);

    
    next_angle = get_next_seeking_angle();
    control_head_yaw = radians(next_angle);

    if(control_head_yaw >= MAX_HEAD_YAW){
        control_head_yaw = MAX_HEAD_YAW;
    }
    else if(control_head_yaw <= MIN_HEAD_YAW){
        control_head_yaw = MIN_HEAD_YAW;
    }

    move_robot_head(head_topic, control_head_pitch, control_head_yaw, 1.0, debug);
    ros::Duration(2).sleep();
    engagement_status = ENGAGEMENT_STATUS_NEUTRAL;
    return 1;
}

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
double degrees(double radians)
{
    double degrees = radians * (double) 180.0 / (double) M_PI; // David Vernon ... cast to float
    return degrees;
}

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
double radians(double degrees)
{
    double radians = degrees / ((double) 180.0 / (double) M_PI); // David Vernon ... cast to float
    return radians;
}
 
/*  
 *   Function to prompt the user to press any key to exit the program
 *
 *   @param:
 *       status: the status of the program
 *
 *   @return:
 *       None
 */
void prompt_and_exit(int status){
    printf("Press any key to exit ... \n");
    getchar();
    exit(status);
}

/*  
 *   Function to prompt the user to press any key to continue or press X to exit the program
 *
 *   @param:
 *       None
 *
 *   @return:
 *       None
 */
void prompt_and_continue(){
    printf("Press X to quit or Press any key to continue...\n");
    char got_char = getchar();
    if ((got_char == 'X') || (got_char == 'x')){
        printf("Exiting ...\n");
       exit(0);
    }
}