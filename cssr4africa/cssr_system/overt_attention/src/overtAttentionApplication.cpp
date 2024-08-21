/* overtAttentionpplication.cpp
*
* <detailed functional description>
* The component test the functionality of the actuator of the robot using the ROS interface.
* The test is performed by sending commands to the robot and checking if the robot performs the
* expected action. The test is performed in two modes: sequential and parallel. In the sequential
* mode, the tests are performed one after the other. In the parallel mode, the tests are performed
* simultaneously. 

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
* The location in the world to pay attention to in x, y, z coordinates
...
* Configuration File Parameters

* Key                   |     Value 
* --------------------- |     -------------------
* platform                    simulator
* camera                      FrontCamera
* realignmentThreshold        5
* xOffsetToHeadYaw            25
* yOffsetToHeadPitch          20
* simulatorTopics             simulatorTopics.dat
* robotTopics                 pepperTopics.dat
* verboseMode                 true

...
* Subscribed Topics and Message Types
*
* None
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
* overtAttentionConfiguration.ini
...
* Example Instantiation of the Module
*
* rosrun overtAttention overtAttention
...
*
* The clients can call the service by providing the attention mode and the location to pay attention to in the world.
* The service will execute the attention mode selected and attend to the location provided.
* AN example of calling the service is shown below:
* ----- rosservice call /overAttention/set_mode -- social 3.0 2.0 1.0
* This will set the attention mode to social and the location to pay attention to is (3.0, 2.0, 1.0)
*
...
*
* Author: Adedayo Akinade, Carnegie Mellon University Africa
* Email: aakinade@andrew.cmu.edu
* Date: April 12, 2024
* Version: v1.0
*
*/

# include "overt_attention/overtAttention.h"

/* Global variables */

// Configuration parameters
std::string implementation_platform;
std::string camera_type;
int realignment_threshold;
int x_offset_to_head_yaw;
int y_offset_to_head_pitch;
std::string simulator_topics;
std::string robot_topics;
string topics_filename;
bool verbose_mode;

// Publisher for the velocity commands
ros::Publisher attention_velocity_publisher;

// Declare the publisher of the /overt_attention/engagement_status topic
ros::Publisher overt_attention_engagement_status_pub;

double current_secs = 0.0;                          // Stores the current time in seconds
double previous_secs = 0.0;                         // Stores the previous time in seconds

/* Callback function for the face detection data */
void face_detection_data_received(const face_detection::FaceDetectionData& data_msg){
    // AngleChange angle_change = get_angles_from_pixel(data_msg.face_centroid[0], data_msg.face_centroid[1], 640, 480, HFOV, VFOV);
    // double local_image_head_pitch = radians(angle_change.delta_pitch);
    // double local_image_head_yaw = radians(angle_change.delta_yaw);

    // // // std::cout << "Image - Head pitch: " << local_image_head_pitch << " Head yaw: " << local_image_head_yaw << std::endl;

    // // Calibrate the head pitch and yaw angles based on the local image angles
    // attention_head_pitch = local_image_head_pitch;
    // attention_head_yaw = local_image_head_yaw;
    // social_attention_done = false;                  // Set the social attention done status to false
    // face_detected = true;                           // Set the face detected status to true

    size_t message_length = data_msg.face_label.size();

    if (message_length == 0) {
        return;
    }

    // Clear the face labels, gaze_angles, attention_head_pitch and attention_head_yaw
    face_labels.clear();
    attention_head_pitch.clear();
    attention_head_yaw.clear();
    gaze_angles.clear();

    AngleChange angle_change;
    double local_image_head_pitch;
    double local_image_head_yaw;
    int image_number = 0;
    // for(size_t i = 0; i < message_length; i++){
    //     angle_change = get_angles_from_pixel(data_msg.face_centroid[image_number], data_msg.face_centroid[image_number + 1], 640, 480, HFOV, VFOV);
    //     local_image_head_pitch = radians(angle_change.delta_pitch);
    //     local_image_head_yaw = radians(angle_change.delta_yaw);

    //     // Calibrate the head pitch and yaw angles based on the local image angles
    //     attention_head_pitch = local_image_head_pitch;
    //     attention_head_yaw = local_image_head_yaw;

    //     image_number += 2;
    //     break;
    // }
    for(size_t i = 0; i < message_length; i++){
        angle_change = get_angles_from_pixel(data_msg.face_centroid[image_number], data_msg.face_centroid[image_number + 1], 640, 480, HFOV, VFOV);
        local_image_head_pitch = radians(angle_change.delta_pitch);
        local_image_head_yaw = radians(angle_change.delta_yaw);

        // Calibrate the head pitch and yaw angles based on the local image angles
        attention_head_pitch.push_back(local_image_head_pitch);
        attention_head_yaw.push_back(local_image_head_yaw);

        //store the face label
        face_labels.push_back(data_msg.face_label[i]);

        // Store the gaze angles -- uncomment after gaze angle is ready from face detection
        // gaze_angles.push_back(data_msg.gaze_angle[i]);
        gaze_angles.push_back(0.0);

        image_number += 2;
        // break;
    }
    sound_count++;
    social_attention_done = false;                  // Set the social attention done status to false
    face_detected = true;                           // Set the face detected status to true
}

/* Callback function for the sound localization topic */
void sound_localization_data_received(const std_msgs::Float64& data_msg){
    if(isnan(std::abs(data_msg.data))){                 // Check if the sound localization data is NaN
        angle_of_sound = previous_angle_of_sound;       // Set the angle of sound to the previous angle of sound
        return;
    }
    previous_angle_of_sound = angle_of_sound;           // Store the previous angle of sound
    angle_of_sound = data_msg.data;                     // Store the current angle of sound
    angle_of_sound = radians(angle_of_sound);
    
    social_attention_done = false;                      // Set the social attention done status to false
    sound_detected = true;                              // Set the sound detected status to true
}


void frontCameraMessageReceived(const sensor_msgs::ImageConstPtr& msg) {
        
    // Print the received image width and height
    // ROS_INFO("[MESSAGE] Image received has a width: %d and height: %d", msg->width, msg->height);

    cv_bridge::CvImagePtr cv_ptr;

    //  convert to BGR image
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat image = cv_ptr->image;

    if (image.empty()) {
        std::cerr << "Could not open or find the image!" << std::endl;
    }

    // Create the StaticSaliencyFineGrained object
    cv::Ptr<cv::saliency::StaticSaliencyFineGrained> saliency = cv::saliency::StaticSaliencyFineGrained::create();

    // Compute the saliency map
    cv::Mat saliencyMap;
    bool success = saliency->computeSaliency(image, saliencyMap);

    if (!success) {
        std::cerr << "Could not compute saliency!" << std::endl;
    }

    // Display the original image
    cv::imshow("Image", image);

    // Display the saliency map
    cv::imshow("Output", saliencyMap);

    // Wait for a key press indefinitely
    cv::waitKey(1);
}


void joint_states_message_received(const sensor_msgs::JointState& msg) {
    // std::vector<std::string> jointNames = {"HeadPitch", "HeadYaw"};
    auto head_pitch_iterator = std::find(msg.name.begin(), msg.name.end(), "HeadPitch");
    auto head_yaw_iterator = std::find(msg.name.begin(), msg.name.end(), "HeadYaw");

    int head_pitch_index = std::distance(msg.name.begin(), head_pitch_iterator);
    int head_yaw_index = std::distance(msg.name.begin(), head_yaw_iterator);

    head_joint_states[0] = msg.position[head_pitch_index];
    head_joint_states[1] = msg.position[head_yaw_index];
    // std::cout << "Joint state - Head pitch: " << head_joint_states[0] << " Head yaw: " << head_joint_states[1] << std::endl;
}

/* This defines the callback function for the /overtAttention/set_mode service */
bool set_mode(overt_attention::set_mode::Request  &service_request, overt_attention::set_mode::Response &service_response){
    // Extract request parameters
    string attention_system_state = service_request.state;
    double point_location_x = service_request.location_x;
    double point_location_y = service_request.location_y;
    double point_location_z = service_request.location_z;

    // Set the attention mode
    if(attention_system_state == ATTENTION_SOCIAL_STATE){
        attention_mode = ATTENTION_MODE_SOCIAL;
        service_response.mode_set_success = 1;                // Attention mode set to social successfully
    } 
    else if(attention_system_state == ATTENTION_SCANNING_STATE){
        attention_mode = ATTENTION_MODE_SCANNING;
        service_response.mode_set_success = 1;                // Attention mode set to scanning successfully
    } 
    else if(attention_system_state == ATTENTION_SEEKING_STATE){
        attention_mode = ATTENTION_MODE_SEEKING;
        service_response.mode_set_success = 1;                // Attention mode set to scanning successfully
    } 
    else if(attention_system_state == ATTENTION_LOCATION_STATE){
        // attention_mode = ATTENTION_MODE_LOCATION;
        location_x = point_location_x;
        location_y = point_location_y;
        location_z = point_location_z;
        // location_attended_to = false;                         // Reset the location attended to status
        // Read robot location input from data file
        read_robot_pose_input(robot_pose);       //Replace with subscriber to /robotLocalization/pose topic
        
        // Call the location attention function
        int location_attention_done = 0;
        location_attention_done = location_attention(location_x, location_y, location_z, topics_filename, attention_velocity_publisher, verbose_mode);
                        
        service_response.mode_set_success = location_attention_done;                // Attention mode set to location successfully
    } 
    else{
        ROS_ERROR("Invalid attention system state in service request; supported states are 'social', 'scanning', and 'location'\n");
        service_response.mode_set_success = 0;                // Attention mode set unsuccessful
    }
   
    // Print the response from the service
    ROS_INFO("Response from /overtAttention/set_mode service: [%ld]\n", (long int)service_response.mode_set_success);
    return true;
}

/* This defines the callback function for the /overtAttention/set_activation service */
bool set_activation(overt_attention::set_activation::Request  &service_request, overt_attention::set_activation::Response &service_response){
    // Extract request parameters
    string attention_system_state = service_request.state;

    // Set the system activation status
    if(attention_system_state == ATTENTION_ENABLED_STATE){
        system_activation_status = ATTENTION_SYSTEM_ENABLED;
        service_response.activation_set_success = 1;                // Attention system enabled successfully
    } 
    else if(attention_system_state == ATTENTION_DISABLED_STATE){
        system_activation_status = ATTENTION_SYSTEM_DISABLED;   
        service_response.activation_set_success = 1;                // Attention system disabled successfully
        attention_mode = ATTENTION_MODE_SEEKING;                    // Reset the attention mode
    } 
    else{
        printf("Invalid attention system state in service request; supported states are 'enabled' and 'disabled'\n");
        service_response.activation_set_success = 0;                // Attention system activation unsuccessful
    }

    // Print the response from the service
    ROS_INFO("Response from /overtAttention/set_activation service: [%ld]\n", (long int)service_response.activation_set_success);
    return true;
}

int main(int argc, char** argv) {
    // Initialize ROS
    ros::init(argc, argv, "overtAttention");
    ros::NodeHandle nh;

    ros::ServiceServer set_mode_service = nh.advertiseService("/overtAttention/set_mode", set_mode);
    ROS_INFO("Attention Subsystem Mode Server Ready\n");

    ros::ServiceServer set_activation_service = nh.advertiseService("/overtAttention/set_activation", set_activation);
    ROS_INFO("Attention Subsystem Activation Server Ready\n");

    // Create an image transport subscriber
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("naoqi_driver/camera/front/image_raw", 1000, frontCameraMessageReceived);

    // Advertise the /overtAttention/engagement_status topic
    std::string engagement_status_topic = "/overtAttention/engagement_status";
    overt_attention_engagement_status_pub = nh.advertise<std_msgs::Float64>(engagement_status_topic, 1000);
    
    // Subscribe to the joint states topic
    ros::Subscriber joint_state_subscriber = nh.subscribe("joint_states", 1000, &joint_states_message_received);

    // Create a subscriber object for faceDetection data
    string topic_name_data = "faceDetection/data";
    ros::Subscriber faces_data = nh.subscribe(topic_name_data, 1000, &face_detection_data_received);

    // Create a subscriber object for sound localization data
    string topic_name_sound = "soundDetection/direction";
    ros::Subscriber sound_data = nh.subscribe(topic_name_sound, 1000, &sound_localization_data_received);

    attention_velocity_publisher = nh.advertise<geometry_msgs::Twist>("/pepper_dcm/cmd_moveto", 1000, true);

    // Read the configuration file
    int config_file_read = 0;
    config_file_read = read_configuration_file(&implementation_platform, &camera_type, &realignment_threshold, &x_offset_to_head_yaw, &y_offset_to_head_pitch, &simulator_topics, &robot_topics, &topics_filename, &verbose_mode);
    
    // Check if the configuration file was read successfully
    if(config_file_read == 1){
        // printf("Error reading the configuration file\n");
        ROS_ERROR("Error reading the configuration file\n");
        return 0;
    }

    // Danso IMplementation
    // std::map<std::string, std::string> configMap;
    // std::map<std::string, std::string>* configMapPtr = &configMap;
    // extractConfig(*configMapPtr);


    // std::string implementation_platform = configMap.at("platform");
    // std::string camera_type;
    // int realignment_threshold = std::stoi(configMap.at("realignmentThreshold"));;
    // int x_offset_to_head_yaw = std::stoi(configMap.at("xOffsetToHeadYaw"));;
    // int y_offset_to_head_pitch = std::stoi(configMap.at("yOffsetToHeadPitch"));;
    // std::string simulator_topics = configMap.at("simulatorTopics");
    // std::string robot_topics = configMap.at("robotTopics");
    
    // bool verbose_mode;
    // if (configMap.at("verboseMode") == "true"){
    //     verbose_mode = 1;
    // } else{
    //     verbose_mode = 0;
    // }
    // string topics_filename = configMap.at("topic");
    
    
     
    bool seek_once = false;                         // Variable to check if the seek mode has been called at least once
    int attention_execution_status = 0;

    std::string head_topic;     // stores the head topic
    // Extract the topic for the head
    head_topic = extract_topic("Head", topics_filename);

    // Set to head to horizontal looking forward
    move_robot_head(head_topic, 0.0, 0.0, 1.0, verbose_mode);

    while(ros::ok()){       
        ROS_INFO_THROTTLE(10, "Overt Attention Node Running...\n");
        ros::spinOnce();                // Check for new messages on the topics
        // Check if the attention system is enabled
        if(system_activation_status == ATTENTION_SYSTEM_DISABLED){          // Attention system is disabled
            if(verbose_mode){
                ROS_ERROR_THROTTLE(10, "Attention System is currently disabled; please enable the system using the /overtAttention/set_activation service\n");
            }
        } 
        else if(system_activation_status == ATTENTION_SYSTEM_ENABLED){    // Attention system is enabled
            if(verbose_mode){
                // printf("Attention System is currently enabled\n");
            }
            /* Execute the attention mode selected in the request */
            switch(attention_mode){
                case ATTENTION_MODE_SOCIAL:         // Social attention mode
                    if(verbose_mode){
                        // printf("Attention Mode: Social\n");
                    }
                    // Call the social attention function
                    
                    if(!social_attention_done){
                        attention_execution_status = social_attention(topics_filename, realignment_threshold, attention_velocity_publisher, verbose_mode);
                        // attention_execution_status = sound_attention(topics_filename, verbose_mode);
                        // attention_execution_status = reactive_attention(topics_filename, verbose_mode);
                        social_attention_done = true;
                    }
                    break;
                case ATTENTION_MODE_SCANNING:       // Scanning attention mode
                    // Call the scanning attention function
                    attention_execution_status = scanning_attention(topics_filename, attention_velocity_publisher, verbose_mode);
                    break;
                case ATTENTION_MODE_SEEKING:       // Scanning attention mode
                    // Call the seeking attention function
                    current_secs = ros::Time::now().toSec();
                    if(current_secs - previous_secs <= ENGAGEMENT_TIMEOUT){
                        attention_execution_status = seeking_attention(topics_filename, realignment_threshold, attention_velocity_publisher, overt_attention_engagement_status_pub, verbose_mode);
                        seek_once = true;
                        break;
                    }
                    previous_secs = current_secs;
                    // Reset the attention mode to scanning and engagement status to neutral.
                    engagement_status = ENGAGEMENT_STATUS_NOT_ENGAGED;
                    if(seek_once){
                        attention_mode = ATTENTION_MODE_SCANNING;
                        seek_once = false;
                    }
                    overt_attention_engagement_status_msg.data = engagement_status;
                    overt_attention_engagement_status_pub.publish(overt_attention_engagement_status_msg);
            
                    break;
                // case ATTENTION_MODE_LOCATION:       // Location attention mode
                //     // Check if the location has already been attended to
                //     if(location_attended_to){       // If location has been attended to, do nothing, but print a message
                //         if(verbose_mode){
                //             ROS_ERROR_THROTTLE(5, "Location has already been attended to; please set a new location using the /overtAttention/set_mode service\n");
                //         }
                //         break;
                //     }
                //     else{                          // If location has not been attended to, attend to the location
                //         // Read robot location input from data file
                //         read_robot_pose_input(robot_pose);       //Replace with subscriber to /robotLocalization/pose topic
                        
                //         // Call the location attention function
                //         attention_execution_status = location_attention(location_x, location_y, location_z, topics_filename, velocity_publisher, verbose_mode);
                //         location_attended_to = true;            // Set the location attended to status to true
                //         break;
                //     }
                default:
                    if(verbose_mode){
                        ROS_ERROR_THROTTLE(3, "Invalid attention mode was set by the previous /overtAttention/set_mode service request\n");
                    }
                    break;
            }
        } 
    }
    return 0;
}
