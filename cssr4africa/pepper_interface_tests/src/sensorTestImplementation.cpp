/* sensorTestImplementation.cpp
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


#include "pepper_interface_tests/sensorTest.h"

bool output = true;
int timeDuration = 10;

/* Test functions */
void backSonar(ros::NodeHandle nh){
    // find the respective topic
    string topicName = extractTopic("BackSonar");

    ROS_INFO_STREAM("Subscribing to : " << topicName << "\n" ); // Print the topic name
    ros::Duration(1).sleep();

    // Subscribe to the /pepper/sonarback topic and associate it with the callback function
    ros::Subscriber sub = nh.subscribe(topicName, 1, backSonarMessageReceived);

    // Listen for incoming messages and execute the callback function
    ros::Rate rate(30); 
    ros::Time startTime = ros::Time::now(); // start now
    ros::Duration waitTime = ros::Duration(timeDuration); 
    ros::Time endTime = startTime + waitTime;  
    while(ros::ok() && ros::Time::now() < endTime) {
        ros::spinOnce();
        rate.sleep();
    }
}

void frontSonar(ros::NodeHandle nh){
    // find the respective topic
    string topicName = extractTopic("FrontSonar");

    ROS_INFO_STREAM("Subscribing to :" << topicName << "\n"  ); // Print the topic name
    ros::Duration(1).sleep();

    // Create an image transport subscriber
    ros::Subscriber sub = nh.subscribe(topicName, 1, frontSonarMessageReceived);

    // Listen for incoming messages and execute the callback function
    ros::Rate rate(30); 
    ros::Time startTime = ros::Time::now(); // start now
    ros::Duration waitTime = ros::Duration(timeDuration);
    ros::Time endTime = startTime + waitTime;  
    while(ros::ok() && ros::Time::now() < endTime) {
        ros::spinOnce();
        rate.sleep();  
    }
    rate.sleep();
}

void frontCamera(ros::NodeHandle nh){
    // find the respective topic
    string topicName = extractTopic("FrontCamera");

    ROS_INFO_STREAM("Subscribing to :" << topicName << "\n"  ); // Print the topic name
    ros::Duration(1).sleep();

    // Create an image transport subscriber
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe(topicName, 1, frontCameraMessageReceived);

    // Listen for incoming messages and execute the callback function
    ros::Rate rate(30); 
    ros::Time startTime = ros::Time::now(); // start now
    ros::Duration waitTime = ros::Duration(timeDuration);  
    ros::Time endTime = startTime + waitTime;  
    
    while(ros::ok() && ros::Time::now() < endTime) {
        ros::spinOnce();
        rate.sleep();
    }

    cv::destroyWindow("Front Camera");
}

void bottomCamera(ros::NodeHandle nh){
    // find the respective topic
    string topicName = extractTopic("BottomCamera");

    ROS_INFO_STREAM("Subscribing to :" << topicName << "\n"  ); // Print the topic name
    ros::Duration(1).sleep();

    // Create an image transport subscriber
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe(topicName, 1, bottomCameraMessageReceived);

    // Listen for incoming messages and execute the callback function
    ros::Rate rate(30); 
    ros::Time startTime = ros::Time::now(); // start now
    ros::Duration waitTime = ros::Duration(timeDuration);  
    ros::Time endTime = startTime + waitTime;  
    while(ros::ok() && ros::Time::now() < endTime) {
        ros::spinOnce();
        rate.sleep();
    }

    cv::destroyWindow("Bottom Camera");
}

void stereoCamera(ros::NodeHandle nh){
    // find the respective topic
    string topicName = extractTopic("StereoCamera");

    ROS_INFO_STREAM("Subscribing to :" << topicName << "\n"  ); // Print the topic name
    ros::Duration(1).sleep();

    // Create an image transport subscriber
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe(topicName, 1, stereoCameraMessageReceived);

    // Listen for incoming messages and execute the callback function
    ros::Rate rate(30); 
    ros::Time startTime = ros::Time::now(); // start now
    ros::Duration waitTime = ros::Duration(timeDuration);  
    ros::Time endTime = startTime + waitTime;  
    
    while(ros::ok() && ros::Time::now() < endTime) {
        ros::spinOnce();
        rate.sleep();
    }

    cv::destroyWindow("Stereo Camera");
}

void depthCamera(ros::NodeHandle nh){
    // find the respective topic
    string topicName = extractTopic("DepthCamera");

    ROS_INFO_STREAM("Subscribing to :" << topicName << "\n"  ); // Print the topic name
    ros::Duration(1).sleep();

    // Create an image transport subscriber
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe(topicName, 1, depthCameraMessageReceived);

    // Listen for incoming messages and execute the callback function
    ros::Rate rate(30); 
    ros::Time startTime = ros::Time::now(); // start now
    ros::Duration waitTime = ros::Duration(timeDuration);  // duration of 5 seconds
    ros::Time endTime = startTime + waitTime;   // end after 5 seconds of the start time
    
    while(ros::ok() && ros::Time::now() < endTime) {
        ros::spinOnce();
        rate.sleep();
    }

    cv::destroyWindow("Depth Camera");
}

void laserSensor(ros::NodeHandle nh){
    // find the respective topic
    string topicName = extractTopic("LaserSensor");

    ROS_INFO_STREAM("Start " << topicName << " Subscribe Test \n"  ); // Print the topic name
    ros::Duration(1).sleep();
    
    ros::Subscriber sub = nh.subscribe(topicName, 1, laserSensorMessageReceived);
    
    // Listen for incoming messages and execute the callback function
    ros::Rate rate(30); 
    ros::Time startTime = ros::Time::now(); // start now
    ros::Duration waitTime = ros::Duration(timeDuration);  // duration of 5 seconds
    ros::Time endTime = startTime + waitTime;   // end after 5 seconds of the start time
    
    while(ros::ok() && ros::Time::now() < endTime) {
        ros::spinOnce();
        rate.sleep();
    }
}


/* Call back functions for each sensor test */
// Callback function to process the received sonar message
void backSonarMessageReceived(const sensor_msgs::Range& msg) {
    // Print a message indicating that sonar data is being printed
    ROS_INFO_STREAM("[MESSAGES] Printing back sonar data received.\n");

    ROS_INFO_STREAM("Frame id: " << msg.header.frame_id << "\n" );// Print the frame ID of the received message
    ROS_INFO_STREAM("Field of view: " << msg.field_of_view << "\n" ); // Print the field of view of the sonar sensor
    ROS_INFO_STREAM("Minimum range value: " << msg.min_range << "\n" ); // Print the minimum range value reported by the sonar sensor
    ROS_INFO_STREAM("Maximum range value: " << msg.max_range << "\n" ); // Print the maximum range value reported by the sonar sensor
    ROS_INFO_STREAM("Range value: " << msg.range << "\n" ); // Print the received range value reported by the sonar sensor
    ROS_INFO_STREAM("[END MESSAGES] Finished printing.\n"); // Print a message indicating the end of printing sonar data
    
    // Write the message received in an output file if the output variable is true
    if (output == true){
        string path;
        // set the main path for the output file
        #ifdef ROS
            path = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
        #else
            path = "..";
        #endif

        // complete the path of the output file
        path += "/data/sensorTestOutput.dat";
        
        // open the output file
        std::ofstream out_of;
        out_of.open(path.c_str(), ofstream::app);
        if (!out_of.is_open()){
            printf("Unable to open the output file %s\n", path.c_str());
            promptAndExit(1);
        }
        
        // write on the output file
        out_of << "[TESTING] ---- BACK SONAR ----\n\n";
        out_of << "[MESSAGES] Printing back sonar data received.\n";
        out_of << "Frame id: " << msg.header.frame_id << "\n";
        out_of << "Field of view: " << msg.field_of_view << "\n";
        out_of << "Minimum range value: " << msg.min_range << "\n";
        out_of << "Maximum range value: " << msg.max_range << "\n";
        out_of << "Range value: " << msg.range << "\n";
        out_of << "[END MESSAGES] Finished printing.\n\n";

        // close the output file
        out_of.close();

        // set the output to false so that only the first received message will be written to the output file
        output = false;
    }
}

// Callback function to process the received front sonar message
void frontSonarMessageReceived(const sensor_msgs::Range& msg) {
    // Print a message indicating that sonar data is being printed
    ROS_INFO_STREAM("[MESSAGES] Printing front sonar data received.\n");

    ROS_INFO_STREAM("Frame id: " << msg.header.frame_id << "\n" );// Print the frame ID of the received message
    ROS_INFO_STREAM("Field of view: " << msg.field_of_view << "\n" ); // Print the field of view of the sonar sensor
    ROS_INFO_STREAM("Minimum range value: " << msg.min_range << "\n" ); // Print the minimum range value reported by the sonar sensor
    ROS_INFO_STREAM("Maximum range value: " << msg.max_range << "\n" ); // Print the maximum range value reported by the sonar sensor
    ROS_INFO_STREAM("Range value: " << msg.range << "\n" ); // Print the received range value reported by the sonar sensor
    ROS_INFO_STREAM("[END MESSAGES] Finished printing.\n"); // Print a message indicating the end of printing sonar data

    // Write the message received in an output file if the output variable is true
    if (output == true){
        string path;

        // set the main path for the output file
        #ifdef ROS
            path = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
        #else
            path = "..";
        #endif
        
        // complete the path of the output file
        path += "/data/sensorTestOutput.dat";
        
        // open the output file
        std::ofstream out_of;
        out_of.open(path.c_str(), ofstream::app);
        if (!out_of.is_open()){
            printf("Unable to open the output file %s\n", path.c_str());
            promptAndExit(1);
        }
        
        // write on the output file
        out_of << "[TESTING] ---- FRONT SONAR ----\n\n";
        out_of << "[MESSAGES] Printing front sonar data received.\n";
        out_of << "Frame id: " << msg.header.frame_id << "\n";
        out_of << "Field of view: " << msg.field_of_view << "\n";
        out_of << "Minimum range value: " << msg.min_range << "\n";
        out_of << "Maximum range value: " << msg.max_range << "\n";
        out_of << "Range value: " << msg.range << "\n";
        out_of << "[END MESSAGES] Finished printing.\n\n";

        // close the output file
        out_of.close();

        // set the output to false so that only the first received message will be written to the output file
        output = false;
    }
}

// Callback function to process the received front camera image message
void frontCameraMessageReceived(const sensor_msgs::ImageConstPtr& msg) {
    // Extract image attributes from the received message
    int imgWidth = msg->width;
    int imgHeight = msg->height;

    // Print the received image attributes
    ROS_INFO("[MESSAGE] Image received has a width: %d and height: %d", imgWidth, imgHeight);
    
    // Write the message received in an output file if the output variable is true
    if (output == true){
        string path;

        // set the main path for the output file
        #ifdef ROS
            path = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
        #else
            path = "..";
        #endif
        
        // complete the path of the output file
        path += "/data/sensorTestOutput.dat";
        
        // open the output file
        std::ofstream out_of;
        out_of.open(path.c_str(), ofstream::app);
        if (!out_of.is_open()){
            printf("Unable to open the output file %s\n", path.c_str());
            promptAndExit(1);
        }

        // write on the output file
        out_of << "[TESTING] ---- FRONT CAMERA ----\n\n";
        out_of << "[MESSAGES] Printing front camera data information received.\n";
        out_of << "Image Width: " << imgWidth << "\n";
        out_of << "Image Height: " << imgHeight << "\n";
        out_of << "[END MESSAGES] Finished printing.\n\n";

        // close the output file
        out_of.close();

        // set the output to false so that only the first received message will be written to the output file
        output = false;
    }

    cv_bridge::CvImagePtr cv_ptr;

    //  convert to BGR image
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    
    cv::Mat img = cv_ptr->image;

    cv::imshow("Front Camera", img);

    cv::waitKey(30);
}

// Callback function to process the received bottom camera image message
void bottomCameraMessageReceived(const sensor_msgs::ImageConstPtr& msg) {
    // Extract image attributes from the received message
    int imgWidth = msg->width;
    int imgHeight = msg->height;

    // Print the received image attributes
    ROS_INFO("[MESSAGE] Image received has a width: %d and height: %d", imgWidth, imgHeight);

    // Write the message received in an output file if the output variable is true
    if (output == true){
        string path;
        // set the main path for the output file
        #ifdef ROS
            path = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
        #else
            path = "..";
        #endif
        
        // complete the path of the output file
        path += "/data/sensorTestOutput.dat";
        
        // open the output file
        std::ofstream out_of;
        out_of.open(path.c_str(), ofstream::app);
        if (!out_of.is_open()){
            printf("Unable to open the output file %s\n", path.c_str());
            promptAndExit(1);
        }
        
        // write on the output file
        out_of << "[TESTING] ---- BOTTOM CAMERA ----\n\n";
        out_of << "[MESSAGES] Printing bottom camera data information received.\n";
        out_of << "Image Width: " << imgWidth << "\n";
        out_of << "Image Height: " << imgHeight << "\n";
        out_of << "[END MESSAGES] Finished printing.\n\n";

        // close the output file
        out_of.close();

        // set the output to false so that only the first received message will be written to the output file
        output = false;
    }

    // create an image pointer
    cv_bridge::CvImagePtr cv_ptr;

    try{
        //  convert to BGR image
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch(const cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    cv::Mat img = cv_ptr->image;

    cv::imshow("Bottom Camera", img);

    cv::waitKey(30);
}

// Callback function to process the received stereo camera image message
void stereoCameraMessageReceived(const sensor_msgs::ImageConstPtr& msg) {
    // Extract image attributes from the received message
    int imgWidth = msg->width;
    int imgHeight = msg->height;

    // Print the received image attributes
    ROS_INFO("[MESSAGE] Image received has a width: %d and height: %d", imgWidth, imgHeight);

    // Write the message received in an output file if the output variable is true
    if (output == true){
        string path;
        // set the main path for the output file
        #ifdef ROS
            path = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
        #else
            path = "..";
        #endif
        
        // complete the path of the output file
        path += "/data/sensorTestOutput.dat";
        
        // open the output file
        std::ofstream out_of;
        out_of.open(path.c_str(), ofstream::app);
        if (!out_of.is_open()){
            printf("Unable to open the output file %s\n", path.c_str());
            promptAndExit(1);
        }

        // write on the output file
        out_of << "[TESTING] ---- STEREO CAMERA ----\n\n";
        out_of << "[MESSAGES] Printing stereo camera data information received.\n";
        out_of << "Image Width: " << imgWidth << "\n";
        out_of << "Image Height: " << imgHeight << "\n";
        out_of << "[END MESSAGES] Finished printing.\n\n";

        // close the output file
        out_of.close();

        // set the output to false so that only the first received message will be written to the output file
        output = false;
    }

    // create an image pointer
    cv_bridge::CvImagePtr cv_ptr;

    try{
        //  convert to BGR image
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch(const cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    cv::Mat img = cv_ptr->image;

    cv::imshow("Stereo Camera", img);

    cv::waitKey(30);
}

// Callback function to process the received depth camera image message
void depthCameraMessageReceived(const sensor_msgs::ImageConstPtr& msg) {
    // Extract image attributes from the received message
    int imgWidth = msg->width;
    int imgHeight = msg->height;

    // Print the received image attributes
    ROS_INFO("[MESSAGE] Image received has a width: %d and height: %d", imgWidth, imgHeight);

    // Write the message received in an output file if the output variable is true
    if (output == true){
        string path;
        // set the main path for the output file
        #ifdef ROS
            path = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
        #else
            path = "..";
        #endif
        
        // complete the path of the output file
        path += "/data/sensorTestOutput.dat";
        
        // open the output file
        std::ofstream out_of;
        out_of.open(path.c_str(), ofstream::app);
        if (!out_of.is_open()){
            printf("Unable to open the output file %s\n", path.c_str());
            promptAndExit(1);
        }

        // write on the output file
        out_of << "[TESTING] ---- DEPTH CAMERA ----\n\n";
        out_of << "[MESSAGES] Printing depth camera data information received.\n";
        out_of << "Image Width: " << imgWidth << "\n";
        out_of << "Image Height: " << imgHeight << "\n";
        out_of << "[END MESSAGES] Finished printing.\n\n";

        // close the output file
        out_of.close();

        // set the output to false so that only the first received message will be written to the output file
        output = false;
    }
    cv_bridge::CvImagePtr cv_ptr;

    try{
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch(const cv_bridge::Exception& e){
        ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
    }

    //Copy the image.data to imageBuf.
    cv::Mat img = cv_ptr->image;

    double min = 0;

    double max = 1000;

    cv::Mat img_scaled_8u;
    cv::Mat color_img;
    

    cv::Mat(cv_ptr->image-min).convertTo(img_scaled_8u, CV_8UC1, 255. / (max - min));

    if(img_scaled_8u.type() ==  CV_8UC1){
       cv::cvtColor(img_scaled_8u, color_img, CV_GRAY2RGB); 
    }

    cv::imshow("Depth Camera", color_img);

    cv::waitKey(30);
}

// Callback function to process the received laser sensor message
void laserSensorMessageReceived(const sensor_msgs::LaserScan& msg) {

    ROS_INFO_STREAM("[MESSAGES] Printing laser sensor data received.\n");
    // Print the received message attributes
    ROS_INFO_STREAM("Frame id: " << msg.header.frame_id << "\n" );
    ROS_INFO_STREAM("Start angle of the scan: " << msg.angle_min << "\n" );
    ROS_INFO_STREAM("End angle of the scan: " << msg.angle_max << "\n" );
    ROS_INFO_STREAM("Angular distance between measurements: " << msg.angle_increment << "\n" );
    ROS_INFO_STREAM("Time between measurements: " << msg.time_increment << "\n" );
    ROS_INFO_STREAM("Time between scans: " << msg.scan_time << "\n" );
    ROS_INFO_STREAM("Minimum range value: " << msg.range_min << "\n" );
    ROS_INFO_STREAM("Maximum range value: " << msg.range_max << "\n" );
    ROS_INFO_STREAM("Range data: (size: " << msg.ranges.size() << ") \n" );
            
    for (auto rng : msg.ranges){
        ROS_INFO_STREAM(rng);
    }
    ROS_INFO_STREAM("\n");

    ROS_INFO_STREAM("[END MESSAGES] Finished printing.\n");

    // Write the message received in an output file if the output variable is true
    if (output == true){
        string path;
        // set the main path for the output file
        #ifdef ROS
            path = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
        #else
            path = "..";
        #endif
        
        // complete the path of the output file
        path += "/data/sensorTestOutput.dat";
        
        // open the output file
        std::ofstream out_of;
        out_of.open(path.c_str(), ofstream::app);
        if (!out_of.is_open()){
            printf("Unable to open the output file %s\n", path.c_str());
            promptAndExit(1);
        }

        // write on the output file
        out_of << "[TESTING] ---- LASER SENSOR ----\n\n";
        out_of << "[MESSAGES] Printing laser sensor data received.\n";
        out_of << "Frame id: " << msg.header.frame_id << "\n" ;
        out_of << "Start angle of the scan: " << msg.angle_min << "\n" ;
        out_of << "End angle of the scan: " << msg.angle_max << "\n" ;
        out_of << "Angular distance between measurements: " << msg.angle_increment << "\n" ;
        out_of << "Time between measurements: " << msg.time_increment << "\n" ;
        out_of << "Time between scans: " << msg.scan_time << "\n" ;
        out_of << "Minimum range value: " << msg.range_min << "\n" ;
        out_of << "Maximum range value: " << msg.range_max << "\n" ;
        out_of << "Range data: (size: " << msg.ranges.size() << ") \n" ;
        out_of << "[END MESSAGES] Finished printing.\n\n";
        
        // close the output file
        out_of.close();

        // set the output to false so that only the first received message will be written to the output file
        output = false;
    }
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

/* Extract topic names for the respective simulator or physical robot */
string extractTopic(string key){
    bool debug = false;   // used to turn debug message on
    
    std::string configFileName = "actuatorTestConfiguration.ini";  // configuration filename
    std::string configPath;                                  // configuration path
    std::string configPathFile;                         // configuration path and filename
    
    std::string platformKey = "platform";                     // platform key 
    std::string robotTopicKey = "robotTopics";                // robot topic key
    std::string simulatorTopicKey = "simulatorTopics";        // simulator topic key

    std::string platformValue;                                // platform value
    std::string robotTopicValue;                              // robot topic value
    std::string simulatorTopicValue;                          // simulator topic value
    
    std::string topicFileName;                                   // topic filename
    std::string topicPath;                                   // topic filename path
    std::string topicPathFile;                          // topic with path and file 

    std::string topic_value = "";                             // topic value

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
        
        if (paramKey == platformKey){ platformValue = paramValue;}
        
        else if (paramKey == robotTopicKey){ robotTopicValue = paramValue;}

        else if (paramKey == simulatorTopicKey){ simulatorTopicValue = paramValue;}
    }
    configFile.close();

    // set the topic file based on the config extracted above
    if (platformValue == "simulator") { topicFileName = simulatorTopicValue; }
    else if (platformValue == "robot") { topicFileName = robotTopicValue; }
    
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

/* Extract the expected tests to run for the respective actuator or sensor tests */
std::vector<std::string> extractTests(std::string set){
    bool debug = false;   // used to turn debug message on
    
    std::string inputFileName;                                  // input filename
    std::string inputPath;                                  // input path
    std::string inputPathFile;                         // input path and filename
    
    std::vector<std::string> testName;
    std::string flag;

    if (set == "actuator"){
        inputFileName = "actuatorTestInput.ini";
    }
    else{
        inputFileName = "sensorTestInput.ini";
    }

    // Construct the full path of the input file
    #ifdef ROS
        inputPath = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        inputPath = "..";
    #endif

    // set input path
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
    
    std::string paramKey, paramValue; // variables to keep the key value pairs read

    // Get key-value pairs from the input file
    while(std::getline(inputFile, inpLineRead)){
        std::istringstream iss(inpLineRead);
    
        iss >> paramKey;
        trim(paramKey);
        std::getline(iss, paramValue);
        iss >> paramValue;
        
        trim(paramValue); // trim whitespace
        transform(paramValue.begin(), paramValue.end(), paramValue.begin(), ::tolower); // convert to lower case
        transform(paramKey.begin(), paramKey.end(), paramKey.begin(), ::tolower); // convert to lower case

        if (paramValue == "true"){ testName.push_back(paramKey);}
    }
    inputFile.close();

    return testName;
}

// Extract the mode to run the tests
std::string extractMode(){
    bool debug = false;   // used to turn debug message on

    std::string configFileName = "sensorTestConfiguration.ini";  // configuration filename
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