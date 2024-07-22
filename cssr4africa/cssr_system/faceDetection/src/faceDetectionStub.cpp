#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include "face_detection/faceDetection.h"
#include "face_detection/faceDetectionData.h"
#include <fstream>

#include <boost/lexical_cast.hpp>
using boost::lexical_cast;

#include <boost/uuid/uuid.hpp>
using boost::uuids::uuid;

#include <boost/uuid/uuid_generators.hpp>
using boost::uuids::random_generator;

#include <boost/uuid/uuid_io.hpp>

string make_uuid()
{
    return lexical_cast<string>((random_generator())());
}

string filename;

/* Helper functions */
void prompt_and_exit(int status) {
   printf("Press any key to continue and close terminal ... \n");
   getchar();
   exit(status);
}

// Callback function to process the received front camera image message
void frontCameraImageMessageReceived(const sensor_msgs::ImageConstPtr& msg) {
    // Extract image attributes from the received message
    int imgWidth = msg->width;
    int imgHeight = msg->height;

    // Print the received image attributes
    // ROS_INFO("[MESSAGE] Image received has a width: %d and height: %d", imgWidth, imgHeight);
    
    cv_bridge::CvImagePtr cv_ptr;

    //  convert to BGR image
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    
    cv::Mat img = cv_ptr->image;

    // cv::Mat big_img = cv_ptr->image;
    
    cv::resize(img, img, cv::Size(640, 480));

    cv::imshow("Stub Frame", img);

    #ifdef ROS
        filename = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        filename = "..";
    #endif

    filename += "/data/detectedImages/";
    filename += make_uuid();
    filename += ".png";

    cv::imwrite(filename, img);
    cv::waitKey(30);
}

void frontCameraDataMessageReceived(const face_detection::faceDetectionData& data_msg) {
    // Print the received image attributes
    // ROS_INFO("[MESSAGE START] Data received shows below.\n");
    // ROS_INFO("Frames of faces: \n");
    // for (auto val : msg.faces){
    //     ROS_INFO_STREAM(val);
    // }
    // ROS_INFO(msg.faces);

    // ROS_INFO("Frames of eyes: \n");
    // for (auto val : msg.eyes){
    //     ROS_INFO_STREAM(val);
    // }
    // ROS_INFO(msg.eyes);

    // ROS_INFO("[MESSAGE END] Data received successfully.\n");

    string path;
    #ifdef ROS
        path = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        path = "..";
    #endif
    
    path += "/data/faceDetectionOutput.dat";
    
    std::ofstream out_of;
    out_of.open(path.c_str(), ofstream::app);
    if (!out_of.is_open()){
        printf("Unable to open the output file %s\n", path.c_str());
        prompt_and_exit(1);
    }
    out_of << "[MESSAGE START] Data received shows below.\n";
    out_of << "Frames of faces: \n";
    out_of << data_msg.face_label << endl;
    for (auto f_val : data_msg.face_centroid){
        out_of << f_val << " ";
    }
    out_of << endl;

    out_of << data_msg.face_width << endl;
    out_of << data_msg.face_height << endl;
    out_of << data_msg.confidence << endl;

    out_of << "Frames of eyes: \n";
    for (auto e_val : data_msg.left_eye_centroid){
        out_of << e_val << " ";
    }
    out_of << endl;

    out_of << data_msg.left_eye_width << endl;
    out_of << data_msg.left_eye_height << endl;

    for (auto e_val : data_msg.right_eye_centroid){
        out_of << e_val << " ";
    }
    out_of << endl;
    
    out_of << data_msg.right_eye_width << endl;
    out_of << data_msg.right_eye_height << endl;
    out_of << "[MESSAGE END] Data received successfully.\n";

    out_of.close();
    
}



int main(int argc, char **argv) {
    // printf("OpenCV: %s", cv::getBuildInformation().c_str());
    // find the respective topic
    string topic_name_img = "faceDetectionImage";

    string topic_name_data = "faceDetection/data";
   
    // Initialize the ROS node
    ros::init(argc, argv, "front_camera_stub");
    ros::NodeHandle nh;

    // Create an image transport subscriber
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber subImg = it.subscribe(topic_name_img, 1000, frontCameraImageMessageReceived);


    // Create a subscriber object for data
    ros::Subscriber subData = nh.subscribe(topic_name_data, 1000, frontCameraDataMessageReceived);

    // Enter the ROS event loop to listen for incoming messages and execute the callback function
    while(ros::ok()){
        ros::spinOnce();    
    }
    
    cv::destroyWindow("Stub Frame");

    return 0;
}