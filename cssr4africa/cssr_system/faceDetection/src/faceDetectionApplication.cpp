/* 
  Example use of openCV to perform face detection using Haar features and boosted classification
  ----------------------------------------------------------------------------------------------
  
  (This is the implementation file: it contains the code for dedicated functions to implement the application.
  These functions are called by client code in the application file. The functions are declared in the interface file.) 

  David Vernon
  24 November 2017

  Ported to Ubuntu 16.04 and OpenCV 3.3
  David Vernon
  1 November 2022

  Read topics from config files
  Merge face and eye detection
  Color code the bounding box of the detected elements
  Publish the image and data on their respective topics
  Mihiretab Taye
  August 18, 2023

  
*/

#include "face_detection/faceDetection.h"
#include "face_detection/faceDetectionData.h" // custom message type for face detection data


// global variables to be used in the callback function
// A map to store the configuration parameters
std::map<std::string, std::string> configMap;
std::map<std::string, std::string>* configMapPtr = &configMap;

// create custom tracker
Tracker tracker;

Mat current_frame;
vector<face_detection::faceDetectionData> data_msg;

std::vector<int> rejectLevels;
std::vector<double> levelWeights;


std::vector<cv::Scalar> colors;

bool onlyOnce = true;

ros::Publisher pubImg;

ros::Publisher pubDat;

sensor_msgs::Image ros_image; // create ros image object

cv_bridge::CvImage cv_image; // create a CvImage type object

// Load haarcascade for eye and face
CascadeClassifier cascade;
CascadeClassifier eye_cascade;



// Callback function to process the received front camera image message
void frontCameraMessageReceived(const sensor_msgs::ImageConstPtr& msg) {
        
    if (onlyOnce){
        // Print the received image width and height
        ROS_INFO("[MESSAGE] Image received has a width: %d and height: %d", msg->width, msg->height);
        onlyOnce = false;
    }

    cv_bridge::CvImagePtr cv_ptr;

    //  convert to BGR image
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    
    cv::Mat img = cv_ptr->image;
    
    // display image
    // displayFrame("Subscribed Image Received", img);
    
    // tmeter.start();
    
    // Select the algorithm to use
    // if (configMap["algorithm"] == "Haar") {
    //     // detect faces and eyes
    //     current_frame = faceDetectionHaar(cascade, eye_cascade, img, tracker, colors, pubDat);
    // } else if (configMap["algorithm"] == "CNN") {
    //     // detect faces and eyes
    //     current_frame = faceDetectionCNN(img, tracker, colors, pubDat);
    // } else if (configMap["algorithm"] == "YOLO") {
    //     // detect faces and eyes
    //     current_frame = faceDetectionYOLO(img, tracker, colors, pubDat);
    // } else {
    //     ROS_INFO("[ERROR] Algorithm not supported");
    // }

    current_frame = faceDetectionHaar(cascade, eye_cascade, img, tracker, colors, pubDat, configMap);
    cv_image.image = current_frame; // assign the local image loaded
    cv_image.encoding = "bgr8";  // set the encoding
    cv_image.toImageMsg(ros_image); // convert CvImage to ros Image type

    // publish the image
    pubImg.publish(ros_image);
    
    // if verboseMode is enabled, display the frame
    if (configMap["verboseMode"] == "true"){
        displayFrame("Face Detection", current_frame);
    }
    // tmeter.stop();
    // double fps = tmeter.getFPS();
    // std::string fpsString = cv::format("FPS : %.2f", (float)fps);
    
}

int main(int argc, char **argv) {
    // extract the configuration parameters
    extractConfig(*configMapPtr);

    // get the topic to subscribe to
    string topic_name = configMap["topic"];
    
    // Initialize the ROS node
    ros::init(argc, argv, "faceDetectionApplication");
    ros::NodeHandle nh;

    pubImg = nh.advertise<sensor_msgs::Image>("faceDetectionImage", 1000);
    pubDat = nh.advertise<face_detection::faceDetectionData>("faceDetection/data", 1000);

    // Create an image transport subscriber
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe(topic_name, 1000, frontCameraMessageReceived);

    // if (depth) {cv_image.encoding = "";}
    // else {cv_image.encoding = "bgr8";}  // set the encoding
 
    ros::Rate loop_rate(10); 
    
    if (configMap["algorithm"] == "Haar") {
        // Load haarcascade for face and eye
        loadHaarcascade(cascade, eye_cascade);
    } 
    // else if (configMap["algorithm"] == "CNN") {
    //     // Load CNN model
    //     loadCNN();
    // } else if (configMap["algorithm"] == "YOLO") {
    //     // Load YOLO model
    //     loadYOLO();
    // } else {
    //     ROS_INFO("[ERROR] Algorithm not supported");
    // }
    
    // generate colors
    generateColors(colors, NUM_OF_COLORS);

    ros::spin();
    return 0;
}




#ifdef ROS
/**
 Linux (POSIX) implementation of _kbhit().
 Morgan McGuire, morgan@cs.brown.edu
 */
int _kbhit() {
    static const int STDIN = 0;
    static bool initialized = false;

    if (! initialized) {
        // Use termios to turn off line buffering
        termios term;
        tcgetattr(STDIN, &term);
        term.c_lflag &= ~ICANON;
        tcsetattr(STDIN, TCSANOW, &term);
        setbuf(stdin, NULL);
        initialized = true;
    }

    int bytesWaiting;
    ioctl(STDIN, FIONREAD, &bytesWaiting);
    return bytesWaiting;
}
#endif
