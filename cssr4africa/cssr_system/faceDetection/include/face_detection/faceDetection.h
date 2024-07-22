/* 
  Example use of openCV to perform face detection using Haar features and boosted classification
  ----------------------------------------------------------------------------------------------
 
  (This is the interface file: it contains the declarations of dedicated functions to implement the application.
  These function are called by client code in the application file. The functions are defined in the implementation file.)

  David Vernon
  24 November 2017
*/
 
#define GCC_COMPILER (defined(__GNUC__) && !defined(__clang__))

#if GCC_COMPILER
   #ifndef ROS
       #define ROS
   #endif
   #ifndef ROS_PACKAGE_NAME
      #define ROS_PACKAGE_NAME "faceDetection"
   #endif
#endif

#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"
#include <ctype.h>
#include <iostream>
#include <string>
#include <image_transport/image_transport.h>
#include <random>
#include <fstream>
#include <regex>
#include "face_detection/faceDetectionData.h" // custom message type for face detection data

#ifndef ROS
   #include <conio.h>
#else
   #include <sys/select.h>
   #include <termios.h>
   #if HAVE_STROPTS_H
      #include <stropts.h>
   #endif

   // #include <stropts.h>
   #include <sys/ioctl.h>
#endif
    
//opencv
// #include <cv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h> 
#include <boost/algorithm/string.hpp>

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>
 

using namespace boost;
using namespace std;
using namespace cv;


#define TRUE  1
#define FALSE 0
#define MAX_STRING_LENGTH 200 
#define MAX_FILENAME_LENGTH 200 
#define HAAR_FACE_CASCADE_INDEX 0
#define HAAR_EYE_CASCADE_INDEX 0
#define NUM_OF_COLORS 25
#define DEBUG 0 // 1
#define PAST_FRAMES 10
#define USE_TRACKER 1
#define USE_EYE_DETECTION 0
/* function prototypes go here */


class Tracker {
   private:
      int face_counter; // number of faces
      int spatialTolerance; // threshold for the face to be considered as a new face
      std::vector<std::vector<std::vector<int>>> centers; // center of the face
      std::vector<std::vector<int>> labels; // labels of the face
      std::map<std::string, std::string> config; // configuration map
   public:
      Tracker();  // constructor
      // Tracker(const std::map<std::string, std::string>& config);  // constructor
      void track(std::vector<cv::Rect> &faces, std::vector<std::vector<cv::Rect>> &all_eyes, std::vector<std::vector<int>> &centers, CascadeClassifier& eye_cascade, Mat & gray, ros::Publisher pubDat); // track the face
      void draw(Mat &frame, std::vector<cv::Rect> &faces, std::vector<std::vector<cv::Rect>> &all_eyes, std::vector<std::vector<int>> &centers, std::vector<cv::Scalar> &colors); // draw the face
      void reset(); // reset the tracker
      std::vector<std::vector<int>> getLabels(); // get the labels of the face
};


cv::Mat faceDetectionHaar(CascadeClassifier& cascade, CascadeClassifier& eye_cascade, Mat current_frame, Tracker & tracker, std::vector<cv::Scalar> &colors, ros::Publisher pubDat, std::map<std::string, std::string>& configMap); 
cv::Mat faceDetectionCNN(Mat current_frame, Tracker & tracker, std::vector<cv::Scalar> &colors, ros::Publisher pubDat);

void loadHaarcascade(CascadeClassifier& cascade, CascadeClassifier& eye_cascade);
void generateColors(std::vector<Scalar> &colors, int numOfColors);
void prompt_and_exit(int status);
void prompt_and_continue();
string extract_topic();
void extractConfig(std::map<std::string, std::string>& configMap);
void displayFrame(string msg, cv::Mat frame);

#ifdef ROS
   // ncurses.h must be included after opencv2/opencv.hpp to avoid incompatibility
//    #include <ncurses.h>
  
   #include <ros/ros.h>
   #include <ros/package.h>
#endif 
    
#ifdef ROS
   int _kbhit();
#endif
