#include "face_detection/faceDetection.h"

void prompt_and_exit(int status) {
   printf("Press any key to continue and close terminal ... \n");
   getchar();
   exit(status);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "publish_images");            // Initialize the ROS system
    ros::NodeHandle nh;                                 // Become a node
    image_transport::ImageTransport img_transport(nh);  // Initialize image transport object

    string use = "photo"; // "video" or "camera" or "photo"

    Mat current_frame;
    VideoCapture camera;

    std::string media_location = ros::package::getPath(ROS_PACKAGE_NAME);
    // media_location += "/data/media/classroom.mp4";
    // media_location += "/data/media/vidasia.mp4";
    // media_location += "/data/media/vidhighfive.webm";
    // media_location += "/data/media/vidpreteen.mp4";
    // media_location += "/data/media/vidteach.mp4";
    // media_location += "/data/media/vidmultigenerational.mp4";
    // media_location += "/data/media/videnterpreneur.mp4";
    media_location += "/data/media/MSECE2019.jpg";
    // media_location += "/data/media/People2.jpg";

    if (use == "file"){
        // Read the video stream
        camera.open(media_location);
    }
    else if (use == "photo"){
        // Read the photo
        current_frame = imread(media_location);
    }
    else {
        // Initialize the camera
        camera.open(0);   // For the internal web camera
        camera.set(cv::CAP_PROP_FRAME_WIDTH, 640);  // resize to 640x480 pixel to have the same pixel size as the output of the camera of Pepper robot
        camera.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    }

    // Check that the camera is opened    
    if (camera.isOpened()) {
        camera >> current_frame;
        if (current_frame.empty())
          prompt_and_exit(1);
    }
    else if (!current_frame.empty()){
        cout<<"Image is loaded"<<endl;
    }
    else{
        cout<<"Camera is not opened"<<endl;
        prompt_and_exit(1);
    }
    cv_bridge::CvImage cv_image; // create a CvImage type object
    
    sensor_msgs::Image ros_image; // create ros image object
    // cv_image.toImageMsg(ros_image); // convert CvImage to ros Image type
    ros::Publisher pub = nh.advertise<sensor_msgs::Image>("top/image", 1);
    ros::Rate loop_rate(10);

    ROS_INFO("Publishing the local image. (Kill the node to end publishing.)");
    
    while (ros::ok()){     
        cv_image.image = current_frame; // assign the local image loaded
        cv_image.encoding = "bgr8";  // set the encoding
        cv_image.toImageMsg(ros_image); // convert CvImage to ros Image type

        pub.publish(ros_image);
        loop_rate.sleep();
        // if (use == "photo"){
        //     cout<<"[INFO] : Image is published"<<endl;
        //     break;
        // }

        // camera >> current_frame;
        if (current_frame.empty()){
        //   prompt_and_exit(1);
            cout<<"[INFO] : End of video"<<endl;
            break;
        }
    }

    return 0;
}