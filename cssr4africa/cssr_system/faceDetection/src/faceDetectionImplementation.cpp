/*
DOCUMENTATION

*/
#include "face_detection/faceDetection.h"
#include "face_detection/faceDetectionData.h" // custom message type for face detection data

// global variables
bool depth_cam = false;
std::vector <std::vector<cv::Rect>> prev_faces;
std::vector <std::vector<std::vector<cv::Rect>>> prev_eyes;

// image showing function using opencv
void displayFrame(string msg, cv::Mat frame){
    cv::imshow(msg, frame);
    cv::waitKey(1);
}

// region of interest extractor function
void roi(cv::Mat &frame, cv::Mat &roi, cv::Rect &rect){
    roi = frame(rect);
}

Tracker::Tracker() {
    // extract config file
    extractConfig(config);
    face_counter = 0;
    spatialTolerance = std::stoi(config.at("spatialTolerance"));
}

// a function to track the detected faces
void Tracker::track(std::vector<cv::Rect> &faces, std::vector<std::vector<cv::Rect>> &all_eyes, 
                    std::vector<std::vector<int>>& detection_center, CascadeClassifier& eye_cascade, 
                    Mat & gray, ros::Publisher pubDat){   
    /**
     * detections: vector of detections in a frame
     * This function will track the face by comparing 
     * the distance of the centers of the detected faces with the previous detections 
    **/
    vector<int> face_center;
    vector<int> face_width;
    vector<int> face_height;
    vector<int> left_eye_center;
    vector<int> left_eye_width;
    vector<int> left_eye_height;
    vector<int> right_eye_center;
    vector<int> right_eye_width;
    vector<int> right_eye_height;

    vector<vector<int>> list_left_eye_center;
    vector<vector<int>> list_right_eye_center;

    int center_x; 
    int center_y;

    Mat faceROI;
    vector<Rect> eyes;
    face_detection::faceDetectionData msg;


    // distance variable
    float distance;    // distance between the centers of the detected faces and the previous detections
    int min_dst;       // minimum distance
    int min_idx;       // index of the minimum distance
    bool update;       // flag to update the label of the detection
    int nearest_label; // label of the nearest detection

    // new labels
    std::vector<int> lbl;  // vector of labels
    std::vector<int> indx; // vector of indices of the detections

    lbl.resize(faces.size());
    if (faces.size() > 0){
        // row - frames, column - detections center
        for (int i=0; i < centers.size(); i++){ // for each frame in the tracker
            for (int j=0; j < centers[i].size(); j++){ // for each detection in a frame 
                min_dst = spatialTolerance;
                min_idx = -1;
                update = false;
                for (int f_idx=0; f_idx < faces.size(); f_idx++){ // for each detection in the current frame
                    center_x = (int) ((faces[f_idx].x + faces[f_idx].width) / 2);
                    center_y = (int) ((faces[f_idx].y + faces[f_idx].height) / 2);
                    
                    distance = sqrt(pow(center_x - centers[i][j][0], 2) + pow(center_y - centers[i][j][1], 2)); // calculate the distance between the centers of the detections
                    if ((distance <= spatialTolerance) && (find(indx.begin(), indx.end(), f_idx) == indx.end())){ // if the distance is less than the threshold and the detection is not assigned a label
                        if (min_dst >= distance){ // if the distance is less than the minimum distance
                            min_dst = distance;   // update the minimum distance
                            min_idx = f_idx;    // update the index of the minimum distance
                            update = true;    // set the flag to update the label of the detection
                        }
                    }
                    
                    if (i == 0 && j == 0){ // collect the needed information only once and detect eyes from each face only once
                        face_width.push_back(faces[f_idx].width);
                        face_height.push_back(faces[f_idx].height);

                        // add the x, y of a center in a vector
                        face_center.push_back(center_x);
                        face_center.push_back(center_y);

                        // add the center of a face vector in a vector
                        detection_center.push_back(face_center);

                        // clear the x, y center for the next x and y value of a face
                        face_center.clear();

                        // detect eyes
                        // find region of interest
                        roi(gray, faceROI, faces[f_idx]);
                        // displayFrame("Original", gray);
                        // displayFrame("Gray", gray);
                        // displayFrame("Face ROI", faceROI);
                        if (USE_EYE_DETECTION && config.at("algorithm") == "Haar"){
                            //In each face, detect eyes
                            // each parameter is explained as follows:
                            // 1. faceROI: the region of interest where the eyes are detected
                            // 2. eyes: the vector of detected eyes
                            // 3. 1.1: the scale factor
                            // 4. 2: the minimum number of neighbors
                            // 5. 0 | CASCADE_SCALE_IMAGE: flags
                            // 6. Size(30, 30): minimum size of the detected eyes
                            // eye_cascade.detectMultiScale(faceROI, eyes, 1.2, 2, 1 |CASCADE_SCALE_IMAGE, Size(5, 5 ));
                            // eye_cascade.detectMultiScale(gray, eyes);
                            // print the width and height of the faceROI
                            // cout << "Width: " << faceROI.cols << " Height: " << faceROI.rows << endl;
                            eye_cascade.detectMultiScale( faceROI, eyes, 1.5, 2, 0 |CASCADE_SCALE_IMAGE, Size(0, 0), Size(0, 0));
                            // print the number of eyes detected
                            cout << "Number of eyes detected: " << eyes.size() << endl;
                            if (eyes.size() == 1){
                                center_x = (int) (faces[f_idx].x + eyes[0].x + eyes[0].width / 2);
                                center_y = (int) (faces[f_idx].x + eyes[0].y + eyes[0].height / 2);

                                left_eye_center.push_back(center_x);
                                left_eye_center.push_back(center_y);

                                list_left_eye_center.push_back(left_eye_center);
                                left_eye_center.clear();

                                left_eye_width.push_back(eyes[0].width);
                                left_eye_height.push_back(eyes[0].height);

                            } else if (eyes.size() == 2){
                                center_x = (int) (faces[f_idx].x + eyes[0].x + eyes[0].width / 2);
                                center_y = (int) (faces[f_idx].x + eyes[0].y + eyes[0].height / 2);
                                left_eye_center.push_back(center_x);
                                left_eye_center.push_back(center_y);

                                list_left_eye_center.push_back(left_eye_center);
                                left_eye_center.clear();
                                
                                left_eye_width.push_back(eyes[0].width);
                                left_eye_height.push_back(eyes[0].height);

                                center_x = (int) (faces[f_idx].x + eyes[1].x + eyes[1].width / 2);
                                center_y = (int) (faces[f_idx].x + eyes[1].y + eyes[1].height / 2);

                                right_eye_center.push_back(center_x);
                                right_eye_center.push_back(center_y);

                                list_right_eye_center.push_back(right_eye_center);
                                right_eye_center.clear();

                                right_eye_width.push_back(eyes[1].width);
                                right_eye_height.push_back(eyes[1].height);
                            } else {
                                left_eye_center.push_back(-1);
                                left_eye_center.push_back(-1);

                                list_left_eye_center.push_back(left_eye_center);
                                left_eye_center.clear();

                                left_eye_width.push_back(-1);
                                left_eye_height.push_back(-1);

                                right_eye_center.push_back(-1);
                                right_eye_center.push_back(-1);

                                list_right_eye_center.push_back(right_eye_center);
                                right_eye_center.clear();

                                right_eye_width.push_back(-1);
                                right_eye_height.push_back(-1);
                            } 
                            
                            all_eyes.push_back(eyes);
                            eyes.clear();
                        }
                        else{
                        left_eye_center.push_back(-1);
                        left_eye_center.push_back(-1);

                        list_left_eye_center.push_back(left_eye_center);
                        left_eye_center.clear();

                        left_eye_width.push_back(-1);
                        left_eye_height.push_back(-1);

                        right_eye_center.push_back(-1);
                        right_eye_center.push_back(-1);

                        list_right_eye_center.push_back(right_eye_center);
                        right_eye_center.clear();

                        right_eye_width.push_back(-1);
                        right_eye_height.push_back(-1);
                        all_eyes.push_back(eyes);
                        eyes.clear();
                    }

                    }
                }
                if (update){ // if the flag is set to update the label of the detection
                    nearest_label = labels[i][j]; // get the label of the detection
                    if (find(lbl.begin(), lbl.end(), nearest_label) == lbl.end()){ // to prevent duplicate label
                        lbl[min_idx] = nearest_label; // assign the label to the detection
                        indx.push_back(min_idx);       // add the index of the detection in the vector 
                    }
                }
            }
            if (i == 0 && centers[0].size() == 0) { // if there is no detection in the frame, collect the detection center from the detected faces in order to store it in the centers vector
                for (int f_idx=0; f_idx < faces.size(); f_idx++){ // for each detection in the current frame
                    center_x = (int) ((faces[f_idx].x + faces[f_idx].width) / 2);
                    center_y = (int) ((faces[f_idx].y + faces[f_idx].height) / 2);
                    
                    face_width.push_back(faces[f_idx].width);
                    face_height.push_back(faces[f_idx].height);

                    // add the x, y of a center in a vector
                    face_center.push_back(center_x);
                    face_center.push_back(center_y);

                    // add the center of a face vector in a vector
                    detection_center.push_back(face_center);

                    // clear the x, y center for the next x and y value of a face
                    face_center.clear();

                    // detect eyes
                    // find region of interest
                    roi(gray, faceROI, faces[f_idx]);
                        
                    if (USE_EYE_DETECTION){
                        //In each face, detect eyes
                        eye_cascade.detectMultiScale( faceROI, eyes, 1.1, 2, 0 |CASCADE_SCALE_IMAGE, Size(0, 0), Size(0, 0) );
                        cout << "Number of eyes detected i = 0: " << eyes.size() << endl;
                        if (eyes.size() == 1){ // it could be left or right eye (assume left eye)
                            center_x = (int) (faces[f_idx].x + eyes[0].x + eyes[0].width / 2);
                            center_y = (int) (faces[f_idx].x + eyes[0].y + eyes[0].height / 2);

                            left_eye_center.push_back(center_x);
                            left_eye_center.push_back(center_y);

                            list_left_eye_center.push_back(left_eye_center);
                            left_eye_center.clear();

                            left_eye_width.push_back(eyes[0].width);
                            left_eye_height.push_back(eyes[0].height);

                        } else if (eyes.size() == 2){
                            center_x = (int) (faces[f_idx].x + eyes[0].x + eyes[0].width / 2);
                            center_y = (int) (faces[f_idx].x + eyes[0].y + eyes[0].height / 2);
                            left_eye_center.push_back(center_x);
                            left_eye_center.push_back(center_y);

                            list_left_eye_center.push_back(left_eye_center);
                            left_eye_center.clear();
                            
                            left_eye_width.push_back(eyes[0].width);
                            left_eye_height.push_back(eyes[0].height);

                            center_x = (int) (faces[f_idx].x + eyes[1].x + eyes[1].width / 2);
                            center_y = (int) (faces[f_idx].x + eyes[1].y + eyes[1].height / 2);

                            right_eye_center.push_back(center_x);
                            right_eye_center.push_back(center_y);

                            list_right_eye_center.push_back(right_eye_center);
                            right_eye_center.clear();

                            right_eye_width.push_back(eyes[1].width);
                            right_eye_height.push_back(eyes[1].height);
                        } else {
                            left_eye_center.push_back(-1);
                            left_eye_center.push_back(-1);

                            list_left_eye_center.push_back(left_eye_center);
                            left_eye_center.clear();

                            left_eye_width.push_back(-1);
                            left_eye_height.push_back(-1);

                            right_eye_center.push_back(-1);
                            right_eye_center.push_back(-1);

                            list_right_eye_center.push_back(right_eye_center);
                            right_eye_center.clear();

                            right_eye_width.push_back(-1);
                            right_eye_height.push_back(-1);
                        } 
                        
                        all_eyes.push_back(eyes);
                        eyes.clear();
                    }
                    else{
                        left_eye_center.push_back(-1);
                        left_eye_center.push_back(-1);

                        list_left_eye_center.push_back(left_eye_center);
                        left_eye_center.clear();

                        left_eye_width.push_back(-1);
                        left_eye_height.push_back(-1);

                        right_eye_center.push_back(-1);
                        right_eye_center.push_back(-1);

                        list_right_eye_center.push_back(right_eye_center);
                        right_eye_center.clear();

                        right_eye_width.push_back(-1);
                        right_eye_height.push_back(-1);
                        all_eyes.push_back(eyes);
                    }
                }
            } 
        }
        // assign new label for new detections and add the label in the message
        for (int k=0; k < faces.size(); k++){
            if (lbl[k] == 0){
                lbl[k] = ++face_counter;
            }
            msg.face_label = "Face " + std::to_string(lbl[k]);
            if (detection_center.size() > 0) {
                msg.face_centroid = detection_center[k];
                msg.face_width = face_width[k];
                msg.face_height = face_height[k];
                msg.left_eye_centroid = list_left_eye_center[k];
                msg.left_eye_width = left_eye_width[k];
                msg.left_eye_height = left_eye_height[k];
                msg.right_eye_centroid = list_right_eye_center[k];
                msg.right_eye_width = right_eye_width[k];
                msg.right_eye_height = right_eye_height[k];
                msg.confidence = -1; // for future compatibility
                pubDat.publish(msg); // publish the data message
            } else {
                msg.face_centroid = face_center;
                msg.face_width = -1;
                msg.face_height = -1;
                msg.left_eye_centroid = face_center;
                msg.left_eye_width = -1;
                msg.left_eye_height = -1;
                msg.right_eye_centroid = face_center;
                msg.right_eye_width = -1;
                msg.right_eye_height = -1;
                msg.confidence = -1; // for future compatibility
                pubDat.publish(msg); // publish the data message
            }
        }
        
        // add new frame in the tracker, insert the detection (centers, labels) in the vectors
        centers.insert(centers.begin(), detection_center);
        labels.insert(labels.begin(), lbl);
    } 
    else{
        // add empty frame in the tracker, insert the detection (centers, labels) in the vectors
        detection_center.resize(0, std::vector<int>(0));
        lbl.resize(0);
        centers.insert(centers.begin(), detection_center);
        labels.insert(labels.begin(), lbl);
    }

    
    // remove the last frame in the tracker
    if (centers.size() > PAST_FRAMES){
        centers.pop_back();
        labels.pop_back();
    }

    // if (DEBUG){
    //     sleep(1); // sleep for 1 second
    // }
}

// a function to draw the detected faces
void Tracker::draw(Mat &frame, std::vector<cv::Rect> & faces, std::vector<std::vector<cv::Rect>> &all_eyes, std::vector<std::vector<int>> &detections, std::vector<cv::Scalar> & colors){
    if (DEBUG){    // it will draw new faces with the previous faces
        if (USE_EYE_DETECTION){
            prev_faces.insert(prev_faces.begin(), faces);
            prev_eyes.insert(prev_eyes.begin(), all_eyes);
            if (prev_faces.size() > PAST_FRAMES){
                prev_faces.pop_back();
                prev_eyes.pop_back();
            }
            
            for (int idx =0; idx < labels.size(); idx++) { // for each frame
                auto tracks = labels[idx];
                int kdet = 0;
                
                for (auto &trk : tracks){ // for each face in a frame
                    cv::putText(frame, "Face " + std::to_string(trk), cv::Point(prev_faces[idx][kdet].x, prev_faces[idx][kdet].y - 10),
                                cv::FONT_HERSHEY_DUPLEX, 1, colors[trk % NUM_OF_COLORS], 2);
                    cv::rectangle(frame, prev_faces[idx][kdet], colors[trk % NUM_OF_COLORS], 3);
                    cv::circle(frame, cv::Point(centers[idx][kdet][0], centers[idx][kdet][1]), 3, colors[trk % NUM_OF_COLORS], -1); // draw the center of the face
                    
                    for (auto &eye : prev_eyes[idx][kdet]){ // for each detected eye of a face
                        if (eye.x == -1){continue;} // if no eye is detected, continue to the next eye
                        cv::Point eye_s( prev_faces[idx][kdet].x + eye.x, prev_faces[idx][kdet].y + eye.y);
                        cv::Point eye_e( prev_faces[idx][kdet].x + eye.x + eye.width, prev_faces[idx][kdet].y + eye.y + eye.height );
                        cv::rectangle(frame, eye_s, eye_e, cv::Scalar( colors[trk % NUM_OF_COLORS] ), 2);
                    }
                    kdet++;
                }
            }
        } else {
            prev_faces.insert(prev_faces.begin(), faces);
            if (prev_faces.size() > PAST_FRAMES){
                prev_faces.pop_back();
            }
            
            for (int idx =0; idx < labels.size(); idx++) {
                auto tracks = labels[idx];
                int kdet = 0;
                for (auto &trk : tracks){
                    cv::putText(frame, "Face " + std::to_string(trk), cv::Point(prev_faces[idx][kdet].x, prev_faces[idx][kdet].y - 10),
                                cv::FONT_HERSHEY_DUPLEX, 1, colors[trk % NUM_OF_COLORS], 2);
                    cv::rectangle(frame, prev_faces[idx][kdet], colors[trk % NUM_OF_COLORS], 3);
                    cv::circle(frame, cv::Point(centers[idx][kdet][0], centers[idx][kdet][1]), 3, colors[trk % NUM_OF_COLORS], -1); // draw the center of the face
                    kdet++;
                }
            }
        }
    } else{   // it will draw only the new faces
        if (USE_EYE_DETECTION){
            auto tracks = labels[0];    
            int kdet = 0;
            for (auto &trk : tracks){ // for each detected face
                cv::putText(frame, "Face " + std::to_string(trk), cv::Point(faces[kdet].x, faces[kdet].y - 10),
                            cv::FONT_HERSHEY_DUPLEX, 1, colors[trk % NUM_OF_COLORS], 2);
                cv::rectangle(frame, faces[kdet], colors[trk % NUM_OF_COLORS], 3);
                // cv::circle(frame, cv::Point(detections[kdet][0], detections[kdet][1]), 3, colors[trk % NUM_OF_COLORS], -1);
                if (all_eyes.size() > 0){
                    for (auto &eye : all_eyes[kdet]){ // for each detected eye of a face
                        if (eye.x == -1){continue;} // if no eye is detected, continue to the next eye
                        cv::Point eye_s(faces[kdet].x + eye.x, faces[kdet].y + eye.y);
                        cv::Point eye_e(faces[kdet].x + eye.x + eye.width, faces[kdet].y + eye.y + eye.height );
                        cv::rectangle(frame, eye_s, eye_e, cv::Scalar( colors[trk % NUM_OF_COLORS] ), 2);
                    }
                }
                kdet++;
            }
        } else {
            auto tracks = labels[0];    
            int kdet = 0;
            for (auto &trk : tracks){
                cv::putText(frame, "Face " + std::to_string(trk), cv::Point(faces[kdet].x, faces[kdet].y - 10),
                            cv::FONT_HERSHEY_DUPLEX, 1, colors[trk % NUM_OF_COLORS], 2);
                cv::rectangle(frame, faces[kdet], colors[trk % NUM_OF_COLORS], 3);
                // cv::circle(frame, cv::Point(detections[kdet][0], detections[kdet][1]), 3, colors[trk % NUM_OF_COLORS], -1);
                kdet++;
            }
        }
    }
    
}

// a function to reset a tracker object
void Tracker::reset(){
    // reset the tracker
    centers.clear();
    labels.clear();
    face_counter = 0;
}

// a function to load the haarcascade files
void loadHaarcascade(CascadeClassifier& cascade, CascadeClassifier& eye_cascade ){
    // Load Haar Cascade(s)
    std::string haar_location = ros::package::getPath(ROS_PACKAGE_NAME);;
    haar_location += "/data/haarcascades/";

    vector<CascadeClassifier> cascades;
    vector<string> cascade_files = {"haarcascade_frontalface_alt2.xml"};
    
    vector<CascadeClassifier> eyes_cascade;
    vector<string> eye_cascade_files = {"haarcascade_eye_tree_eyeglasses.xml"};
    
    int number_of_cascades = cascade_files.size();
    
    // load the face cascade files
    for (int cascade_file_no=0; (cascade_file_no < number_of_cascades); cascade_file_no++)
    {
        CascadeClassifier cascade;
        string filename(haar_location);
        filename.append(cascade_files[cascade_file_no]);
        if( !cascade.load( filename ) )
        {
            cout <<__FUNCTION__<< ": Cannot load cascade file: " << filename << endl;
            prompt_and_exit(1);
        }
        else cascades.push_back(cascade);
    }

    int number_of_eyes_cascades = eye_cascade_files.size();

    // load the eye cascade files
    for (int eye_cascade_file_no=0; (eye_cascade_file_no < number_of_eyes_cascades); eye_cascade_file_no++){
        
        CascadeClassifier eye_cascade;

        string eye_filename(haar_location);
        eye_filename.append(eye_cascade_files[eye_cascade_file_no]);

        if( !eye_cascade.load( eye_filename ) )
        {
            cout <<__FUNCTION__<< ": Cannot load eye_cascade file: " << eye_filename << endl;
            prompt_and_exit(1);
        }
        else eyes_cascade.push_back(eye_cascade);  
    }
    // select the first cascade
    cascade = cascades[HAAR_FACE_CASCADE_INDEX];
    eye_cascade = eyes_cascade[HAAR_EYE_CASCADE_INDEX];
}
int somenumber = 0;

// // CNN face detection (YuNet)
// cv::Mat faceDetectionCNN(Mat frame, Tracker& tracker, std::vector<cv::Scalar>& colors, ros::Publisher pubDat){
//     // detect faces using CNN
//     string model_path;
//     vector<Rect> faces;     // vector of faces
//     vector<vector<Rect>> all_eyes; // vector of all eyes
//     Mat gray;              // gray image for face detection
//     vector<int> faces_v;
//     vector<int> eyes_v;
    
//     vector<vector<int>> detection_center;
//     vector<int> centers;

//     // Construct the full path of the topic file
//     #ifdef ROS
//         model_path = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
//     #else
//         model_path = "..";
//     #endif
//     String fd_modelPath = model_path + "/data/face_detection_yunet_2023mar.onnx";

//     float scoreThreshold = 0.9; // confidence threshold
//     float nmsThreshold = 0.3; // suppress the overlapping boxes
//     int topK = 5000; // keep top k bounding boxes before NMS

//     bool save = false;
//     float scale = 1.0; // scale factor to resize input video frames

//     double cosine_similar_thresh = 0.363;
//     double l2norm_similar_thresh = 1.128;

//     int frameWidth, frameHeight;
//     frameWidth = frame.cols;
//     frameHeight = frame.rows;

//     // Initialize FaceDetectorYN
//     Ptr<FaceDetectorYN> detector = FaceDetectorYN::create(fd_modelPath, "", Size(320, 320), scoreThreshold, nmsThreshold, topK);
//     detector->setInputSize(Size(frameWidth, frameHeight));

//     resize(frame, frame, Size(frameWidth, frameHeight));

//     // Mat faces;
//     detector->detect(frame, faces);
//     if (USE_TRACKER){
//         // int a = 0;
//         tracker.track(faces, all_eyes, detection_center, eye_cascade, gray, pubDat);
//         tracker.draw(img_tracking, faces, all_eyes, detection_center, colors);
//     }

//     return frame;
// }

// a function to detect faces and eyes
cv::Mat faceDetectionHaar(CascadeClassifier& cascade, CascadeClassifier& eye_cascade, Mat frame, Tracker& tracker, std::vector<cv::Scalar>& colors, ros::Publisher pubDat, std::map<std::string, std::string>& configMap) {
    Mat gray;               // gray image for face detection 
    Mat img_tracking;       // image for tracking
    vector<Rect> faces;     // vector of faces
    vector<vector<Rect>> all_eyes; // vector of all eyes

    vector<int> faces_v;
    vector<int> eyes_v;
    
    vector<vector<int>> detection_center;
    vector<int> centers;

    // int center_x, center_y;
    
    // resize(frame, frame, Size(640, 480)); // resize frame to 640x480
    
    img_tracking = frame.clone();  // create a copy of the frame for tracking

    cvtColor(frame, gray, CV_BGR2GRAY ); // convert the frame to gray scale
    equalizeHist(gray, gray); // equalize the histogram of the gray scale image
    
    // detect faces
    cascade.detectMultiScale(gray, faces, 1.1, 2, cv::CASCADE_SCALE_IMAGE, Size(0, 0), Size(0, 0));
    
    // number of detected faces
    int detectedFaces = (int)faces.size();
    
    if (detectedFaces == 0){ 
        // tracker.track(faces, all_eyes, detection_center, eye_cascade, gray, pubDat); // will update the tracking with no detection
        return frame;
    }

    if (USE_TRACKER){
        // int a = 0;
        tracker.track(faces, all_eyes, detection_center, eye_cascade, gray, pubDat);
        tracker.draw(img_tracking, faces, all_eyes, detection_center, colors);
    } else{
        // draw the detected faces and eyes
        // for (int count = 0; count < (int)faces.size(); count++ ){
        //     x_value = faces[count].x;
        //     y_value = faces[count].y;
        //     width_val = faces[count].width;
        //     height_val = faces[count].height;

        //     // faces_v.push_back(x_value);
        //     // faces_v.push_back(y_value);
        //     // faces_v.push_back(width_val);
        //     // faces_v.push_back(height_val);

        //     Point origin(faces[count].x, faces[count].y);
        //     cv::putText(frame, "Face " + std::to_string(count+1), origin,
        //         FONT_HERSHEY_SIMPLEX, 0.45, colors[(count+1) % NUM_OF_COLORS], 2);
        //     rectangle(frame, faces[count], colors[(count+1) % NUM_OF_COLORS], 2);
            
            
        //     Mat faceROI = gray( faces[count] );
            
        //     vector<Rect> eyes;
        //     //In each face, detect eyes
        //     eye_cascade.detectMultiScale( faceROI, eyes, 1.1, 2, 0 |CASCADE_SCALE_IMAGE, Size(30, 30) );
        
            
        //     if (eyes.size() == 0){
        //         for (int i; i < 8; i++){
        //             eyes_v.push_back(-1);}
        //     }
        //     else if (eyes.size() == 1){
        //         eyes_v.push_back(faces[0].x + eyes[0].x);
        //         eyes_v.push_back(faces[0].y + eyes[0].y);
        //         eyes_v.push_back(eyes[0].width);
        //         eyes_v.push_back(eyes[0].height);
        //         eyes_v.push_back(-1);
        //         eyes_v.push_back(-1);
        //         eyes_v.push_back(-1);
        //         eyes_v.push_back(-1);
        //     }
        //     else if (eyes.size() == 2){
        //         for ( size_t j = 0; j < eyes.size(); j++ ){
        //             eyes_v.push_back(faces[count].x + eyes[count].x);
        //             eyes_v.push_back(faces[count].y + eyes[count].y);
        //             eyes_v.push_back(eyes[count].width);
        //             eyes_v.push_back(eyes[count].height);

        //             Point eye_s( faces[count].x + eyes[j].x, faces[count].y + eyes[j].y);
        //             Point eye_e( faces[count].x + eyes[j].x + eyes[j].width, faces[count].y + eyes[j].y + eyes[j].height );
        //             rectangle(frame, eye_s, eye_e, cv::Scalar( colorCodes[colorCounter], colorCodes[colorCounter+1], colorCodes[colorCounter+2] ), 2);
        //             // int radius = cvRound( (eyes[j].width + eyes[j].height)*0.25 );
        //             // circle( frame, eye_center, radius, cv::Scalar( colorCodes[colorCounter], colorCodes[colorCounter+1], colorCodes[colorCounter+2] ),, 4, 8, 0 );
        //         }
        //     }
        // }
        somenumber++;
        
    }
    
    return img_tracking;
    
}



/* a function to generate colors */ 
void generateColors(std::vector<cv::Scalar> &colors, int numColors){
    for (int i = 0; i < numColors; i++) {
        colors.emplace_back(cv::Scalar(rand() % 255, rand() % 255, rand() % 255));
    }
}


void extractConfig(std::map<std::string, std::string>& configMap) {
    std::string conf_file = "faceDetectionConfiguration.ini"; // configuration filename
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

/*=======================================================*/
/* Utility functions to prompt user to continue          */ 
/*=======================================================*/
// a function to prompt the user to exit the program
void prompt_and_exit(int status) {
   printf("Press any key to continue and close terminal ... \n");
   getchar();
   exit(status);
}
// a function to prompt the user to continue
void prompt_and_continue() {
   printf("Press any key to continue ... \n");
   getchar();
}