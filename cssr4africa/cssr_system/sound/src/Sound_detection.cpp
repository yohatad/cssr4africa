/************************************************************************************************
*  
* This code implement Interaural Time Difference (ITD) algorithm to detect the direction of the sound source
* by reading two audio file one is the original audio file and the other is the delayed audio file. 
* 
*
*
*
*************************************************************************************************/

#include "util.h"
#include "sound.h"
#include <vector>
#include <ros/ros.h>

const float speed_of_sound = 346;
const float distance_between_ears = 0.07;
const float sampling_rate = 48000;
const int bufferSize = 4096;

void Itd(float data1[], float data2[]){
    
    float ans[2 * bufferSize];
    float sum_square_leftSound = 0;
    float sum_square_rightSound = 0;

    // Apply Energy Thresholding
    for (int i = 0; i < bufferSize; i++) {
        sum_square_leftSound += data1[i] * data1[i];
        sum_square_rightSound += data2[i] * data2[i];
    }

    float threshold = 1000;
    if (sum_square_leftSound < threshold || sum_square_rightSound < threshold) {
        std::cout << "azimuth: 0 degree (Front)" << std::endl;
        return;
    }

    // correl(data1, data2, bufferSize, ans);
    correl(data1 - 1, data2 - 1, bufferSize, ans);

    float max = ans[1];
    int location = 1;
    
    for (int i = 2; i <= bufferSize; i++) {
        if (max < ans[i]) {
            max = ans[i];
            location = i;
        }
    }

    if (location != 1) {
        int num_samples = (location >= bufferSize / 2 + 1) ? (bufferSize + 1) - location : location - 1;
        float ITD = num_samples / sampling_rate;
        float z = ITD * (speed_of_sound / distance_between_ears);
        double azimuth = std::asin(z) * (180.0 / M_PI);

        std::cout << "azimuth: "<<azimuth << " degree " << ((location >= bufferSize / 2 + 1) ? "(Left)" : "(Right)");
        std::cout << std::endl;
    }
    else {
        std::cout << "azimuth: 0 degree (Front)" << std::endl;
    }
}
void audiocallback(const naoqi_driver::AudioCustomMsg& msg) {

    const std::vector<int16_t>* frontLeft = nullptr;
    const std::vector<int16_t>* frontRight = nullptr;
    
    frontLeft   = &msg.frontLeft;
    frontRight  = &msg.frontRight;

    float data1[bufferSize];
    float data2[bufferSize];

    for (int i = 0; i < bufferSize; i++) {
        data1[i] = frontLeft->at(i);
        data2[i] = frontRight->at(i);
    }

    // perform ITD
    Itd(data1, data2);
}

int main (int argc, char *argv[]){
    ros::init (argc, argv, "sound_localization");
    ros::NodeHandle nh;

    // Subscribers
    ros::Subscriber sub1 = nh.subscribe("/naoqi_driver/audio", 1000, audiocallback);

    ros::spin();
    return 0;
}

