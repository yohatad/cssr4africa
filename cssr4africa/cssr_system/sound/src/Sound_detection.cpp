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

void audiocallback(const naoqi_driver::AudioCustomMsg& msg) {

    std::string currentChannel = "frontLeft";
    const std::vector<int16_t>* channelData = nullptr;

    if (currentChannel == "frontLeft") {
        channelData = &msg.frontLeft;
    } else if (currentChannel == "frontRight") {
        channelData = &msg.frontRight;
    } else if (currentChannel == "rearLeft") {
        channelData = &msg.rearLeft;
    } else if (currentChannel == "rearRight") {
        channelData = &msg.rearRight;
    }

    // print channelData
    for (int i = 0; i < channelData->size(); i++) {
        std::cout << channelData->at(i) << std::endl;
    }
    

    std::cout << "Received audio message" << std::endl;
}


int main (int argc, char *argv[]){
    ros::init (argc, argv, "sound_localization");
    ros::NodeHandle nh;

    // Subscribers
    ros::Subscriber sub1 = nh.subscribe("/naoqi_driver/audio", 1000, audiocallback);

    ros::spin();
    return 0;
}


    
    // // Constants
    // const float speed_of_sound = 346;
    // const float distance_between_ears = 0.09;
    // const float sampling_rate = 48000;
    // const unsigned long n = 16384;
    
    // float ans[2 * n];                       // Cross-correlation function
    // float sum_square_leftSound = 0;
    // float sum_square_rightSound = 0;
    
    // // File paths
    // int num_samples = std::min(inputFileInfo.frames, inputFileInfo2.frames);
    // float data1[num_samples];
    // float data2[num_samples];

    // sf_read_float(inputFile1, data1, num_samples);
    // sf_read_float(inputFile2, data2, num_samples);

    // // Apply Energy Thresholding
    // for (int i = 0; i < num_samples; ++i) {
    //     sum_square_leftSound += data1[i] * data1[i];
    //     sum_square_rightSound += data2[i] * data2[i];
    // }

    // float threshold = 1000;
    // if (sum_square_leftSound < threshold || sum_square_rightSound < threshold) {
    //     std::cout << "azimuth: 0 degree (Front)" << std::endl;
    //     return 0;
    // }

    // }   

    // correl(data1 - 1, data2 - 1, n, ans);

    // float max = ans[1];
    // int location = 1;
    // for (int i = 2; i <= n; i++) {
    //     if (max < ans[i]) {
    //         max = ans[i];
    //         location = i;
    //     }
    // }

    // if (location != 1) {
    //     num_samples = (location >= n / 2 + 1) ? (n + 1) - location : location - 1;
    //     float ITD = num_samples / sampling_rate;
    //     float z = ITD * (speed_of_sound / distance_between_ears);
    //     double azimuth = std::asin(z) * (180.0 / M_PI);

    //     std::cout << "azimuth: "<<azimuth << " degree " << ((location >= n / 2 + 1) ? "(Left)" : "(Right)");
    //     std::cout << std::endl;
    // }
    // else {
    //     std::cout << "azimuth: 0 degree (Front)" << std::endl;
    // }


    // return 0;

