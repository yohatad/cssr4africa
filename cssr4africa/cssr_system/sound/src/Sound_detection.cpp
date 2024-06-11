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

const int windowSize = 3;  // Number of values to consider for the mode
std::vector<double> itd_values;  // Vector to store ITD values

double itd_value;
bool value_received = false;

double prev_sum_square_leftSound = 0;
double prev_sum_square_rightSound = 0; 

double Itd(float data1[], float data2[]){
    
    float ans[2 * bufferSize];
    float sum_square_leftSound = 0;
    float sum_square_rightSound = 0;

    // Apply Energy Thresholding
    for (int i = 0; i < bufferSize; i++) {
        sum_square_leftSound += data1[i] * data1[i];
        sum_square_rightSound += data2[i] * data2[i];
    }   

    float leftThreshold = 2.66608e+07;
    float rightThreshold = 4.77235e+07;

    if (sum_square_leftSound < leftThreshold || sum_square_rightSound < rightThreshold) {
        return 0;
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

        if (location >= bufferSize / 2 + 1) {
            return -azimuth;
        }
        else {
            return azimuth;
        }
    }
    else {
        return 0;
    }
}

float calculate_rms(const std::vector<int16_t>& data, int bufferSize) {
    float sum = 0.0;
    for (int i = 0; i < bufferSize; ++i) {
        sum += data[i] * data[i];
    }
    return std::sqrt(sum / bufferSize);
}

void audiocallback(const naoqi_driver::AudioCustomMsg& msg) {
    const std::vector<int16_t>* frontLeft   =  &msg.frontLeft;
    const std::vector<int16_t>* frontRight  =  &msg.frontRight;
    const std::vector<int16_t>* rearLeft    =  &msg.rearLeft;
    const std::vector<int16_t>* rearRight   =  &msg.rearRight;

    int bufferSize = frontLeft->size();  // Assuming all channels have the same buffer size

    float data1[bufferSize];
    float data2[bufferSize];
    float data3[bufferSize];
    float data4[bufferSize];

    for (int i = 0; i < bufferSize; i++) {
        data1[i] = frontLeft->at(i);
        data2[i] = frontRight->at(i);
        data3[i] = rearLeft->at(i);
        data4[i] = rearRight->at(i);
    }

    // Calculate RMS of the front and rear microphones
    float rmsFrontLeft  = calculate_rms(*frontLeft, bufferSize);
    float rmsFrontRight = calculate_rms(*frontRight, bufferSize);
    float rmsRearLeft   = calculate_rms(*rearLeft, bufferSize);
    float rmsRearRight  = calculate_rms(*rearRight, bufferSize);

    // Average RMS for front and rear
    float rmsFront = (rmsFrontLeft + rmsFrontRight) / 2.0;
    float rmsRear  = (rmsRearLeft + rmsRearRight) / 2.0;

    // Check if sound source is from the front
    if (rmsFront > rmsRear) {
        double value = Itd(data1, data2);
        itd_values.push_back(value);
        value_received = true;  // Set flag to indicate value has been updated
    } else {
        std::cout << "Sound is coming from the back" << std::endl;
    }
}

double calculateMode(const std::vector<double>& values) {
    std::map<double, int> frequency_map;
    for (double value : values) {
        frequency_map[value]++;
    }

    double mode = values[0];
    int max_count = 0;
    for (const auto& pair : frequency_map) {
        if (pair.second > max_count) {
            max_count = pair.second;
            mode = pair.first;
        }
    }
    return mode;
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "sound_localization");
    ros::NodeHandle nh;

    // Subscribers
    ros::Subscriber sub1 = nh.subscribe("/naoqi_driver/audio", 1000, audiocallback);

    while (ros::ok()) {
        ros::spinOnce();

        if (value_received && itd_values.size() >= windowSize) {
            std::vector<double> last_ten_values(itd_values.end() - windowSize, itd_values.end());
            double mode = calculateMode(last_ten_values);
            ROS_INFO("Mode of last 10 ITD values: %f", mode);

            // Clear the vector for the next batch
            itd_values.clear();

            // Reset flag
            value_received = false;
        }

        // Add a small sleep to avoid busy-waiting
        ros::Duration(0.1).sleep();
    }
    return 0;
}

