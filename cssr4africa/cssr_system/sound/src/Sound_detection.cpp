/************************************************************************************************
*  
* This code implements the Interaural Time Difference (ITD) algorithm to detect the direction of the 
* sound source by reading audio from four microphones arranged in a square pattern.
* 
*************************************************************************************************/

#include "util.h"
#include "sound.h"
#include <vector>
#include <ros/ros.h>
#include <map>
#include <cmath>
#include <algorithm>
#include <numeric>  // For std::inner_product

const float SPEED_OF_SOUND = 346.0f;
const float DISTANCE_BETWEEN_EARS = 0.07f;
const float SAMPLING_RATE = 48000.0f;
const int BUFFER_SIZE = 4096;
const int WINDOW_SIZE = 3;  // Number of values to consider for the mode
const float INTENSITY_THRESHOLD = 250;

std::vector<double> angle_values;  // Vector to store ITD angle values

double calculateItd(const float* data1, const float* data2, int size);
float calculateRms(const std::vector<int16_t>& data);
void audioCallback(const naoqi_driver::AudioCustomMsg& msg);
double calculateMode(const std::vector<double>& values);

double calculateItd(const float* data1, const float* data2, int size) {
    std::vector<float> ans(2 * size, 0.0f);
    correl(data1 - 1, data2 - 1, size, ans.data());

    auto max_it = std::max_element(ans.begin() + 1, ans.begin() + size + 1);
    int location = std::distance(ans.begin(), max_it);

    if (location != 1) {
        int num_samples = (location >= size / 2 + 1) ? (size + 1 - location) : (location - 1);
        float ITD = num_samples / SAMPLING_RATE;
        float z = ITD * (SPEED_OF_SOUND / DISTANCE_BETWEEN_EARS);
        double angle = std::asin(z) * (180.0 / M_PI);

        return (location >= size / 2 + 1) ? -angle : angle;
    }
    return 0.0;
}

float calculateRms(const std::vector<int16_t>& data) {
    float sum = std::inner_product(data.begin(), data.end(), data.begin(), 0.0f);
    return std::sqrt(sum / data.size());
}

void audioCallback(const naoqi_driver::AudioCustomMsg& msg) {
    const std::vector<int16_t>& frontLeft = msg.frontLeft;
    const std::vector<int16_t>& frontRight = msg.frontRight;

    std::vector<float> data1(frontLeft.begin(), frontLeft.end());
    std::vector<float> data2(frontRight.begin(), frontRight.end());

    float rmsFrontLeft = calculateRms(frontLeft);
    float rmsFrontRight = calculateRms(frontRight);

    float combinedIntensity = rmsFrontLeft + rmsFrontRight;

    if (combinedIntensity > INTENSITY_THRESHOLD){
        double itd = calculateItd(data1.data(), data2.data(), BUFFER_SIZE);
        angle_values.push_back(itd);        
    }
}

double calculateMode(const std::vector<double>& values) {
    std::map<double, int> frequency_map;
    for (double value : values) {
        frequency_map[value]++;
    }
    return std::max_element(frequency_map.begin(), frequency_map.end(),
                            [](const std::pair<double, int>& a, const std::pair<double, int>& b) {
                                return a.second < b.second;
                            })->first;
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "sound_localization");
    ros::NodeHandle nh;
    
    ros::Subscriber sub = nh.subscribe("/naoqi_driver/audio", 1000, audioCallback);

    ros::Rate rate(10);  // 10 Hz loop rate
    while (ros::ok()) {
        ros::spinOnce();

        if (angle_values.size() >= WINDOW_SIZE) {
            std::vector<double> last_values(angle_values.end() - WINDOW_SIZE, angle_values.end());
            double mode = calculateMode(last_values);
            ROS_INFO("Mode of last %d ITD angle values: %f", WINDOW_SIZE, mode);
            angle_values.clear();
        }
        rate.sleep();
    }
    return 0;
}

// #include "util.h"
// #include "sound.h"
// #include <vector>
// #include <ros/ros.h>
// #include <map>
// #include <cmath>
// #include <algorithm>
// #include <numeric>
// #include <fftw3.h>

// const float SPEED_OF_SOUND = 346.0f;
// const float DISTANCE_BETWEEN_EARS = 0.07f;
// const float SAMPLING_RATE = 48000.0f;
// const int BUFFER_SIZE = 4096;
// const int WINDOW_SIZE = 3;
// const float INTENSITY_THRESHOLD = 250;

// // Bandpass filter parameters
// const double LOW_CUT = 300.0;
// const double HIGH_CUT = 3000.0;
// const int ORDER = 5;

// std::vector<double> angle_values;

// // Simple Butterworth bandpass filter
// void butterworth_bandpass(std::vector<double>& data, double lowcut, double highcut, double fs, int order) {
//     int n = data.size();
//     double nyquist = 0.5 * fs;
//     double low = lowcut / nyquist;
//     double high = highcut / nyquist;

//     // Design the Butterworth filter
//     double pi = M_PI;
//     double K = tan(pi * low);
//     double K2 = K * K;
//     double omega = 2 * pi * high;
//     double sinOmega = sin(omega);
//     double cosOmega = cos(omega);
//     double alpha = sinOmega * sinh(log(2) / 2 * order * omega / sinOmega);
//     double a0 = 1 + alpha;
//     double a1 = -2 * cosOmega;
//     double a2 = 1 - alpha;
//     double b0 = (1 - cosOmega) / 2;
//     double b1 = 1 - cosOmega;
//     double b2 = (1 - cosOmega) / 2;

//     std::vector<double> filtered(n);
//     filtered[0] = data[0];
//     filtered[1] = data[1];

//     for (int i = 2; i < n; ++i) {
//         filtered[i] = (b0 / a0) * data[i] + (b1 / a0) * data[i - 1] + (b2 / a0) * data[i - 2]
//                       - (a1 / a0) * filtered[i - 1] - (a2 / a0) * filtered[i - 2];
//     }

//     data = filtered;
// }

// void hanning_window(std::vector<double>& signal) {
//     int N = signal.size();
//     for (int n = 0; n < N; ++n) {
//         signal[n] *= 0.5 * (1 - cos(2 * M_PI * n / (N - 1)));
//     }
// }

// double gcc_phat(const std::vector<double>& signal1, const std::vector<double>& signal2, double fs) {
//     int N = signal1.size();

//     // Apply Hanning window
//     std::vector<double> windowed_signal1 = signal1;
//     std::vector<double> windowed_signal2 = signal2;
//     hanning_window(windowed_signal1);
//     hanning_window(windowed_signal2);

//     // Allocate memory for FFTW
//     fftw_complex *FFT1 = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * N);
//     fftw_complex *FFT2 = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * N);
//     fftw_complex *CPSD = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * N);
//     fftw_plan p1 = fftw_plan_dft_r2c_1d(N, windowed_signal1.data(), FFT1, FFTW_ESTIMATE);
//     fftw_plan p2 = fftw_plan_dft_r2c_1d(N, windowed_signal2.data(), FFT2, FFTW_ESTIMATE);
//     fftw_plan p_cpsd = fftw_plan_dft_c2r_1d(N, CPSD, windowed_signal1.data(), FFTW_ESTIMATE);

//     // Execute FFT
//     fftw_execute(p1);
//     fftw_execute(p2);

//     // Compute Cross-Power Spectral Density with PHAT
//     for (int i = 0; i < N; ++i) {
//         double real = FFT1[i][0] * FFT2[i][0] + FFT1[i][1] * FFT2[i][1];
//         double imag = FFT1[i][1] * FFT2[i][0] - FFT1[i][0] * FFT2[i][1];
//         double mag = sqrt(real * real + imag * imag);
//         if (mag > 0) {
//             CPSD[i][0] = real / mag;
//             CPSD[i][1] = imag / mag;
//         } else {
//             CPSD[i][0] = 0;
//             CPSD[i][1] = 0;
//         }
//     }

//     // Compute inverse FFT
//     fftw_execute(p_cpsd);

//     // Find peak in cross-correlation
//     double max_value = -1;
//     int max_index = 0;
//     for (int i = 0; i < N; ++i) {
//         if (fabs(windowed_signal1[i]) > max_value) {
//             max_value = fabs(windowed_signal1[i]);
//             max_index = i;
//         }
//     }

//     // Calculate time delay
//     int max_shift = N / 2;
//     int peak_index = max_index - max_shift;
//     double time_delay = peak_index / fs;

//     // Cleanup
//     fftw_destroy_plan(p1);
//     fftw_destroy_plan(p2);
//     fftw_destroy_plan(p_cpsd);
//     fftw_free(FFT1);
//     fftw_free(FFT2);
//     fftw_free(CPSD);

//     return time_delay;
// }

// void preprocess_signal(std::vector<double>& signal, double fs) {
//     butterworth_bandpass(signal, LOW_CUT, HIGH_CUT, fs, ORDER);
// }

// double calculateItd(const float* data1, const float* data2, int size) {
//     std::vector<double> signal1(data1, data1 + size);
//     std::vector<double> signal2(data2, data2 + size);

//     preprocess_signal(signal1, SAMPLING_RATE);
//     preprocess_signal(signal2, SAMPLING_RATE);

//     double itd = gcc_phat(signal1, signal2, SAMPLING_RATE);
//     double z = itd * (SPEED_OF_SOUND / DISTANCE_BETWEEN_EARS);
//     double angle = std::asin(z) * (180.0 / M_PI);

//     return angle;
// }

// float calculateRms(const std::vector<int16_t>& data) {
//     float sum = std::inner_product(data.begin(), data.end(), data.begin(), 0.0f);
//     return std::sqrt(sum / data.size());
// }

// void audioCallback(const naoqi_driver::AudioCustomMsg& msg) {
//     const std::vector<int16_t>& frontLeft = msg.frontLeft;
//     const std::vector<int16_t>& frontRight = msg.frontRight;

//     std::vector<float> data1(frontLeft.begin(), frontLeft.end());
//     std::vector<float> data2(frontRight.begin(), frontRight.end());

//     float rmsFrontLeft = calculateRms(frontLeft);
//     float rmsFrontRight = calculateRms(frontRight);

//     float combinedIntensity = rmsFrontLeft + rmsFrontRight;

//     if (combinedIntensity > INTENSITY_THRESHOLD) {
//         double angle = calculateItd(data1.data(), data2.data(), BUFFER_SIZE);
//         angle_values.push_back(angle);
//     }
// }

// double calculateMode(const std::vector<double>& values) {
//     std::map<double, int> frequency_map;
//     for (double value : values) {
//         frequency_map[value]++;
//     }
//     return std::max_element(frequency_map.begin(), frequency_map.end(),
//                             [](const std::pair<double, int>& a, const std::pair<double, int>& b) {
//                                 return a.second < b.second;
//                             })->first;
// }

// int main(int argc, char* argv[]) {
//     ros::init(argc, argv, "sound_localization");
//     ros::NodeHandle nh;

//     ros::Subscriber sub = nh.subscribe("/naoqi_driver/audio", 1000, audioCallback);

//     ros::Rate rate(10);  // 10 Hz loop rate
//     while (ros::ok()) {
//         ros::spinOnce();

//         if (angle_values.size() >= WINDOW_SIZE) {
//             std::vector<double> last_values(angle_values.end() - WINDOW_SIZE, angle_values.end());
//             double mode = calculateMode(last_values);
//             ROS_INFO("Mode of last %d ITD angle values: %f", WINDOW_SIZE, mode);
//             angle_values.clear();
//         }
//         rate.sleep();
//     }
//     return 0;
// }

