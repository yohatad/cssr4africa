#ifndef ROBOT_LOCALIZATION_INTERFACE_H
#define ROBOT_LOCALIZATION_INTERFACE_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cmath>
#include <ctype.h>
#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>

using namespace std;

#define ROS_PACKAGE_NAME "cssr_system"
#define MAX_FILENAME_LENGTH 200

bool computeAbsolutePose();
bool computeAbsolutePoseWith3Landmarks();
bool computeAbsolutePoseWithDepth();

// Triangulation helper methods
bool areLandmarksCollinear(double x1, double y1, double x2, double y2, double x3, double y3, double tolerance = 0.01);
double computeBearingAngle(const cv::Point2f& p1, const cv::Point2f& p2);
bool triangulatePosition(double x1, double y1, double x2, double y2, double x3, double y3,
                        double alpha1, double alpha2, double* robot_x, double* robot_y);
bool computeCircleFromChordAndAngle(double x1, double y1, double x2, double y2, double angle,
                                    double* xc1, double* yc1, double* xc2, double* yc2, double* r);
bool selectCorrectIntersection(double x1, double y1, double x2, double y2,
                                double lm1_x, double lm1_y, double lm2_x, double lm2_y, 
                                double lm3_x, double lm3_y, double* robot_x, double* robot_y);
bool validateTriangulationResult(double robot_x, double robot_y,
                                double x1, double y1, double x2, double y2, double x3, double y3,
                                const cv::Point2f& img1, const cv::Point2f& img2, const cv::Point2f& img3);
double computeExpectedAngle(double robot_x, double robot_y, 
                            double lm1_x, double lm1_y, double lm2_x, double lm2_y);
double computeRobotHeading(const cv::Point2f& marker_center, 
                            double marker_x, double marker_y, 
                            double robot_x, double robot_y);
void updateRobotPose(double x, double y, double theta);

// Legacy methods
double computeAngle(const std::pair<double, double>& p1, const std::pair<double, double>& p2);
double computeYaw(const std::pair<double, double>& marker_center, double marker_x, double marker_y, 
                    double robot_x, double robot_y);
int circle_circle_intersection(double x0, double y0, double r0, double x1, double y1, double r1,
                                double *xi, double *yi, double *xi_prime, double *yi_prime);
int circle_centre(double x1, double y1, double x2, double y2, double alpha, 
                    double *xc1, double *yc1, double *xc2, double *yc2, double *r);
void publishPose();

#endif