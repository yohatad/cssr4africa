/*******************************************************************************************************************
 *   Utility functions for inverse kinematics for the Pepper robot
 *   ------------------------------------------------------------------------------------
 *
 *
 *   The solution to the inverse kinematic problem is based on original code by 
 *   Oleg Mazurov:    www.circuitsathome.com/mcu/robotic-arm-inverse-kinematics-on-arduino
 *   and subsequently adapted by Eric Goldsmith:  www.ericgoldsmith.com
 *   
 *   See the following for a figure to explain the variables in the inverse kinematic solution:
 *   github.com/EricGoldsmith/AL5D-BotBoarduino-PS2/blob/master/Robot_Arm_IK.pdf
 *
 *   Also, refer to the CR_13.pdf lectures slides on www.cognitiverobotics.net for more details on the kinematics 
 *   and the solution to the inverse kinematics.
 *
 *   This inverse kinematic solution has been modified to make it compatible with the task-level 
 *   robot program specification and so that T5 is embedded in the wrist.
 *
 *   The serial port interface to the AL5D robot was written by Victor Akinwande, Carnegie Mellon University Africa 
 *   Refer to lynxmotion_ssc-32u_usb_user_guide.pdf, p. 24, for details of the serial port command protocol.
 * 
 *   Audit Trail
 *   -----------
 *   28 June 2020:     re-factored code to separate calculation of the joint angles using the inverse kinematics,  
 *                     from the calculation of servomotor setpoint values.  
 *                     This was done to allow the simulator to be controlled by publishing joint angles on the 
 *                     ROS /lynxmotion_al5d/joints_positions/command topic 
 *
 *   11 February 2022: Added simulator field to robotConfigurationData
 *                     Created this lynxmotionUtilities.cpp file; previously these utilities were embedded in the 
 *                     implementation file
 *
 *
 *******************************************************************************************************************/

#include <gesture_execution/pepperKinematicsUtilities.h>


/*********************************************************************/
/*                                                                   */
/* Inverse kinematics for LynxMotion AL5D robot manipulator          */
/*                                                                   */
/*********************************************************************/

double degrees(double radians)
{
    double degrees = radians * (double) 180.0 / (double) M_PI; // David Vernon ... cast to float
    return degrees;
}

double radians(double degrees)
{
    double radians = degrees / ((double) 180.0 / (double) M_PI); // David Vernon ... cast to float
    return radians;
}
 
// # Function to get the position of the partial arm end-effector (elbow)
void get_elbow_position(int arm, double theta1, double theta2, double* position){
   double l1 = -57.0;
   double l2 = -57.0;
   double l3 = 86.82;
   double l4 = 181.2;
   double l5 = -57.0;
   double l6 = 0.13;

   if (arm == RIGHT_ARM){
      l2 = -149.74;
      l5 = -15.0;
   }
   else if (arm == LEFT_ARM){
      l2 = 149.74;
      l5 = 15.0;
   }

   double sin_theta1 = sin(theta1);
   double cos_theta1 = cos(theta1);
   double sin_theta2 = sin(theta2);
   double cos_theta2 = cos(theta2);

   double f1 = l6 * sin_theta1;
   double f2 = l4 * cos_theta2;
   double f3 = l5 * sin_theta2;
   double f4 = f2 - f3;
   double f5 = cos_theta1 * f4;
   double position_x = l1 + f1 + f5;

   f1 = l5 * cos_theta2;
   f2 = l4 * sin_theta2;
   double position_y = l2 + f1 + f2;

   f1 = l6 * cos_theta1;
   f2 = sin_theta1 * f4;
   double position_z = l3 + f1 - f2;

   // # formulas derived from the FK matrix 
   position[0] = position_x;
   position[1] = position_y;
   position[2] = position_z;

   // printf("The position of the elbow is %f, %f, %f\n", position_x, position_y, position_z);

}

// # Function to get the position of the arm end-effector (wrist)
void get_wrist_position(int arm, double theta1, double theta2, double* position){
   double l1 = -57.0;
   double l2 = -57.0;
   double l3 = 86.82;
   double l4 = 181.2;
   double l5 = -57.0;
   double l6 = 0.13;

   if (arm == RIGHT_ARM){
      l2 = -149.74;
      l5 = -15.0;
   }
   else if (arm == LEFT_ARM){
      l2 = 149.74;
      l5 = 15.0;
   }

   // # formulas derived from the FK matrix 
   position[0] = l1 + l6*sin(theta1) + (cos(theta1)*(l4*cos(theta2)) - l5*sin(theta2));
   position[1] = l2 + l5*cos(theta2) + l4*sin(theta2);
   position[2] = l3 + l6*cos(theta1) - (sin(theta1)*(l4*cos(theta2)) - l5*sin(theta2));

   // printf("The position of the elbow is %f, %f, %f\n", position[0], position[1], position[2]);

}

// # Function that returns the partial angles ShoulderPitch (t1) and ShoulderRoll (t2) given the elbow position (ex, ey, ez)
void get_arm_partial_angles(int arm, double ex, double ey, double ez, double* theta1, double* theta2){
   double l1 = -57.0;
   double l2 = 0.0;
   double l3 = 86.82;
   double l4 = 181.2;
   double l5 = 0.0;
   double l6 = 0.13;
   double shoulderRoll = 0.0;
   double shoulderPitch = 0.0;

   if (arm == RIGHT_ARM){
      l2 = -149.74;
      l5 = -15.0;
   }
   else if (arm == LEFT_ARM){
      l2 = 149.74;
      l5 = 15.0;
   } 
   // # ShoulderRoll or theta2 
   double f1 = ey-l2;
   double f2 = sqrt(pow(l4,2) + pow(l5,2));
   double f3 = asin(f1/f2);
   double f4 = atan(l5/l4);
   double t2_temp = f3 - f4;
   // printf("f1 is %f, f2 is %f, f3 is %f, f4 is %f\n", f1, f2, f3, f4);
   // printf("t2_temp is %f\n", t2_temp);

   if (arm == RIGHT_ARM){
      if (t2_temp + f4 > -PI/2 - f4)
         shoulderRoll = t2_temp;
      else{
         shoulderRoll = -PI - f3 - f4;
         if (shoulderRoll < -1.5630)
               shoulderRoll = t2_temp;
      }
      // # check if solution is within Peppers range
      if ((shoulderRoll < -1.58 || shoulderRoll >= -0.0087))
         shoulderRoll = -0.0087;
   }

   else if (arm == LEFT_ARM){
      if (t2_temp + f4 < PI/2 - f4)
         shoulderRoll = t2_temp;
      else{
         shoulderRoll = PI - f3 - f4;
         if (shoulderRoll > 1.5630)
               shoulderRoll = t2_temp;
      }
      // # check if solution is within Peppers range
      if (shoulderRoll > 1.58 || shoulderRoll <= 0.0087)
         shoulderRoll = 0.0087;
   }

   // # ShoulderPitch or theta1
   double n = (l4*cos(shoulderRoll)) - (l5*sin(shoulderRoll));
   f1 = ex-l1;
   f2 = ez-l3;
   f3 = atan2(f1,f2);
   f4 = pow(f1,2) + pow(f2,2) - pow(l6,2);
   f4 = sqrt(f4);
   f4 = atan2(f4,l6);
   double t1_1 = f3 - f4;
   // double t1_1 = atan2(ex-l1, ez-l3) - atan2(sqrt(pow((ex-l1),2) + pow((ez-l3),2) - pow(l6,2)), l6);
   f3 = (l6*f1) - (n*f2);
   f4 = (l6*f2) + (n*f1);
   double t1_2 = atan2(f3, f4);
   // double t1_2 = atan2(((l6*(ex-l1)) - (n*(ez-l3))), ((l6*(ez-l3) )+ (n*(ex-l1))));
   // # check if solution is within Peppers range
   if (t1_1 < -2.1 || t1_1 > 2.1)
      t1_1 = NAN;

   if (t1_2 < -2.1 || t1_2 > 2.1)
      t1_2 = NAN;

   double sol1[3], sol2[3];

   get_elbow_position(arm, t1_1, shoulderRoll, sol1);
   get_elbow_position(arm, t1_2, shoulderRoll, sol2);

   // printf("SOlution 1 is %f, %f, %f\n", sol1[0], sol1[1], sol1[2]);
   // printf("SOlution 2 is %f, %f, %f\n", sol2[0], sol2[1], sol2[2]);
   // # check which of the solutions is closer to the position of the elbow 
   double dist1 = sqrt(pow((sol1[0]-ex),2) + pow((sol1[1]-ey),2) + pow((sol1[2]-ez),2));
   double dist2 = sqrt(pow((sol2[0]-ex),2) + pow((sol2[1]-ey),2) + pow((sol2[2]-ez),2));

   // printf("dist 1, %f, dist 2, %f\n", dist1, dist2);

   if (dist1 < dist2 || isnan(dist2))
      shoulderPitch = t1_1;
   else if (dist1 > dist2 || isnan(dist1))
      shoulderPitch = t1_2;
   
   // printf("Shoulder pitch is %f, shoulder roll is %f\n", shoulderPitch, shoulderRoll);
   *theta1 = shoulderPitch;
   *theta2 = shoulderRoll;
}

// # Function that returns all 4 angles of the arm chain given the elbow positon (ex, ey, ez) and wrist position (px, py, pz)
void getArmAngles(int arm, double ex, double ey, double ez, double px, double py, double pz, double* shoulderPitch, double* shoulderRoll){
   // double shoulderPitch, shoulderRoll;
   get_arm_partial_angles(arm, ex, ey, ez, shoulderPitch, shoulderRoll);
   // printf("Shoulder pitch is %f, shoulder roll is %f\n", *shoulderPitch, *shoulderRoll);
   //  elbowRoll = calculate_theta4(shoulderPitch, shoulderRoll, px, py, pz, arm)
   //  elbowYaw = calculate_theta3(shoulderPitch, shoulderRoll, elbowRoll, px, py, pz, arm)

   //  return shoulderPitch, shoulderRoll, elbowYaw, elbowRoll
}

// # Function that returns the head angles given the head end-effector position (BottomCamera)
void getHeadAngles(double px, double py, double pz, double* headYaw, double* headPitch){
   double l1 = -38.0;
   double l2 = 169.9;
   double l3 = 93.6;
   double l4 = 61.6;

   // # theta1
   *headYaw = atan2(py, (px - l1));
   // # theta2
   *headPitch = asin((l2 - pz) / sqrt(pow(l4,2) + pow(l3,2))) + atan(l4/l3);

   // # Check if the calculated angles fall within Pepper's range
   // # if not set the angles to 0
   if (isnan(*headYaw) || *headYaw < -2.1 || *headYaw > 2.1)
      *headYaw = 0.0;
   // if (isnan(*headPitch) || *headPitch < -0.71 || *headPitch > 0.45)
   if (isnan(*headPitch) || *headPitch < -0.71 || *headPitch > 0.6371)
      *headPitch = 0.0;
}

void inverseKinematicsArm(int arm, double p_x, double p_y, double p_z){

   printf("In the inverse kinematics, p_x is %f, p_y is %f, p_z is %f\n", p_x, p_y, p_z);
   // forward_kinematics_transfrom(arm, p_x, p_y, p_z, theta_x, theta_y, theta_z);

   // int arm = RIGHT_ARM;

   double l_1 = SHOULDER_OFFSET_X;
   double l_2 = SHOULDER_OFFSET_Y;
   double l_3 = SHOULDER_OFFSET_Z;
   double l_4 = UPPER_ARM_LENGTH;
   double l_5 = ELBOW_OFFSET_Y;
   double l_6 = ELBOW_OFFSET_Z;

   double theta_2_max = 1.5620;
   double theta_2_min = 0.0087;
   double theta_1_max = 2.0857;
   double theta_1_min = -2.0857;

   if(arm == RIGHT_ARM){
      l_2 = -l_2;
      l_5  = -l_5;

      theta_2_min = -1.5620;
      theta_2_max = -0.0087;
   }

   double theta_2_draft[2] = {0, 0}; 
   double theta_1_draft[2] = {0, 0};  

   // THeta temp 1 and 3 are for left arm
   // Theta temp 1 and 2 are for right arm

   double theta_temp = asin((p_y - l_2)/sqrt(pow(l_4, 2) + pow(l_5, 2))) - atan(l_5/l_4);
   printf("Theta temp 1 is %f\n", theta_temp);
   theta_2_draft[0] = theta_temp;

   theta_temp = -PI - asin((p_y - l_2)/sqrt(pow(l_4, 2) + pow(l_5, 2))) - atan2(l_5,l_4);
   printf("Theta temp 2 is %f\n", theta_temp);
   theta_2_draft[1] = theta_temp;

   theta_temp = PI - asin((p_y - l_2)/sqrt(pow(l_4, 2) + pow(l_5, 2))) - atan2(l_5,l_4);
   printf("Theta temp 3 is %f\n", theta_temp);
   theta_temp = theta_temp;
   // theta_2_draft[1] = theta_temp;

   double theta_2 = 0;
   double theta_1 = 0;
   double theta_3 = 0;
   double theta_4 = 0;

   if(theta_2_draft[0] >= theta_2_min && theta_2_draft[0] <= theta_2_max){
      theta_2 = theta_2_draft[0];
   }
   else if(theta_2_draft[1] >= theta_2_min && theta_2_draft[1] <= theta_2_max){
      theta_2 = theta_2_draft[1];
   }
   else{
      printf("No solution for theta 2\n");
      // return;
   }

   double A = (l_4 * cos(theta_2)) - (l_5 * sin(theta_2));
   
   theta_temp = atan2(((l_6*(p_x - l_1))- (A * (p_z - l_3))), ((l_6 * (p_z - l_3)) + (A * (p_x - l_1))));
   printf("Theta temp 4 is %f\n", theta_temp);
   theta_1_draft[0] = theta_temp;

   theta_temp = atan2((p_x - l_1), (p_z - l_3)) - atan2((sqrt(pow((p_x - l_1), 2) + pow((p_z - l_3), 2) - pow(l_6, 2))), l_6);
   printf("Theta temp 5 is %f\n", theta_temp);
   theta_1_draft[1] = theta_temp;

   if(theta_1_draft[0] >= theta_1_min && theta_1_draft[0] <= theta_1_max){
      theta_1 = theta_1_draft[0];
   }
   else if(theta_1_draft[1] >= theta_1_min && theta_1_draft[1] <= theta_1_max){
      theta_1 = theta_1_draft[1];
   }
   else{
      printf("No solution for theta 1\n");
      // return;
   }


   printf("Theta 1 is %f, Theta 2 is %f\n", theta_1, theta_2);
   return;
}

void wait(int ms)
{
   usleep(ms * 1000);
}

