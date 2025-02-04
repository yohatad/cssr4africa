""" angle_calculation_implementation.py Calculates Pepper angles from mediapipe landmarks

    Author:Daniel Barros
    Date: November 21, 2024
    Version: v1.0

    Copyright (C) 2023 CSSR4Africa Consortium

    This project is funded by the African Engineering and Technology Network (Afretec
    Inclusive Digital Transformation Research Grant Programme.
    
    Website: www.cssr4africa.org

This program comes with ABSOLUTELY NO WARRANTY.
"""

import sys

import numpy as np
import math

## class KeypointsToAngles
#
# This class contains methods to receive 3D keypoints and calculate skeleton joint angles  
class HumanToPepperRetargeting:
    '''
    # Body parts associated to their index
    body_mapping = {'0':  "Nose", 
                    '1':  "Neck", 
                    '2':  "RShoulder",
                    '3':  "RElbow",
                    '4':  "RWrist",
                    '5':  "LShoulder",
                    '6':  "LElbow",
                    '7':  "LWrist",
                    '8':  "MidHip"}
    '''

    ##  method __init__
    #
    #   Initialization method 
    def __init__(self):
        pass

    #   calculate 3D vector from two points ( vector = P2 - P1 )
    def vector_from_points(self, P1, P2):
        vector = [P2[0] - P1[0], P2[1] - P1[1], P2[2] - P1[2]]
        return vector
    
    def normalize(self, v):
        """ Return the normalized vector of v. """
        norm = np.linalg.norm(v)
        if norm < 1e-10:
            return v  # Avoid divide-by-zero; return original if near zero-length
        return v / norm

    def vector_angle_deg(self, v1, v2):
        """ Return angle in degrees between two vectors. """
        v1u = self.normalize(v1)
        v2u = self.normalize(v2)
        dot = np.dot(v1u, v2u)
        # Clamp dot to [-1, 1] to avoid floating-point errors
        dot = max(-1.0, min(1.0, dot))
        return np.degrees(np.arccos(dot))
    
    def LShoulderPitchRoll(self, LShoulder, LElbow, MidHip, Neck):
        
        torso_vector = MidHip - Neck
        Z = self.normalize(torso_vector)

        across_shoulders = Neck - LShoulder

        X_unnormalized = np.cross(Z, across_shoulders)
        X = self.normalize(X_unnormalized)

        Y = np.cross(Z, X)
        Y = self.normalize(Y)

        upper_arm = LElbow - LShoulder

        # Normalize the vector
        upper_arm = self.normalize(upper_arm)

        upper_arm_x = np.dot(upper_arm, X)
        upper_arm_y = np.dot(upper_arm, Y)
        upper_arm_z = np.dot(upper_arm, Z)

        pitch_radians = np.arctan2(upper_arm_y, upper_arm_z)
        roll_radians  = np.arctan2(upper_arm_x, upper_arm_y)

        pitch_angle = 90 - np.degrees(pitch_radians)
        roll_angle =  90 - np.degrees(roll_radians)

        # if the roll angle is outside [0, 90] range then just put 5.67 degrees
        if roll_angle < 0 or roll_angle > 90:
            roll_angle = 5.67


        return np.radians(pitch_angle), np.radians(roll_angle)

        # return pitch_angle, roll_angle

    def RShoulderPitchRoll(self, LShoulder, LElbow, MidHip, Neck):        
        torso_vector = MidHip - Neck
        Z = self.normalize(torso_vector)

        across_shoulders = Neck - LShoulder

        X_unnormalized = np.cross(Z, across_shoulders)
        X = self.normalize(X_unnormalized)

        Y = np.cross(Z, X)
        Y = self.normalize(Y)

        upper_arm = LElbow - LShoulder

        # Normalize the vector
        upper_arm = self.normalize(upper_arm)

        upper_arm_x = np.dot(upper_arm, X)
        upper_arm_y = np.dot(upper_arm, Y)
        upper_arm_z = np.dot(upper_arm, Z)

        pitch_radians = np.pi/2 - np.arctan2(upper_arm_y, upper_arm_z)
        roll_radians  = np.arctan2(upper_arm_x, upper_arm_y)

        pitch_angle = np.degrees(pitch_radians)
        roll_angle = -np.degrees(roll_radians) - 90

        # if the roll angle is outside [-90 , 0] range then just put -5.67 degrees
        if roll_angle < -90 or roll_angle > 0:
            roll_angle = -5.67


        # Change back to radians
        return np.radians(pitch_angle), np.radians(roll_angle)

        # return pitch_angle, roll_angle
    
    def LEblow(self, LShoulder, LElbow, LWrist):
        Z_elbow = LShoulder - LElbow
        Z_elbow = self.normalize(Z_elbow)

        ref_vector = LWrist - LElbow
        ref_vector = self.normalize(ref_vector)

        X_elbow_unnormalized = np.cross(Z_elbow, ref_vector)
        X_elbow = self.normalize(X_elbow_unnormalized)

        Y_elbow_unnormalized = np.cross(Z_elbow, X_elbow)
        Y_elbow = self.normalize(Y_elbow_unnormalized)

        forearm_global = (LWrist - LElbow)
        forearm_global = self.normalize(forearm_global)  # if you just want orientation

        forearm_x = np.dot(forearm_global, X_elbow)
        forearm_y = np.dot(forearm_global, Y_elbow)
        forearm_z = np.dot(forearm_global, Z_elbow)

        elbow_roll_radians = np.arctan2(forearm_y, forearm_z)
        elbow_yaw_radians = np.arctan2(forearm_x, forearm_y)

        elbow_roll_degrees = np.degrees(elbow_roll_radians) + 180
        elbow_yaw_degrees = np.degrees(elbow_yaw_radians)

        # if elbow roll degree is between 180 and 260, then just put
            # if elbow_roll_degrees < 0 and elbow_roll_degrees > 80:
            #     elbow_roll_degrees = 0

        elbow_roll_degrees = 0
        elbow_yaw_degrees = 0

        return elbow_yaw_degrees, elbow_roll_degrees

    def REblow(self, RShoulder, RElbow, RWrist):
        Z_elbow = RShoulder - RElbow
        Z_elbow = self.normalize(Z_elbow)

        ref_vector = RWrist - RElbow
        ref_vector = self.normalize(ref_vector)

        X_elbow_unnormalized = np.cross(Z_elbow, ref_vector)
        X_elbow = self.normalize(X_elbow_unnormalized)

        Y_elbow_unnormalized = np.cross(Z_elbow, X_elbow)
        Y_elbow = self.normalize(Y_elbow_unnormalized)

        forearm_global = (RWrist - RElbow)
        forearm_global = self.normalize(forearm_global)  # if you just want orientation

        forearm_x = np.dot(forearm_global, X_elbow)
        forearm_y = np.dot(forearm_global, Y_elbow)
        forearm_z = np.dot(forearm_global, Z_elbow)

        elbow_roll_radians = np.arctan2(forearm_z, forearm_y)
        elbow_yaw_radians = np.arctan2(forearm_x, forearm_y)

        elbow_yaw_degrees = np.degrees(elbow_yaw_radians)
        elbow_roll_degrees = np.degrees(elbow_roll_radians)

        elbow_yaw_degrees = 0
        elbow_roll_degrees = 0

        return elbow_yaw_degrees, elbow_roll_degrees
    
    #   Calculate right hip pitch angle
    def obtain_HipPitch_angles(self, P0_curr, P8_curr):
         # Calculate vector
        v_0_8_curr = self.vector_from_points(P0_curr, P8_curr)

        # Normals to axis planes
        n_YZ = [1, 0, 0]
        n_XZ = [0, 1, 0]
        n_XY = [0, 0, 1]

        # Project vectors on YZ plane
        v_0_8_curr_proj = v_0_8_curr - np.dot(v_0_8_curr, n_YZ)

        # Calculate HipPitch module
        # omega_HP_module = np.arccos((np.dot(v_0_8_prev_proj, v_0_8_curr_proj))/(np.linalg.norm(v_0_8_prev_proj) * np.linalg.norm(v_0_8_curr_proj)))
        x = (np.dot(n_XZ, v_0_8_curr_proj))/(np.linalg.norm(n_XZ) * np.linalg.norm(v_0_8_curr_proj))
        try:
            omega_HP_module =  math.acos(x)
        except ValueError:
            omega_HP_module =  0
        # omega_HP_module = np.arccos(x, where=(abs(x)<1), out=np.full_like(x, 0))

        # Intermediate vector and angle to calculate positive or negative pich
        x = np.dot(v_0_8_curr_proj, n_XY) / (np.linalg.norm(v_0_8_curr_proj) * np.linalg.norm(n_XY))
        try:
            intermediate_angle =  math.acos(x)
        except ValueError:
            intermediate_angle =  0
        # intermediate_angle = np.arccos(x, where=(abs(x)<1), out=np.full_like(x, 0))

        # Choose positive or negative pitch angle
        correction = 0.15
        if intermediate_angle > np.pi/2:
            HipPitch = np.pi - omega_HP_module - correction
        else:
            HipPitch = omega_HP_module - np.pi - correction
        
        return HipPitch

    def invert_right_left(self, wp_dict):
        temp_dict = {}

        if '0' in wp_dict:
            temp_dict['0'] = wp_dict['0']
        if '1' in wp_dict:
            temp_dict['1'] = wp_dict['1']
        if '2' in wp_dict:
            temp_dict['5'] = wp_dict['2']
        if '3' in wp_dict:
            temp_dict['6'] = wp_dict['3']
        if '4' in wp_dict:
            temp_dict['7'] = wp_dict['4']

        if '5' in wp_dict:
            temp_dict['2'] = wp_dict['5']
        if '6' in wp_dict:
            temp_dict['3'] = wp_dict['6']
        if '7' in wp_dict:
            temp_dict['4'] = wp_dict['7']
        if '8' in wp_dict:
            temp_dict['8'] = wp_dict['8']
        
        # print(temp_dict)
        return temp_dict

    ##  method get_angles
    #
    #   Get angles from socket and calculate joint angles
    def get_angles(self, wp_dict):
            # LShoulderPitch and LShoulderRoll needed keypoints
            LS = ['1','5','6','8']

            # LElbowYaw and LElbowRoll needed keypoints
            LE = ['1','5','6','7']

            # RShoulderPitch and RShoulderRoll needed keypoints
            RS = ['1','2','3','8']

            # RElbowYaw and RElbowRoll needed keypoints
            RE = ['1','2','3','4']   

            # HipPitch needed keypoints
            HP = ['1', '8']

            # Invert right arm with left arm
            wp_dict = self.invert_right_left(wp_dict) 
            
            # Init angles
            LShoulderPitch = LShoulderRoll = LElbowYaw = LElbowRoll = RShoulderPitch = RShoulderRoll = RElbowYaw = RElbowRoll = HipPitch = None

            LShoulderPitch, LShoulderRoll = self.LShoulderPitchRoll(wp_dict.get('5'), wp_dict.get('6'), wp_dict.get('8'), wp_dict.get('1'))

            LElbowYaw, LElbowRoll = self.LEblow(wp_dict.get('5'), wp_dict.get('6'), wp_dict.get('7'))

            RShoulderPitch, RShoulderRoll= self.RShoulderPitchRoll(wp_dict.get('2'), wp_dict.get('3'), wp_dict.get('8'), wp_dict.get('1'))
                        
            RElbowYaw, RElbowRoll = self.REblow(wp_dict.get('2'), wp_dict.get('3'), wp_dict.get('4'))

            
            # Invert right arm with left arm
            # wp_dict = self.invert_right_left(wp_dict) 
            
            # HipPitch angles 
            if all (body_part in wp_dict for body_part in HP):
                HipPitch = self.obtain_HipPitch_angles(wp_dict.get(HP[0]), wp_dict.get(HP[1]))

            # LShoulder angles 
            # if all (body_part in wp_dict for body_part in LS):        
            #     LShoulderPitch, LShoulderRoll = self.obtain_LShoulderPitchRoll_angles(wp_dict.get(LS[0]), wp_dict.get(LS[1]), wp_dict.get(LS[2]), wp_dict.get(LS[3]))

            # # LElbow angles
            # if all (body_part in wp_dict for body_part in LE):
            #     LElbowYaw, LElbowRoll = self.obtain_LElbowYawRoll_angle(wp_dict.get(LE[0]), wp_dict.get(LE[1]), wp_dict.get(LE[2]), wp_dict.get(LE[3]))

            # RShoulder angles
            # if all (body_part in wp_dict for body_part in RS):        
            #     RShoulderPitch, RShoulderRoll = self.obtain_RShoulderPitchRoll_angle(wp_dict.get(RS[0]), wp_dict.get(RS[1]), wp_dict.get(RS[2]), wp_dict.get(RS[3]))

            # RElbow angles
            # if all (body_part in wp_dict for body_part in RE):
            #     RElbowYaw, RElbowRoll = self.obtain_RElbowYawRoll_angle(wp_dict.get(RE[0]), wp_dict.get(RE[1]), wp_dict.get(RE[2]), wp_dict.get(RE[3]))
 
            return LShoulderPitch, LShoulderRoll, LElbowYaw, LElbowRoll, RShoulderPitch, RShoulderRoll, RElbowYaw, RElbowRoll, HipPitch
                

