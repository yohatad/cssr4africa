#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from l2cs import Pipeline, render
import torch
import numpy as np
from collections import OrderedDict

class GazeEstimationNode:
    def __init__(self):
        self.bridge = CvBridge()
        
        # Initialize the gaze estimation pipeline
        self.gaze_pipeline = Pipeline(
            weights='models/L2CSNet_gaze360.pkl',
            arch='ResNet50',
            device=torch.device('cpu')  # or 'cuda' if you have a GPU
        )

        # Initialize ROS node
        rospy.init_node('gaze_estimation_node', anonymous=True)

        # Subscribe to the /camera topic
        self.image_sub = rospy.Subscriber('/naoqi_driver/camera/front/image_raw', Image, self.image_callback)

        # Create a publisher to publish processed images
        self.image_pub = rospy.Publisher('/gaze_estimation', Image, queue_size=10)

        # Initialize tracker
        self.next_object_id = 0
        self.objects = OrderedDict()
        self.disappeared = OrderedDict()

    def register(self, centroid):
        self.objects[self.next_object_id] = centroid
        self.disappeared[self.next_object_id] = 0
        self.next_object_id += 1

    def deregister(self, object_id):
        del self.objects[object_id]
        del self.disappeared[object_id]

    def update(self, input_centroids):
        if len(input_centroids) == 0:
            for object_id in list(self.disappeared.keys()):
                self.disappeared[object_id] += 1
                if self.disappeared[object_id] > 50:  # Disappeared threshold
                    self.deregister(object_id)
            return self.objects

        if len(self.objects) == 0:
            for i in range(len(input_centroids)):
                self.register(input_centroids[i])
        else:
            object_ids = list(self.objects.keys())
            object_centroids = list(self.objects.values())

            D = self.distance_matrix(object_centroids, input_centroids)
            rows = D.min(axis=1).argsort()
            cols = D.argmin(axis=1)[rows]

            used_rows = set()
            used_cols = set()

            for (row, col) in zip(rows, cols):
                if row in used_rows or col in used_cols:
                    continue

                object_id = object_ids[row]
                self.objects[object_id] = input_centroids[col]
                self.disappeared[object_id] = 0

                used_rows.add(row)
                used_cols.add(col)

            unused_rows = set(range(0, D.shape[0])).difference(used_rows)
            unused_cols = set(range(0, D.shape[1])).difference(used_cols)

            if D.shape[0] >= D.shape[1]:
                for row in unused_rows:
                    object_id = object_ids[row]
                    self.disappeared[object_id] += 1
                    if self.disappeared[object_id] > 50:
                        self.deregister(object_id)
            else:
                for col in unused_cols:
                    self.register(input_centroids[col])

        return self.objects

    @staticmethod
    def distance_matrix(a, b):
        a = np.array(a)
        b = np.array(b)
        return np.linalg.norm(a[:, np.newaxis] - b, axis=2)

    def image_callback(self, data):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

        # Perform gaze estimation
        results = self.gaze_pipeline.step(cv_image)
        cv_image = render(cv_image, results)

        # Prepare centroids for tracking
        input_centroids = []

        # Extract centroids from results
        for result in results:
            bbox = result['bbox']
            centroid = (int((bbox[0] + bbox[2]) / 2), int((bbox[1] + bbox[3]) / 2))
            input_centroids.append(centroid)

        # Update the tracker with the new centroids
        objects = self.update(input_centroids)

        # Draw the centroids and object IDs
        for (object_id, centroid) in objects.items():
            text = f"ID {object_id}"
            cv2.putText(cv_image, text, (centroid[0] - 10, centroid[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.circle(cv_image, (centroid[0], centroid[1]), 4, (0, 255, 0), -1)

        # Display the image with bounding boxes, landmarks, and centroids
        cv2.imshow('Gaze Estimation', cv_image)
        cv2.waitKey(1)

        # Convert OpenCV image back to ROS Image message
        processed_image_msg = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')

        # Publish the processed image
        self.image_pub.publish(processed_image_msg)

if __name__ == '__main__':
    try:
        GazeEstimationNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
