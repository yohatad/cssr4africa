import numpy as np
import lap
from filterpy.kalman import KalmanFilter
from itertools import count
from collections import OrderedDict
from scipy.spatial import distance as dist

class TrackerUtils:
    """
    Utility functions for SORT tracker operations.
    """

    @staticmethod
    def linear_assignment(cost_matrix):
        """
        Solves the Linear Assignment Problem using Jonker-Volgenant algorithm.

        Args:
            cost_matrix (ndarray): 2D cost matrix.

        Returns:
            ndarray: Array of matched pairs [col_idx, row_idx].
        """
        _, x, y = lap.lapjv(cost_matrix, extend_cost=True)
        return np.array([[y[i], i] for i in x if i >= 0])

    @staticmethod
    def iou_batch(bb_test, bb_gt):
        """
        Computes the Intersection over Union (IoU) between two sets of bounding boxes.

        Args:
            bb_test (ndarray): Bounding boxes to test, shape (N, 4), format [x1, y1, x2, y2].
            bb_gt (ndarray): Ground truth bounding boxes, shape (M, 4), format [x1, y1, x2, y2].

        Returns:
            ndarray: IoU matrix of shape (N, M).
        """
        bb_test = np.expand_dims(bb_test, axis=1)  # Shape: (N, 1, 4)
        bb_gt = np.expand_dims(bb_gt, axis=0)      # Shape: (1, M, 4)

        xx1 = np.maximum(bb_test[..., 0], bb_gt[..., 0])
        yy1 = np.maximum(bb_test[..., 1], bb_gt[..., 1])
        xx2 = np.minimum(bb_test[..., 2], bb_gt[..., 2])
        yy2 = np.minimum(bb_test[..., 3], bb_gt[..., 3])

        w = np.maximum(0.0, xx2 - xx1)
        h = np.maximum(0.0, yy2 - yy1)
        intersection = w * h

        area_bb_test = (bb_test[..., 2] - bb_test[..., 0]) * (bb_test[..., 3] - bb_test[..., 1])
        area_bb_gt = (bb_gt[..., 2] - bb_gt[..., 0]) * (bb_gt[..., 3] - bb_gt[..., 1])

        union = area_bb_test + area_bb_gt - intersection
        return intersection / np.maximum(union, 1e-9)

    @staticmethod
    def associate_detections_to_trackers(detections, trackers, iou_threshold=0.3):
        """
        Assigns detections to tracked objects using Intersection over Union (IoU).

        Args:
            detections (ndarray): Detected bounding boxes, shape (N, 4).
            trackers (ndarray): Tracked bounding boxes, shape (M, 4).
            iou_threshold (float): Minimum IoU to consider a valid match.

        Returns:
            tuple: 
                - matches (ndarray): Matched pairs [detection_idx, tracker_idx].
                - unmatched_detections (ndarray): Indices of unmatched detections.
                - unmatched_trackers (ndarray): Indices of unmatched trackers.
        """
        if len(trackers) == 0:
            return np.empty((0, 2), dtype=int), np.arange(len(detections)), np.empty((0, 5), dtype=int)

        iou_matrix = TrackerUtils.iou_batch(detections, trackers)

        if min(iou_matrix.shape) > 0:
            a = (iou_matrix > iou_threshold).astype(np.int32)
            if a.sum(1).max() == 1 and a.sum(0).max() == 1:
                matched_indices = np.stack(np.where(a), axis=1)
            else:
                matched_indices = TrackerUtils.linear_assignment(-iou_matrix)
        else:
            matched_indices = np.empty(shape=(0, 2))

        unmatched_detections = list(range(len(detections)))
        unmatched_trackers = list(range(len(trackers)))

        matches = []
        for d, t in matched_indices:
            if iou_matrix[d, t] < iou_threshold:
                unmatched_detections.append(d)
                unmatched_trackers.append(t)
            else:
                matches.append([d, t])
                unmatched_detections.remove(d)
                unmatched_trackers.remove(t)

        return np.array(matches), np.array(unmatched_detections), np.array(unmatched_trackers)

    @staticmethod
    def convert_bbox_to_z(bbox):
        """
        Converts a bounding box [x1, y1, x2, y2] to [x, y, s, r].

        Args:
            bbox (list or ndarray): Bounding box.

        Returns:
            ndarray: Center format [x, y, scale, ratio].
        """
        w, h = bbox[2] - bbox[0], bbox[3] - bbox[1]
        x, y = bbox[0] + w / 2.0, bbox[1] + h / 2.0
        s, r = w * h, w / float(h)
        return np.array([x, y, s, r]).reshape((4, 1))

    @staticmethod
    def convert_x_to_bbox(x, score=None):
        """
        Converts a bounding box from center format [x, y, s, r] to corner format [x1, y1, x2, y2].

        Args:
            x (ndarray): Bounding box in center format.
            score (float, optional): Confidence score.

        Returns:
            ndarray: Bounding box in corner format [x1, y1, x2, y2].
        """
        w = np.sqrt(x[2] * x[3])
        h = x[2] / w
        x1 = x[0] - w / 2.0
        y1 = x[1] - h / 2.0
        x2 = x[0] + w / 2.0
        y2 = x[1] + h / 2.0

        bbox = [x1, y1, x2, y2]
        if score is not None:
            bbox.append(score)

        return np.array(bbox).reshape((1, -1))

class KalmanBoxTracker(object):
    """
    Represents the internal state of an individual tracked object observed as a bounding box.
    Uses a Kalman Filter to estimate and predict bounding box states.
    """
    # Unique ID generator for all trackers
    _id_counter = count(0)
    
    def __init__(self, bbox):
        """
        Initializes a KalmanBoxTracker with a bounding box.

        Args:
            bbox (ndarray): Initial bounding box [x1, y1, x2, y2].
        """
        #define constant velocity model
        self.kf = KalmanFilter(dim_x=7, dim_z=4) 
        self.kf.F = np.eye(7) + np.eye(7, k=4)
        
        # Measurement Matrix
        self.kf.H = np.zeros((4, 7))
        self.kf.H[0:4, 0:4] = np.eye(4)

        # Measurement Uncertainty (R)
        self.kf.R[2:, 2:] *= 10.0
        
        # Covariance Matrix (P) and Process Noise (Q)
        self.kf.P[4:, 4:] *= 1000.0  # High uncertainty for velocities
        self.kf.P *= 10.0
        self.kf.Q[-1, -1] *= 0.01
        self.kf.Q[4:, 4:] *= 0.01

        # Initialize state vector with the given bbox
        self.kf.x[:4] = TrackerUtils.convert_bbox_to_z(bbox)
        
        # Tracker metadata
        self.id = next(KalmanBoxTracker._id_counter)
        self.time_since_update = 0
        self.history = []
        self.hits = 0
        self.hit_streak = 0
        self.age = 0

    def update(self, bbox):
        """
        Updates the Kalman Filter state with a new bounding box.

        Args:
            bbox (ndarray): Detected bounding box [x1, y1, x2, y2].
        """
        self.time_since_update = 0
        self.history = []
        self.hits += 1
        self.hit_streak += 1
        self.kf.update(TrackerUtils.convert_bbox_to_z(bbox))

    def predict(self):
        """
        Predicts the next bounding box using the Kalman Filter.

        Returns:
            ndarray: Predicted bounding box [x1, y1, x2, y2].
        """
        if((self.kf.x[6]+self.kf.x[2])<=0):
            self.kf.x[6] *= 0.0
        self.kf.predict()
        self.age += 1
        if(self.time_since_update>0):
            self.hit_streak = 0
        self.time_since_update += 1
        self.history.append(TrackerUtils.convert_x_to_bbox(self.kf.x))
        return self.history[-1]

    def get_state(self):
        """
        Returns the current bounding box estimate.

        Returns:
            ndarray: Estimated bounding box [x1, y1, x2, y2].
        """
        return TrackerUtils.convert_x_to_bbox(self.kf.x)

class Sort:
    """
    SORT: Simple Online and Realtime Tracker.
    """

    def __init__(self, max_age=1, min_hits=3, iou_threshold=0.3):
        """
        Initializes the SORT tracker.

        Args:
            max_age (int): Maximum allowed frames without updates.
            min_hits (int): Minimum hits before the object is confirmed.
            iou_threshold (float): IoU threshold for association.
        """
        self.max_age, self.min_hits, self.iou_threshold = max_age, min_hits, iou_threshold
        self.trackers, self.frame_count = [], 0

    def update(self, detections=np.empty((0, 5))):
        """
        Updates trackers with new detections.

        Args:
            detections (ndarray): Detected bounding boxes [x1, y1, x2, y2, score].

        Returns:
            ndarray: Array of tracked objects [x1, y1, x2, y2, ID].
        """
        self.frame_count += 1
        ret, trks = [], np.zeros((len(self.trackers), 5))

        # Predict existing trackers
        for t, trk in enumerate(self.trackers):
            trks[t, :4] = trk.predict()[0]
        trks = np.ma.compress_rows(np.ma.masked_invalid(trks))

        # Associate detections to trackers
        matches, unmatched_dets, unmatched_trks = TrackerUtils.associate_detections_to_trackers(
            detections, trks, self.iou_threshold
        )

        # Update matched trackers
        for m in matches:
            self.trackers[m[1]].update(detections[m[0]])

        # Initialize new trackers for unmatched detections
        for i in unmatched_dets:
            self.trackers.append(KalmanBoxTracker(detections[i]))

        # Remove dead trackers and prepare output
        for t, trk in reversed(list(enumerate(self.trackers))):
            if trk.time_since_update > self.max_age:
                self.trackers.pop(t)
            elif trk.hit_streak >= self.min_hits or self.frame_count <= self.min_hits:
                ret.append(np.concatenate((trk.get_state()[0], [trk.id + 1])).reshape(1, -1))

        return np.concatenate(ret) if ret else np.empty((0, 5))
    
# Centroid tracker class for tracking objects. 
class CentroidTracker:
    def __init__(self, max_disappeared=50, distance_threshold=50):
        # Adjustable parameters
        self.max_disappeared = max_disappeared
        self.distance_threshold = distance_threshold

        # Internal variables
        self.next_object_id = 0
        self.objects = OrderedDict()
        self.disappeared = OrderedDict()

    def register(self, centroid):
        """Registers a new object with the next available ID and resets its disappearance count."""
        self.objects[self.next_object_id] = centroid
        self.disappeared[self.next_object_id] = 0
        self.next_object_id += 1

    def deregister(self, object_id):
        """Removes an object from tracking."""
        if object_id in self.objects:
            del self.objects[object_id]
            del self.disappeared[object_id]

    def update(self, centroids):
        """Updates the tracker with new centroids from the current frame."""
        # If no centroids are detected, increase disappearance count for existing objects
        if len(centroids) == 0:
            for object_id in list(self.disappeared.keys()):
                self.disappeared[object_id] += 1
                if self.disappeared[object_id] > self.max_disappeared:
                    self.deregister(object_id)
            return self.objects

        # Convert input centroids to numpy array for easy computation
        input_centroids = np.array(centroids)

        # Register each centroid if there are no tracked objects
        if len(self.objects) == 0:
            for i in range(0, len(input_centroids)):
                self.register(input_centroids[i])
        else:
            # List of tracked object IDs and their current centroids
            object_ids = list(self.objects.keys())
            object_centroids = list(self.objects.values())

            # Compute distances between existing objects and new centroids
            D = dist.cdist(np.array(object_centroids), input_centroids)

            # Sort rows and columns by closest distances
            rows = D.min(axis=1).argsort()
            cols = D.argmin(axis=1)[rows]

            used_rows = set()
            used_cols = set()

            # Match each tracked object with the closest new centroid if within the distance threshold
            for (row, col) in zip(rows, cols):
                if row in used_rows or col in used_cols:
                    continue

                # Check if distance is within the threshold for matching
                if D[row, col] > self.distance_threshold:
                    continue

                # Update the centroid of the matched object
                object_id = object_ids[row]
                self.objects[object_id] = input_centroids[col]
                self.disappeared[object_id] = 0

                used_rows.add(row)
                used_cols.add(col)

            # Process unmatched rows and columns
            unused_rows = set(range(0, D.shape[0])).difference(used_rows)
            unused_cols = set(range(0, D.shape[1])).difference(used_cols)

            # Mark unmatched existing objects as disappeared
            for row in unused_rows:
                object_id = object_ids[row]
                self.disappeared[object_id] += 1
                if self.disappeared[object_id] > self.max_disappeared:
                    self.deregister(object_id)

            # Register unmatched new centroids as new objects
            for col in unused_cols:
                self.register(input_centroids[col])

        return self.objects
