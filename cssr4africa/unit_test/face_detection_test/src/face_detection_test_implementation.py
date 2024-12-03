import rospkg

class FaceDetectionTest:
    def __init__(self):
        self._rospack = rospkg.RosPack()
        self._face_detection_test_package_path = self._rospack.get_path('face_detection_test')
        self._face_detection_package_path = self._rospack.get_path('face_detection')
    
    # Perfrom two test one for the mediapipe face detection and other for sexdrep face detection. 
    def test_media_pipe(self):
        pass

    def test_sexdrep(self):
        pass

if __name__ == '__main__':
    FaceDetectionTest()