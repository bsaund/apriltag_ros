from apriltag_ros.msg import AprilTagDetectionArray
from apriltag_ros.srv import CalibrateBundle, CalibrateBundleRequest
import rospy

class BundleCalibrator:
    def __init__(self):
        self.tag_sub = rospy.Subscriber("tag_detections", AprilTagDetectionArray, self.tag_detection_callback)
        self.calibration_data = []

    def get_unique_tags_seen(self):
        tags = [d.id[0] for arr in self.calibration_data for d in arr.detections]
        return sorted(list(set(tags)))

    def isTooSimilarToPrevious(self, tags):
        if len(self.calibration_data) == 0:
            return False

        prev = self.calibration_data[-1]

        #Check if detected tags are different
        if len(prev.detections) != len(tags.detections):
            return False
        for i in range(len(prev.detections)):
            if prev.detections[i].id[0] != tags.detections[i].id[0]:
                return False

        #Check if tags (i.e. camrea) has moved significantly since previous measurement
        for i in range(len(prev.detections)):
            pp = prev.detections[i].pose.pose.pose.position
            tp = tags.detections[i].pose.pose.pose.position
            d_sq = (pp.x - tp.x)**2 + (pp.y - tp.y)**2 + (pp.z - tp.z)**2
            tag_size = prev.detections[i].size[0]
            if d_sq > tag_size**2:
                return False
            
        return True

    def tag_detection_callback(self, tags_and_bundles):
        tags = AprilTagDetectionArray()
        tags.header = tags_and_bundles.header

        tags.detections = [d for d in tags_and_bundles.detections if len(d.id)==1]

        tags.detections.sort(key = lambda detection: detection.id[0])

        ids = [d.id[0] for d in tags.detections]

        if len(ids) <= 1:
            return

        if self.isTooSimilarToPrevious(tags):
            return

        self.calibration_data.append(tags)


    def filter_calibration_data(self, selected_ids):
        filtered = []
        for datum in self.calibration_data:
             filtered_datum = AprilTagDetectionArray()
             filtered_datum.header = datum.header
             filtered_datum.detections = [t for t in datum.detections if t.id[0] in selected_ids]
             if len(filtered_datum.detections) > 1:
                 filtered.append(filtered_datum)
        return filtered
    


    def calibrate(self, ids_to_calibrate):
        try:
            rospy.wait_for_service('calibrate_bundle', 2)
        except rospy.ROSException as e:
            rospy.loginfo('calibrate bundle service not running')
            raise e

        calibration_service = rospy.ServiceProxy('calibrate_bundle', CalibrateBundle)
        print("Calling service")
        req = CalibrateBundleRequest()

        

        req.calibration_data = self.filter_calibration_data(ids_to_calibrate)
        
        resp = calibration_service(req)
        print("Service responded")
