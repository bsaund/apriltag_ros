#!/usr/bin/env python

import rospy
from apriltag_ros.bundle_calibrator import BundleCalibrator


if __name__ == "__main__":
    rospy.init_node("bundle_calibrator_test_node")
    bc = BundleCalibrator()
    rospy.spin()

