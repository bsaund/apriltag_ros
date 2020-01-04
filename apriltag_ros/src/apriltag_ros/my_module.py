import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtGui import QPixmap, QImage

from apriltag_ros.msg import *
from apriltag_ros.bundle_calibrator import BundleCalibrator

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class BundleCalibration(Plugin):

    def __init__(self, context):
        super(BundleCalibration, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('BundleCalibration')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = BundleCalibrationWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('apriltag_ros'), 'resource', 'BundleCalibration.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        self._widget.init()
        # Give QObjects reasonable names
        self._widget.setObjectName('Bundle Calibration UI')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        del self._widget


    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog

        
class BundleCalibrationWidget(QWidget):
    def __init__(self):
        super(BundleCalibrationWidget, self).__init__()

        self.calibrator = BundleCalibrator()
        self.bridge = CvBridge()

    def init(self):
        self.tag_sub = rospy.Subscriber("tag_detections", AprilTagDetectionArray, self.tag_detection_callback)
        self.tag_image_sub = rospy.Subscriber("tag_detections_image", Image, self.tag_image_callback)

    def tag_image_callback(self, tag_image):
        cv_img = self.bridge.imgmsg_to_cv2(tag_image)
        cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)

        frame_size = (int(self.camera_img.width()), int(self.camera_img.height()))
        
        cv_img = cv2.resize(cv_img, (frame_size))
        
        h, w, c = cv_img.shape
        bytesPerLine = 3*w

        pixmap = QPixmap.fromImage(QImage(cv_img.data, w, h, bytesPerLine, QImage.Format_RGB888))
        self.camera_img.setPixmap(pixmap)


    def tag_detection_callback(self, tag_msg):
        tags = [t.id[0] for t in tag_msg.detections if len(t.id) == 1]
        tags.sort()
        self.tags_detected.setText("Tags Detected: " + str(tags))
        self.num_calibration_points.setText("Calibration Points: " + str(len(self.calibrator.calibration_data)))
