import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import pyqtSignal
from python_qt_binding.QtWidgets import QWidget, QCheckBox, QButtonGroup
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

        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('apriltag_ros'), 'resource', 'BundleCalibration.ui')

        # Create QWidget
        self._widget = BundleCalibrationWidget(ui_file)
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
        self._widget.shutdown()
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
    make_checkbox_signal = pyqtSignal()
    
    def __init__(self, ui_file):
        super(BundleCalibrationWidget, self).__init__()
        self.setObjectName('Bundle Calibration UI')
        loadUi(ui_file, self)

        self.calibrator = BundleCalibrator()
        self.bridge = CvBridge()


        self.tag_sub = rospy.Subscriber("tag_detections", AprilTagDetectionArray, self.tag_detection_callback)
        self.tag_image_sub = rospy.Subscriber("tag_detections_image", Image, self.tag_image_callback)
        self.checkboxes = QButtonGroup()
        self.checkboxes.setExclusive(False)
        self.checkbox_offset = 40
        # self.make_checkbox(0)

        # self.make_checkbox_signal = SIGNAL("changeUI(PyQt_PyObject)")
        self.make_checkbox_signal.connect(self.sync_checkboxes)
        # self.connect(self, self.make_checkbox_signal, self.make_checkbox)
        self.make_checkbox(1)
        self.make_checkbox(3)


    def shutdown(self):
        self.tag_sub = None
        self.tag_image_sub = None

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
        self.make_checkbox_signal.emit()

    def sync_checkboxes(self):
        for t in self.calibrator.get_unique_tags_seen():
            if self.checkboxes.button(t) is None:
                # self.emit(SIGNAL("changeUI(PyQt_PyObject)"), t)
                self.make_checkbox(t)

    def make_checkbox(self, id):

        checkbox = QCheckBox("Checkbox", self.tag_selection)
        # checkbox.moveToThread(QThread())
        # checkbox = QCheckBox("Checkbox")
        checkbox.setText("tag " + str(id))
        checkbox.move(0, self.checkbox_offset)
        self.checkbox_offset += 25
        checkbox.setChecked(True)
        self.checkboxes.addButton(checkbox, id)
        print("Making checkbox" + str(id))


