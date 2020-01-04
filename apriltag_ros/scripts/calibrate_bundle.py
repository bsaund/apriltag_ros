#!/usr/bin/env python

import sys

from apriltag_ros.my_module import BundleCalibration
from rqt_gui.main import Main

plugin = 'apriltag_ros'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))

