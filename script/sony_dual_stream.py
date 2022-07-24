#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import warnings

warnings.filterwarnings(action='ignore')

# import re
# import os
import sys
import cv2
import time
import pyudev

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from utils.cam_prop import CAM_PROP

class webcam_capture:
    def __init__(self):
        namespace  = str(rospy.get_namespace())
        subsystem  = str(rospy.get_param(f"{namespace}/subsystem"))
        l_serial   = str(rospy.get_param(f"{namespace}/cam1_serial"))
        r_serial   = str(rospy.get_param(f"{namespace}/cam2_serial"))
        fps        = int(rospy.get_param(f"{namespace}/fps"))
        resolution = str(rospy.get_param(f"{namespace}/resolution"))
        format     = str(rospy.get_param(f"{namespace}/format"))

        self.cam1 = CAM_PROP()
        self.cam2 = CAM_PROP()

        try:
            self.cam1.set_port(self.serial2port(subsystem, l_serial))
        except Exception as e:
            rospy.logerr(e)
            rospy.logerr("Retry with the right device name.")
            rospy.logerr("Process is shutdown.")
            sys.exit(0)

        try:
            self.cam2.set_port(self.serial2port(subsystem, r_serial))
        except Exception as e:
            rospy.logerr(e)
            rospy.logerr("Retry with the right device name.")
            rospy.logerr("Process is shutdown.")
            sys.exit(0)

        self.cam1.set_fps(fps)
        self.cam2.set_fps(fps)

        try:
            self.cam1.set_resolution(resolution)
        except Exception as e:
            rospy.logwarn(e)
            rospy.logwarn("Resolution will set default value : ['1080']")

        try:
            self.cam2.set_resolution(resolution)
        except Exception as e:
            rospy.logwarn(e)
            rospy.logwarn("Resolution will set default value : ['1080']")

        try:
            self.cam1.set_format(format)
        except Exception as e:
            rospy.logwarn(e)
            rospy.logwarn("Pixel format will set default value : ['NV12']")

        try:
            self.cam2.set_format(format)
        except Exception as e:
            rospy.logwarn(e)
            rospy.logwarn("Pixel format will set default value : ['NV12']")

        try:
            self.cam1.set_capture(cv2.VideoCapture(self.cam1.get_port()))
            self.cam1.set_codec()
        except Exception as e:
            rospy.logwarn(e)

        try:
            self.cam2.set_capture(cv2.VideoCapture(self.cam2.get_port()))
            self.cam2.set_codec()
        except Exception as e:
            rospy.logwarn(e)

        self.publisher = rospy.Publisher(f"/image_raw_{namespace[1:]}", Image, queue_size=1)

        self.bridge = CvBridge()

        rospy.loginfo(f"Camera setup done. Information")
        rospy.loginfo(f"  - cam_side:         {namespace}")
        rospy.loginfo(f"  - cam_serial_left:  {l_serial}")
        rospy.loginfo(f"  - cam_serial_right: {r_serial}")
        rospy.loginfo(f"  - cam_resolution:   {self.cam1.get_capture().get(cv2.CAP_PROP_FRAME_WIDTH)} x {self.cam1.get_capture().get(cv2.CAP_PROP_FRAME_HEIGHT)}")
        rospy.loginfo(f"  - cam_FPS:          {self.cam1.get_capture().get(cv2.CAP_PROP_FPS)}")
        rospy.loginfo(f"  - cam_pixel_format: {self.cam1.get_capture().get(cv2.CAP_PROP_FOURCC)}")

        while True:
            ret1, img1 = self.cam1.get_capture().read()
            ret2, img2 = self.cam2.get_capture().read()

            img = cv2.hconcat(([img1, img2]))

            cvt_msg_format = "bgr8"

            if self.cam1.get_format() == "YUYV":
                cvt_msg_format = "bgr8"
            elif self.cam1.get_format() == "NV12":
                cvt_msg_format = "rgb8"

            msg = self.bridge.cv2_to_imgmsg(img, cvt_msg_format)

            self.publisher.publish(msg)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            time.sleep(0.01)

        self.cam1.get_capture().release()
        self.cam2.get_capture().release()

        cv2.destroyAllWindows()

    def serial2port(self, subsystem, serial):

        context = pyudev.Context()

        port = None

        for device in context.list_devices(subsystem=subsystem):
            if not port:
                if device.get("ID_SERIAL_SHORT") == serial:
                    test_port = int(device.get("DEVNAME")[-1])

                    cap = cv2.VideoCapture(test_port)

                    if cap.read()[0]:
                        port = test_port

                    cap.release()

        if port is None:
            raise Exception(
                f"[ERROR] Serial number of device is error!!! Check the device serial number. not [{serial}]\n"
                f"[ERROR] Command:\n"
                f"\t\tudevadm info --name=/dev/video*")

        return port

    """
    def dev2port(self, dev_nm):
        port = 0
        if os.path.exists(dev_nm):
            dev_path = os.path.realpath(dev_nm)
            dev_re   = re.compile("\/dev\/video(\d+)")
            dev_info = dev_re.match(dev_path)
            if dev_info:
                print(dev_info.group(0))
                print(dev_info.group(1))

                port = int(dev_info.group(1))
                rospy.loginfo(f"Using default video capture device on /dev/video{port}")
        else:
            raise Exception(f"[ERROR] Device name is error!!! Device name must have '/dev/sony-camera-left' not [{dev_nm}]")
        return port
    """


def main():

    rospy.init_node('sony_stream', anonymous=True)

    img = webcam_capture()

    try:
        rospy.spin()
    except KeyboardInterrupt as e:
        rospy.logerr(e)
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)


if __name__ == '__main__':
    main()
