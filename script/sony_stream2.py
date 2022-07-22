#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import os.path
import re
import warnings
warnings.filterwarnings(action='ignore')

import sys
import cv2

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class CAM_INFO():
    def __init__(self):
        self.port = None
        self.capture = None
        self.data = None
        self.resolution = '1080'
        self.comp_data = None
        self.width = None
        self.height = None
        self.fps = None
        self.video_fourcc = None

        self.resolution_list = {"4K": [3840, 2160],
                                "2.2K": [2560, 1440],
                                "1080": [1920, 1080],
                                "720": [1280, 720]}

    def get_port(self):
        return self.port

    def set_port(self, value):
        self.port = value

    def get_capture(self):
        return self.capture

    def set_capture(self, value):
        self.capture = value

    def get_data(self):
        return self.data

    def set_data(self, value):
        self.data = value

    def get_resolution(self):
        return self.resolution

    def set_resolution(self, value):
        if value not in self.resolution_list.keys():
            raise Exception(
                f"Resolution wrong!!! Resolution must select in {self.resolution_list.keys()} not [{value}].")
        else:
            self.resolution = value

    def get_width(self):
        return self.width

    def set_width(self, value):
        self.width = value

    def get_height(self):
        return self.height

    def set_height(self, value):
        self.height = value

    def get_fps(self):
        return self.fps

    def set_fps(self, value):
        self.fps = value

    def get_comp_data(self):
        return self.comp_data

    def set_comp_data(self, value):
        self.comp_data = value

    def get_format(self):
        return self.video_fourcc

    def set_format(self, value):
        if value not in ['NV12', 'YUYV']:
            value = 'NV12'
            raise Exception(f"Pixel format wrong!!! Pixel format of camera must select in ['NV12', 'YUYV'] not [{value}]")
        self.video_fourcc = value

    def set_codec(self):
        self.get_capture().set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution_list[self.get_resolution()][0])
        self.get_capture().set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution_list[self.get_resolution()][1])
        self.get_capture().set(cv2.CAP_PROP_FPS, self.get_fps())
        self.get_capture().set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(self.get_format()[0],
                                                                           self.get_format()[1],
                                                                           self.get_format()[2],
                                                                           self.get_format()[3]))

class webcam_capture:
    def __init__(self, device_name, fps, resolution, format):

        self.cam = CAM_INFO()

        port = 0

        if isinstance(device_name, str):
            try:
                port = self.dev2port("" + device_name)
            except Exception as e:
                rospy.logerr(e)
                rospy.logerr("Retry with the right device name.")
                rospy.logerr("Process is shutdown.")
                sys.exit(0)

        self.cam.set_port(port)
        self.cam.set_fps(int(fps))

        try:
            self.cam.set_resolution(resolution)
        except Exception as e:
            rospy.logwarn(e)
            rospy.logwarn("Resolution will set default value : ['1080']")

        try:
            self.cam.set_format(format)
        except Exception as e:
            rospy.logwarn(e)
            rospy.logwarn("Pixel format will set default value : ['NV12']")

        try:
            # self.cam.set_capture(cv2.VideoCapture(self.cam.get_port()))
            self.cam.set_capture(cv2.VideoCapture(2))
            self.cam.set_codec()
        except Exception as e:
            rospy.logwarn(e)

        # try:
        #     self.publisher = rospy.Publisher(f"/image_raw_{device_name.split('-')[-1]}", Image, queue_size=1)
        # except:
        self.publisher = rospy.Publisher("/image_raw_test2", Image, queue_size=1)


        self.bridge = CvBridge()

        while True:
            ret, img = self.cam.get_capture().read()

            cvt_msg_format = "bgr8"

            if self.cam.get_format() == "YUYV":
                cvt_msg_format = "bgr8"
            elif self.cam.get_format() == "NV12":
                cvt_msg_format = "rgb8"

            msg = self.bridge.cv2_to_imgmsg(img, cvt_msg_format)

            self.publisher.publish(msg)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.cam.get_capture().release()
        cv2.destroyAllWindows()

    def dev2port(self, dev_nm):
        port = 0
        if os.path.exists(dev_nm):
            dev_path = os.path.realpath(dev_nm)
            dev_re   = re.compile("\/dev\/video(\d+)")
            dev_info = dev_re.match(dev_path)
            if dev_info:
                port = int(dev_info.group(1)) - 1
                rospy.loginfo(f"Using default video capture device on /dev/video{port}")
        else:
            raise Exception(f"[ERROR] Device name is error!!! Device name must have '/dev/sony-camera-left' not [{dev_nm}]")
        return port

def main():
    rospy.init_node('sony_stream', anonymous=True)

    # dev = rospy.get_param(f"/{rospy.get_namespace()}/device_name")
    # fps = rospy.get_param("/fps")
    # res = rospy.get_param("/resolution")
    # fmt = rospy.get_param("/format")

    dev = "/dev/sony-camera-right"
    dev = "/dev/sony-camera-left"
    # dev = 0
    fps = "60"
    res = "1080"
    fmt = "YUYV"

    img = webcam_capture(device_name=dev, fps=fps, resolution=res, format=fmt)
    # img = webcam_capture(device_name=args[0], fps=args[1], location=args[2], resolution=args[3], format=args[4])
    # img = webcam_capture(device_name=1, fps=30, resolution='4K', format='NV12')
    # img = webcam_capture(port=1, fps=60, location='left', resolution='1080', format='YUYV')
    # img = webcam_capture(device_name="/dev/sony-camera-left", fps=60, resolution='1080', format='YUYV')
    # img = webcam_capture(device_name="/dev/sony-camera-right", fps=60, location='right', resolution='1080', format='YUYV')

    try:
        rospy.spin()
    except KeyboardInterrupt as e:
        rospy.logerr(e)
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)


if __name__ == '__main__':
    main()
