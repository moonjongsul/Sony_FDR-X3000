import cv2


class CAM_PROP():
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
            raise Exception(
                f"Pixel format wrong!!! Pixel format of camera must select in ['NV12', 'YUYV'] not [{value}]")
        self.video_fourcc = value

    def set_codec(self):
        self.get_capture().set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution_list[self.get_resolution()][0])
        self.get_capture().set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution_list[self.get_resolution()][1])
        self.get_capture().set(cv2.CAP_PROP_FPS, self.get_fps())
        self.get_capture().set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(self.get_format()[0],
                                                                           self.get_format()[1],
                                                                           self.get_format()[2],
                                                                           self.get_format()[3]))