import cv2
import io
import time
import imutils
import numpy as np

from PIL import Image

from src.facetracking import FaceTracking


class CameraControl(object):
    """A class for handling camera related control with Spot

    Attribute:
        spot_core: reference to SpotCore object
        _take_image: track whether to capture image
        _video_mode: track whether to caputre video
        _face_track: track whether to start face detection
        face_tracker: initialize FaceTracking object
        frame: store frame data
        camera_selection: camera sources
    """
    def __init__(self, spot_core):
        self.spot_core = spot_core
        self._take_image = False
        self._video_mode = False
        self._face_track = False

        self.face_tracker = FaceTracking("resources/haarcascade_frontalface_default.xml")
        self.frame = None
        self.camera_selection = []

    def frontL_camera_toggle(self, toggled):
        """Toggle front-left camera

        Args:
            toggled: boolean
        """
        if toggled:
            self.camera_selection.append("frontleft_fisheye_image")
        else:
            self.camera_selection.remove("frontleft_fisheye_image")
        self.set_image_source()

    def frontL_camera_toggle_depth(self, toggled):
        """Toggle front-left camera

        Args:
            toggled: boolean
        """
        if toggled:
            self.camera_selection.append("frontleft_depth")
        else:
            self.camera_selection.remove("frontleft_depth")
        self.set_image_source()

    def frontR_camera_toggle(self, toggled):
        """Toggle front-right camera

        Args:
            toggled: boolean
        """
        if toggled:
            self.camera_selection.append("frontright_fisheye_image")
        else:
            self.camera_selection.remove("frontright_fisheye_image")
        self.set_image_source()

    def frontR_camera_toggle_depth(self, toggled):
        """Toggle front-right camera

        Args:
            toggled: boolean
        """
        if toggled:
            self.camera_selection.append("frontright_depth")
        else:
            self.camera_selection.remove("frontright_depth")
        self.set_image_source()

    def left_camera_toggle(self, toggled):
        """Toggle left camera

        Args:
            toggled: boolean
        """
        if toggled:
            self.camera_selection.append("left_fisheye_image")
        else:
            self.camera_selection.remove("left_fisheye_image")
        self.set_image_source()

    def left_camera_toggle_depth(self, toggled):
        """Toggle left camera

        Args:
            toggled: boolean
        """
        if toggled:
            self.camera_selection.append("left_depth")
        else:
            self.camera_selection.remove("left_depth")
        self.set_image_source()

    def right_camera_toggle(self, toggled):
        """Toggle right camera

        Args:
            toggled: boolean
        """
        if toggled:
            self.camera_selection.append("right_fisheye_image")
        else:
            self.camera_selection.remove("right_fisheye_image")
        self.set_image_source()

    def right_camera_toggle_depth(self, toggled):
        """Toggle right camera

        Args:
            toggled: boolean
        """
        if toggled:
            self.camera_selection.append("right_depth")
        else:
            self.camera_selection.remove("right_depth")
        self.set_image_source()

    def back_camera_toggle(self, toggled):
        """Toggle back camera

        Args:
            toggled: boolean
        """
        if toggled:
            self.camera_selection.append("back_fisheye_image")
        else:
            self.camera_selection.remove("back_fisheye_image")
        self.set_image_source()

    def back_camera_toggle_depth(self, toggled):
        """Toggle back camera

        Args:
            toggled: boolean
        """
        if toggled:
            self.camera_selection.append("back_depth")
        else:
            self.camera_selection.remove("back_depth")
        self.set_image_source()

    def set_image_source(self):
        """Pass camera sources to async camera task. Disable video mode if no camera is selected"""
        if (not self.camera_selection) and self.spot_core._image_task._video_mode:
            self.spot_core._image_task.toggle_video_mode()
        else:
            self.spot_core._image_task.image_sources = self.camera_selection
        cv2.destroyAllWindows()

    def take_image(self):
        """Execute take image command in SpotCore object"""
        if not self.camera_selection:
            self.spot_core._robot.logger.info("No Camera Selected Yet")
        else:
            self._take_image = True
            self.spot_core._image_task.take_image()
            time.sleep(0.5)

    def toggle_video_mode(self):
        """Toggle video mode"""
        if not self.camera_selection:
            self.spot_core._robot.logger.info("No Camera Selected Yet")
        else:
            self.spot_core._image_task.toggle_video_mode()
            time.sleep(0.5)

    def image_rotation_correction(self, camera, index):
        """Correct image orientation

        Args:
            camera: camera source
            index: index for tracking image_data list
        """
        self.frame = np.array(Image.open(io.BytesIO(self.spot_core._image_task._image_data[index])))
        if camera == "frontright_fisheye_image":
            self.frame = imutils.rotate_bound(self.frame, 90)
        elif camera == "frontleft_fisheye_image":
            self.frame = imutils.rotate_bound(self.frame, 90)
        elif camera == "frontright_depth":
            self.frame = imutils.rotate_bound(self.frame, 90)
        elif camera == "frontleft_depth":
            self.frame = imutils.rotate_bound(self.frame, 90)
        elif camera == "right_fisheye_image":
            self.frame = imutils.rotate(self.frame, 180)
        elif camera == "right_depth":
            self.frame = imutils.rotate(self.frame, 180)

    def cv_camera_update(self):
        """Camera stream update

            Face tracking mode, video mode, or camera shot mode.
            The resulting image data is shown through OpenCV windows.
            The front left, front right, and right cameras are rotated to show correct orientation.
        """
        if self._face_track and self.spot_core._image_task._video_mode:  # Face Track
            if not self.camera_selection:
                self.spot_core._image_task.toggle_video_mode()
                self.spot_core._robot.logger.info("No Camera Selected Yet")
            else:
                index = 0
                for camera in self.camera_selection:
                    self.image_rotation_correction(camera, index)
                    self.frame = self.face_tracker.face_center(self.frame)
                    cv2.imshow(camera, self.frame)
                    index += 1
        elif self.spot_core._image_task._video_mode:  # Video Mode
            if not self.camera_selection:
                self.spot_core._image_task.toggle_video_mode()
                self.spot_core._robot.logger.info("No Camera Selected Yet")
            else:
                index = 0
                for camera in self.camera_selection:
                    self.image_rotation_correction(camera, index)
                    cv2.imshow(camera, self.frame)
                    index += 1
        elif self._take_image:  # Screenshot
            if not self.camera_selection:
                self.spot_core._robot.logger.info("No Camera Selected Yet")
            else:
                index = 0
                for camera in self.camera_selection:
                    self.image_rotation_correction(camera, index)
                    cv2.imshow(camera, self.frame)
                    index += 1
            self._take_image = False

    def toggle_face_track(self):
        """Toggle face tracking mode"""
        self._face_track = not self._face_track
        self.spot_core._image_task.toggle_video_mode()
