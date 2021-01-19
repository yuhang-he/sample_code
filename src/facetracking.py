import imutils
import cv2


class FaceTracking(object):
    def __init__(self, haarPath):
        self.detector = cv2.CascadeClassifier(haarPath)
        self.obj_location = None

    def update(self, frame, frameCenter):
        """Face detection

        Args:
            frame: image frame
            frameCenter: frame center location
        """
        # convert the frame to grayscale
        # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # cv2.imshow("Gray", gray)
        # detect all faces in the input frame
        rects = self.detector.detectMultiScale(frame, scaleFactor=1.05, minNeighbors=9, minSize=(30, 30),
                                               flags=cv2.CASCADE_SCALE_IMAGE)

        # check to see if a face was found
        if len(rects) > 0:
            # extract the bounding box coordinates of the face
            # and use the coordinates to determine the center of the face
            (x, y, w, h) = rects[0]
            faceX = int(x + (w / 2.0))
            faceY = int(y + (h / 2.0))

            # return the center (x, y) coordinates of the face
            return ((faceX, faceY), rects[0])

        return (frameCenter, None)

    def face_center(self, frame):
        """Calculate center of face and draw rectangle over the detected face

        Args:
            frame: image frame
        """
        (H, W) = frame.shape[:2]
        centerX = W / 2.0
        centerY = H / 2.0
        # print("x: {} y: {}".format(centerX, centerY))
        # cv2.imshow("color", frame)
        self.obj_location = self.update(frame, (centerX, centerY))
        ((objX, objY), rect) = self.obj_location

        if rect is not None:
            (x, y, w, h) = rect
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        return frame
