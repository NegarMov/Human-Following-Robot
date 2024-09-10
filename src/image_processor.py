#!/usr/bin/python3

# Python
import copy

# Object detection
import cv2
import numpy as np
from ultralytics import YOLO
from ultralytics.yolo.utils.plotting import Annotator
from ultralytics.yolo.engine.results import Results

# ROS
import rospy
from sensor_msgs.msg import Image

# Service
from turtlebot3_object_tracker.srv import ObjectDetection, ObjectDetectionResponse

class ImageProcessor:
    def __init__(self) -> None:
        # Image message
        self.image_msg = Image()

        self.image_res = 240, 320, 3 # Camera resolution: height, width
        self.image_np = np.zeros(self.image_res) # The numpy array to pour the image data into

        self.camera_subscriber = rospy.Subscriber("/follower/camera/image", Image, self.camera_listener)

        self.model: YOLO = YOLO('../yolo/yolov8n.pt')
        
        self.results: Results = None
        # box center x, box center y, box width, box height
        self.box_info = {}

        self.cv2_frame_size = 400, 320
        cv2.namedWindow("robot_view", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("robot_view", *self.cv2_frame_size)

        self.human_detection_server = rospy.Service('ObjectDetectionServer', ObjectDetection, self.detect_object)

        self.update_view()


    def detect_object(self, req):
        # rospy.loginfo(self.box_info)

        res = ObjectDetectionResponse()

        if req.obj_class in self.box_info:
            res.found = True
            res.center_x, res.center_y, res.width, res.height = self.box_info[req.obj_class][0]
            res.img_width = self.image_res[1]
            res.img_height = self.image_res[0]
        else:
            res.found = False
        
        return res


    def camera_listener(self, msg: Image):
        self.image_msg.data = copy.deepcopy(msg.data)


    def update_view(self):
        try:
            while not rospy.is_shutdown():
                if len(self.image_msg.data) == 0: # If there is no image data
                    continue

                # Convert binary image data to numpy array
                self.image_np = np.frombuffer(self.image_msg.data, dtype=np.uint8)
                self.image_np = self.image_np.reshape(self.image_res)

                frame = copy.deepcopy(self.image_np)
                self.results = self.model(frame)
                self.box_info = {}

                annotator = Annotator(frame)
                for r in self.results:
                    boxes = r.boxes
                    for box in boxes:
                        b = box.xyxy[0]
                        cls_name = self.model.names[int(box.cls)]
                        annotator.box_label(b, cls_name)
                        
                        top_x, top_y, bottom_x, bottom_y = b
                        center_x = (top_x + bottom_x) / 2
                        center_y = (top_y + bottom_y) / 2
                        width = abs(top_x - bottom_x)
                        height = abs(top_y - bottom_y)
                        info = (center_x, center_y, width, height)
                        self.box_info.setdefault(cls_name, []).append(info)
                        
                frame = annotator.result()

                cv2.imshow("robot_view", cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
                cv2.waitKey(1)

        except rospy.exceptions.ROSInterruptException:
            pass


if __name__ == "__main__":
    rospy.init_node("image_processor", anonymous=True)

    rospy.on_shutdown(cv2.destroyAllWindows)

    image_processor = ImageProcessor()

    rospy.spin()


