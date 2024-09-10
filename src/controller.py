#!/usr/bin/python3

# ROS
import rospy
from geometry_msgs.msg import Twist

# Service
from turtlebot3_object_tracker.srv import ObjectDetection, ObjectDetectionRequest


class Controller:
    def __init__(self) -> None:
        # Use these Twists to control your robot
        self.move = Twist()

        # The "p" parameter for your p-controller
        self.angular_vel_coef = 0.0055
        self.linear_speed = 0.2
        self.angular_speed = 0.4

        # Bounding box info
        self.center_x = 0
        self.obj_ratio = 0
        self.img_width = 0
        self.max_obj_ratio = 0.8

        # Possible states enum
        self.FOLLOW, self.ROTATE = 0, 1
        self.state = self.ROTATE

        self.server_call = rospy.ServiceProxy('ObjectDetectionServer', ObjectDetection)
        
        self.cmd_vel = rospy.Publisher('/follower/cmd_vel', Twist, queue_size=5)


    def get_detected_object(self):
        try:
            req = ObjectDetectionRequest()
            req.obj_class = 'person'

            res = self.server_call(req)

            if res.found == False:
                return False

            self.center_x = res.center_x
            self.obj_ratio = res.height / res.img_height
            self.img_width = res.img_width

            return True

        except rospy.ServiceException as error:
            # rospy.logerr(f"Mission service call failed: {error}")
            pass
    

    def run(self) -> None:
        try:
            while not rospy.is_shutdown():
                self.cmd_vel.publish(self.move)

                obj_detected = self.get_detected_object()

                if self.state == self.FOLLOW:
                    if not obj_detected:
                        self.move = Twist()
                        self.state = self.ROTATE
                        rospy.loginfo("Changed state to ROTATE")
                        continue

                    self.move = Twist()

                    # Don't get too close to the object
                    if self.obj_ratio < self.max_obj_ratio:
                        self.move.linear.x = self.linear_speed

                    err = (self.img_width / 2) - self.center_x
                    self.move.angular.z = err * self.angular_vel_coef

                else:
                    if obj_detected:
                        self.move = Twist()
                        self.state = self.FOLLOW
                        rospy.loginfo("Changed state to FOLLOW")
                        continue
                    self.move = Twist()
                    self.move.angular.z = self.angular_speed

        except rospy.exceptions.ROSInterruptException:
            pass
                

if __name__ == "__main__":
    rospy.init_node("controller", anonymous=True)

    rospy.sleep(3)
    
    controller = Controller()
    controller.run()
    

