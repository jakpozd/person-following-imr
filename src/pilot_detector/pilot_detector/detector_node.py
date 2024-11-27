import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from pilot_detector.detector import PilotDetector


class PilotDetectorNode(Node):
    def __init__(self):
        super().__init__('pilot_detector')
        self.detector = PilotDetector()
        self.publisher = self.create_publisher(PoseStamped, 'pilot_position', 10)

        detection_timer_period = 0.1
        self.detection_timer = self.create_timer(detection_timer_period, self.detection_timer_callback)

        publication_timer_period = 0.1
        self.pos_publication_timer = self.create_timer(publication_timer_period, self.pos_publication_timer_callback)

        self.distance = None
        self.angle = None

    def detection_timer_callback(self):
        self.distance, self.angle = self.detector.detect()

    def pos_publication_timer_callback(self):
        goal_msg = PoseStamped()
        #check for detection
        if (self.distance != None) and (self.angle != None):
            # angle already comes in radians from detector.py
            goal_msg.pose.orientation.z = self.angle
            goal_msg.pose.position.x = self.distance

            self.publisher.publish(goal_msg)

        else:
            goal_msg.pose.orientation.z = np.nan
            goal_msg.pose.position.x = np.nan
            self.publisher.publish(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    pilot_detector_node = PilotDetectorNode()
    rclpy.spin(pilot_detector_node)
    pilot_detector_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
