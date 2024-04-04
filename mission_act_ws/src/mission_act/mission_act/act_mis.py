#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose 
from nav2_simple_commander.robot_navigator import BasicNavigator
from action_msgs.msg import GoalStatus
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from AprilTagModule import Detector
import tf_transformations
import time

class InitialPose(Node):
    def __init__(self):
        super().__init__('missions_node')
        self.publisher = self.create_publisher(
            PoseStamped,
            'goal_pose',
            10
        )
        self.nav = BasicNavigator()
        inital_pose= self.create_pose_stamped(0.0,0.0,0.0)
        self.nav.setInitialPose(inital_pose)
        self.get_logger().info("SET INITAL POSITION.")
        time.sleep(4.0)

    def create_pose_stamped(self, pose_x, pose_y, orientation_z):
        q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, orientation_z)
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.nav.get_clock().now().to_msg()
        pose.pose.position.x = pose_x
        pose.pose.position.y = pose_y
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = q_x
        pose.pose.orientation.y = q_y
        pose.pose.orientation.z = q_z
        pose.pose.orientation.w = q_w
        return pose
    
    def publish_goal_pose(self, pose):
        self.publisher.publish(pose)

    def mission0(self):
        # goal_pose = self.create_pose_stamped(2.5, 0.0, 1.57)
        # self.publish_goal_pose(goal_pose)
        self.get_logger().info("MISSION 0: Goal pose published")


class AprilTagDetector(Node):
    def __init__(self):
        super().__init__('april_tag_detector')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.bridge = CvBridge()
        self.tag_detector = Detector()
        self.pose_class =InitialPose()

        self.detector_flag=False
        #self.missions = Missions()
        #self.pose_publish_FLAG = False

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        detections = self.tag_detector.detect(gray)

        if not detections:
            pass
            #self.get_logger().info("EMPTY LIST")
        else:
            for tag in detections:
                tag_id = tag.tag_id
                self.get_logger().info(str(tag_id))
                self.get_logger().info('Image Callback')

                if tag_id==1:
                    self.get_logger().info('Mission 0 Detected')
                    self.pose_class.mission0()
                    

class Navigate(Node):
    def __init__(self):
        super().__init__('missions_act_node')
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_to_pose_client.wait_for_server()
        self.tag_detect=AprilTagDetector()

    def moveToGoal(self,xGoal,yGoal,wGoal ):
        goal_msg = NavigateToPose.Goal()
        
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = xGoal
        pose.pose.position.y = yGoal
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = wGoal
        pose.pose.orientation.z = 0.0

        goal_msg.pose = pose

        self.get_logger().info('Sending goal location...')
        self._send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)


    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')

        while rclpy.ok():
            status = goal_handle._status
            self.get_logger().info(str(status))

            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info('Goal succeeded!')
                break
            elif status == GoalStatus.STATUS_EXECUTING:
                self.get_logger().info('Robot Going To Goal!')
                rclpy.spin_once(self.tag_detect)

            elif status == GoalStatus.STATUS_CANCELED:
                self.get_logger().info('Goal canceled')
                break
            elif status == GoalStatus.STATUS_ABORTED:
                self.get_logger().info('Goal aborted')
                break
            
            rclpy.spin_once(self)


def main(args=None):
    rclpy.init(args=args)

    navigate_to_pose = Navigate()

    navigate_to_pose.moveToGoal(1.5, 0.0, 1.0)

    rclpy.spin(navigate_to_pose)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
