import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge, CvBridgeError  # Package to convert between ROS and OpenCV Images
import cv2


class Agent(Node):
    def __init__(self):
        Node.__init__(self, "Agent")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("namespace", rclpy.Parameter.Type.STRING),
                ("robot_size", rclpy.Parameter.Type.DOUBLE)
            ]
        )
        self.ns = self.get_parameter("namespace").value
        self.robot_size = self.get_parameter("robot_size").value

        self.cmd_vel = Twist()
        self.cmd_pub = self.create_publisher(Twist, f"{self.ns}/cmd_vel", 10)

        self.agent_x = self.agent_y = self.agent_yaw = None  #current position of the agent
        self.other_agent_x = self.other_agent_y = self.other_agent_yaw = None  #position of the other agent
        self.create_subscription(Odometry, f"{self.ns}/odom", self.pose_cb, 1)
        if self.ns == "bot_1":  #if our current agent is bot_1, subscribe to the other agent position topic to know where it is located
            self.create_subscription(Odometry, f"bot_2/odom", self.other_agent_pose_cb, 1)
        else:   #same but if our current agent is bot_2
            self.create_subscription(Odometry, f"bot_1/odom", self.other_agent_pose_cb, 1)

        self.create_subscription(Image, f"{self.ns}/camera/image_raw", self.image_cb, qos_profile=qos_profile_sensor_data)
        self.display_image = True
        self.br = CvBridge()    # Used to convert between ROS and OpenCV images
    

    def __del__(self):
        """ Destructor """
        cv2.destroyAllWindows()
    

    def pose_cb(self, msg):
        """ Get the current position and orientation of the agent """
        #Yaw angle expressed in [-pi/2, pi/2]
        self.agent_x = msg.pose.pose.position.x
        self.agent_y = msg.pose.pose.position.y
        self.agent_yaw = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])[2]
    

    def other_agent_pose_cb(self, msg):
        """ Get the current position and orientation of the other agent """
        #Yaw angle expressed in [-pi/2, pi/2]
        self.other_agent_x = msg.pose.pose.position.x
        self.other_agent_y = msg.pose.pose.position.y
        self.other_agent_yaw = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])[2]
    

    def image_cb(self, data):
        """ Method called whenever a new image is published on the robot's image topic """
        try:
            cv_image = self.br.imgmsg_to_cv2(data)

            #TODO: write your strategy here...
            linear = 0.0  #move the robot forward or backwards
            angular = 0.1 #make the robot rotate
            self.send_velocities(linear, angular)
            if self.agent_x is not None:
                self.get_logger().info(f"Robot pose: {self.agent_x:.2f}\t{self.agent_y:.2f}\t{self.agent_yaw:.2f}")
            if self.other_agent_x is not None:
                self.get_logger().info(f"Other agent pose: {self.other_agent_x:.2f}\t{self.other_agent_y:.2f}\t{self.other_agent_yaw:.2f}")

            # Display the resulting frame
            if self.display_image:
                cv2.imshow(f"{self.ns} frame", cv_image)   #TODO: change it to the image you want to display
                cv2.waitKey(3)

        except CvBridgeError as e:
            pass
    

    def send_velocities(self, linear, angular):
        """ Publish velocity commands to move the agent """
        self.cmd_vel.linear.x = self.constrain(linear)
        self.cmd_vel.angular.z = self.constrain(angular)
        self.cmd_pub.publish(self.cmd_vel)

    
    def constrain(self, val, min_val=-1, max_val=1):
        """ Constrain a value to range [min_val, max_val] """
        if val < min_val:
            return min_val
        elif val > max_val:
            return max_val
        return val







""" DO NOT TOUCH THIS PART!!! """
def main():
    rclpy.init()

    node = Agent()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.send_velocities(0.0, 0.0)  #stop the agent
        node.destroy_node()
        rclpy.shutdown()