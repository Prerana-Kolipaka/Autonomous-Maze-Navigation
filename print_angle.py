import rclpy
import numpy as np
import math
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist, Quaternion, Point
from nav_msgs.msg import Odometry
from tf_transformations import *

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.odom_sub = self.create_subscription(
                Odometry,
                '/odom',
                self.pose_callback,
                10)
        self.Init = True
        self.Init_pos = Point()
        self.Init_pos.x = 0.0
        self.Init_pos.y = 0.0
        self.Init_ang = 0.0 
        self.globalPos = Odometry()
        self.odom_sub  # prevent unused variable warning

    def pose_callback(self, msg):
        Odom = msg
        position = Odom.pose.pose.position

        #Orientation uses the quaternion aprametrization.
        #To get the angular position along the z-axis, the following equation is required.
        q = Odom.pose.pose.orientation
        orientation = np.arctan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

        if self.Init:
            #The initial data is stored to by subtracted to all the other values as we want to start at position (0,0) and orientation 0
            self.Init = False
            self.Init_ang = orientation
            self.globalAng = self.Init_ang
            Mrot = np.matrix([[np.cos(self.Init_ang), np.sin(self.Init_ang)],[-np.sin(self.Init_ang), np.cos(self.Init_ang)]])        
            self.Init_pos.x = Mrot.item((0,0))*position.x + Mrot.item((0,1))*position.y
            self.Init_pos.y = Mrot.item((1,0))*position.x + Mrot.item((1,1))*position.y
            self.Init_pos.z = position.z
        Mrot = np.matrix([[np.cos(self.Init_ang), np.sin(self.Init_ang)],[-np.sin(self.Init_ang), np.cos(self.Init_ang)]])        

        #We subtract the initial values
        #self.globalPos.x = Mrot.item((0,0))*position.x + Mrot.item((0,1))*position.y - self.Init_pos.x
        #self.globalPos.y = Mrot.item((1,0))*position.x + Mrot.item((1,1))*position.y - self.Init_pos.y
        #self.globalAng = orientation - self.Init_ang
        
        self.globalPos.pose.pose.position.x = Mrot.item((0,0))*position.x + Mrot.item((0,1))*position.y - self.Init_pos.x
        self.globalPos.pose.pose.position.y = Mrot.item((1,0))*position.x + Mrot.item((1,1))*position.y - self.Init_pos.y
        q = quaternion_from_euler(0,0,orientation-self.Init_ang)
        self.globalPos.pose.pose.orientation.x = q[0]
        self.globalPos.pose.pose.orientation.y = q[1]
        self.globalPos.pose.pose.orientation.z = q[2]
        self.globalPos.pose.pose.orientation.w = q[3]
        curAngle_q = [self.globalPos.pose.pose.orientation.x, self.globalPos.pose.pose.orientation.y, self.globalPos.pose.pose.orientation.z, self.globalPos.pose.pose.orientation.w]

        globalAngle = list(euler_from_quaternion(curAngle_q))
        #if globalAngle[2] < 0:
            #globalAngle[2] = globalAngle[2]+ 2*math.pi    
        self.get_logger().info('I heard: "%s"' % globalAngle[2])


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
