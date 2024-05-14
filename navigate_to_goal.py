import rclpy
import time
import numpy as np
import math
import copy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist, Quaternion, Point
from nav_msgs.msg import Odometry
from tf_transformations import *
from statistics import mode

class Navigate(Node):

    def __init__(self):
        super().__init__('navigate')
        self.Init = True
        self.Init_pos = Point()
        self.Init_pos.x = 0.0
        self.Init_pos.y = 0.0
        self.Init_ang = 0.0
        self.sign_sub = self.create_subscription(
            String,
            '/sign_desc',
            self.sign_callback,
            10)
        self.sign_sub  # prevent unused variable warning
        self.dist_sub = self.create_subscription(
            Float32,
            '/dist_val',
            self.dist_callback,
            10)
        self.cur_dist = float("inf")
        self.odom_sub = self.create_subscription(
                Odometry,
                '/odom',
                self.pose_callback,
                10)
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub  # prevent unused variable warning
        self.currPos = Odometry()
        self.globalPos = Odometry()
        self.goalAngle = 0.0
        self.state = 0
        self.prev_sign = ''
        self.executing_behaviour = False
        self.buffer = []
        self.prev_global_ang = 0.0
        self.init_time = time.time()
        self.prev_pos = Odometry()
        
        
    def sign_callback(self, msg):
        sign = msg.data
        #self.get_logger().info('I heard: "%s"' % msg.data)
        #self.get_logger().info('Buffer length: "%s"' % len(self.buffer))
        
        if not self.executing_behaviour:
            if len(self.buffer) <= 10:
                
                self.buffer.append(sign)
            else:
                sign = mode(self.buffer)
                self.buffer = []
                #or self.prev_sign == 'u_turn' or self.prev_sign == 'stop'
                if self.prev_sign == 'left' :
                    correction = -0.08
                elif self.prev_sign == 'right' :
                    correction = 0.105
                else:
                    correction = 0
                    
                if sign=='wall':
                    self.state = 6
                    #self.goalAngle = self.globalAngle[2] + 1.62 + correction        
                    #if self.goalAngle >= 2*math.pi:
                     #   self.goalAngle -= 2*math.pi
                    
                elif sign=='left':
                    self.state = 2
                    self.goalAngle = self.globalAngle[2] + 1.62 + correction        
                    if self.goalAngle >= 2*math.pi:
                        self.goalAngle -= 2*math.pi
                        
                elif sign=='right':
                    self.goalAngle = self.globalAngle[2] - 1.58 + correction    
                    if self.goalAngle < 0:
                        self.goalAngle=2*math.pi + self.goalAngle
                    self.state = 3
                    
                elif sign=='u_turn':
                    self.goalAngle = self.globalAngle[2] + 3.12 + correction      
                    if self.goalAngle >= 2*math.pi:
                        self.goalAngle -= 2*math.pi
                    self.state = 4
                    
                elif sign=='stop':
                    self.goalAngle = self.globalAngle[2] + 3.12 + correction      
                    if self.goalAngle >= 2*math.pi:
                        self.goalAngle -= 2*math.pi
                    self.state = 5
                    
                        
                elif sign=='goal':
                    self.state = 7
                self.prev_sign = sign

    def velocity_controller(self,linear,angular):
    
        vel = Twist()
        vel.linear.x = linear
        vel.linear.y = 0.0
        vel.linear.z = 0.0
        vel.angular.x = 0.0
        vel.angular.y = 0.0
        vel.angular.z = angular
        self.vel_publisher.publish(vel)
            
    def move_forward(self):
        if self.cur_dist > 0.55:           
            self.velocity_controller(0.12,0.0)
        else:            
            self.velocity_controller(0.0,0.0)
        
    def turn_left(self):
        
        self.get_logger().info('Angle difference "%s"' % abs(self.globalAngle[2] - self.goalAngle))
        if abs(self.globalAngle[2] - self.goalAngle) >= 0.05:
            self.velocity_controller(0.0, 0.4)
            return False
        else:
            self.velocity_controller(0.0, 0.0)
            return True
    
    def turn_right(self):
        
        self.get_logger().info('Angle difference "%s"' % abs(self.globalAngle[2] - self.goalAngle))
        
        if abs(self.globalAngle[2] - self.goalAngle) >= 0.05:
            self.velocity_controller(0.0, -0.4)
            return False
        else:
            self.velocity_controller(0.0, 0.0)
            return True
            
    def euclidean_dist(self):
        return math.sqrt((self.globalPos.pose.pose.position.x - self.prev_pos.pose.pose.position.x)**2 + (self.globalPos.pose.pose.position.y - self.prev_pos.pose.pose.position.y)**2)
        
    def turn_around(self):        
        
        self.get_logger().info('Angle difference "%s"' % abs(self.globalAngle[2] - self.goalAngle))
        if abs(self.globalAngle[2] - self.goalAngle) >= 0.05:
            self.velocity_controller(0.0, 0.4)
            return False
        else:
            self.velocity_controller(0.0, 0.0)
            return True
              
    def navigate_robot(self):
        self.get_logger().info('Current state "%s"' % self.state)
        if self.state == 0:
            self.state = 1
            #self.move_forward()
            
        elif self.state == 1:
            self.move_forward()
            
            dis = self.euclidean_dist()
            
            #self.get_logger().info('global pos "%s %s"' % (self.prev_pos.pose.pose.position.x, self.prev_pos.pose.pose.position.y))
            #self.get_logger().info('prev pos "%s %s"' % (self.prev_pos.pose.pose.position.x, self.prev_pos.pose.pose.position.y))
            #self.get_logger().info('dist "%s"' % dis)
            if dis > 0.2 and self.cur_dist > 0.6:
                
                target = self.goalAngle * 180 / math.pi
                current = self.globalAngle[2] * 180 / math.pi
                self.get_logger().info('target angle "%s"' % target)
                self.get_logger().info('current angle "%s"' % current)
                diff_clockwise = (target - current)  %360
                diff_anti = (current - target) %360  
                if diff_clockwise >=2 and diff_anti >=2:      
                    if diff_clockwise <= diff_anti:
                        self.goalAngle = self.globalAngle[2] + (diff_clockwise * math.pi /180)
                        if self.goalAngle >= 2*math.pi:
                            self.goalAngle -= 2*math.pi
                        self.state = 2
                    else:
                        self.goalAngle = self.globalAngle[2] - (diff_clockwise * math.pi /180)
                        if self.goalAngle <= 0:
                            self.goalAngle=2*math.pi + self.goalAngle
                        self.state=3
                    
        elif self.state == 2:
            self.executing_behaviour = True            
            res = self.turn_left()
            if res:
                self.executing_behaviour = False
                time.sleep(1)                
                self.state = 1
                self.prev_pos = copy.deepcopy(self.globalPos)
            
        elif self.state == 3:
            self.executing_behaviour = True            
            res = self.turn_right()
            if res:
                self.executing_behaviour = False
                time.sleep(1)                
                self.state = 1
                
                self.prev_pos = copy.deepcopy(self.globalPos)
        
        elif self.state == 4:
            self.executing_behaviour = True            
            res = self.turn_around()
            if res:
                self.executing_behaviour = False
                time.sleep(1)                
                self.state = 1
                self.prev_pos = copy.deepcopy(self.globalPos)
        
        elif self.state == 5:
            time.sleep(3)
            self.state = 4
            
        elif self.state == 8:
        
            self.executing_behaviour = True
            res = self.turn_left()
            if res:
                self.executing_behaviour = False
                time.sleep(1)                
                self.state = 1
                self.prev_pos = copy.deepcopy(self.globalPos)
            
        elif self.state == 7:
            self.get_logger().info('Goal Reached!')
            raise SystemExit
        
        
    
    def dist_callback(self, dist):
        
        self.cur_dist = dist.data
        
    def pose_callback(self, pose):
        
        self.update_Odometry(pose)
        curr_time = time.time()
        if(curr_time - self.init_time >5):
            self.navigate_robot()
        

    def update_Odometry(self,Odom):
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
        
        self.globalAngle = list(euler_from_quaternion(curAngle_q))
        
        if self.globalAngle[2] < 0:
            self.globalAngle[2] = self.globalAngle[2]+ 2*math.pi
        #self.get_logger().info('current global angle "%s"' % self.globalAngle[2])
        #self.get_logger().info('prev global angle "%s"' % self.prev_global_ang)
        curr_time = time.time()
        if curr_time - self.init_time >4 and curr_time - self.init_time <5:
            self.prev_pos = copy.deepcopy(self.globalPos)
        
        if(curr_time - self.init_time >10):
            if 4.5 > abs(self.prev_global_ang - self.globalAngle[2]) and abs(self.prev_global_ang - self.globalAngle[2]) > 0.6 :
                self.goalAngle = self.prev_global_ang -0.07
                
                if self.goalAngle >= 2*math.pi:
                            self.goalAngle -= 2*math.pi  
                
                self.get_logger().info('setting goal angle "%s"' % self.goalAngle)
                self.state = 8
                time.sleep(3)
            
        self.prev_global_ang = self.globalAngle[2]
            
            
        
       
def main(args=None):
    rclpy.init(args=args)

    navigate = Navigate()

    try:
        rclpy.spin(navigate)
    except SystemExit:
        rclpy.logging.get_logger("Quitting").info('Back to Main')

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    navigate.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
