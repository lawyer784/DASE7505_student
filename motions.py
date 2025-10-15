# Imports
import rclpy
from rclpy.node import Node
from utilities import Logger, euler_from_quaternion
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# TODO Part 3: Import message types needed: 
    # For sending velocity commands to the robot: Twist
    # For the sensors: Imu, LaserScan, and Odometry
# Check the online documentation to fill in the lines below
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from rclpy.time import Time

# You may add any other imports you may need/want to use below
import math
import time


CIRCLE=0; SPIRAL=1; ACC_LINE=2
motion_types=['circle', 'spiral', 'line']

class motion_executioner(Node):
    
    def __init__(self, motion_type=0):
        
        super().__init__("motion_types")
        
        self.type=motion_type
        
        self.radius_=0.0
        self.start_time = time.time()
        
        self.successful_init=False
        self.imu_initialized=False
        self.odom_initialized=False
        self.laser_initialized=False
        
        # TODO Part 3: Create a publisher to send velocity commands by setting the proper parameters in (...)
        self.vel_publisher=self.create_publisher(Twist, '/cmd_vel', 10)
        # syntax: self.create_publisher(MessageType, 'topic_name', queue_size)      
        # loggers
        self.imu_logger=Logger('imu_content_'+str(motion_types[motion_type])+'.csv', headers=["acc_x", "acc_y", "acc_z", "angular_x", "angular_y", "angular_z", "orientation_x", "orientation_y", "orientation_z", "orientation_w", "stamp"])
        self.odom_logger=Logger('odom_content_'+str(motion_types[motion_type])+'.csv', headers=["position_x","position_y","position_z", "orientation_x", "orientation_y", "orientation_z", "orientation_w", "linear_x", "linear_y", "linear_z", "angular_x", "angular_y", "angular_z", "stamp"])
        self.laser_logger=Logger('laser_content_'+str(motion_types[motion_type])+'.csv', headers=["range_min", "range_max", "angle_min", "angle_max", "angle_increment", "ranges_count", "ranges_data", "stamp"])
        
        # TODO Part 3: Create the QoS profile by setting the proper parameters in (...)
        qos=QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # TODO Part 5: Create below the subscription to the topics corresponding to the respective sensors
        # IMU subscription
        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            qos
        )
        
        # ENCODER subscription
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos
        )
        
        # LaserScan subscription 
        self.laser_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            qos
        )
        
        self.create_timer(0.1, self.timer_callback)


    # TODO Part 5: Callback functions: complete the callback functions of the three sensors to log the proper data.
    # To also log the time you need to use the rclpy Time class, each ros msg will come with a header, and then
    # inside the header you have a stamp that has the time in seconds and nanoseconds, you should log it in nanoseconds as 
    # such: Time.from_msg(imu_msg.header.stamp).nanoseconds
    # You can save the needed fields into a list, and pass the list to the log_values function in utilities.py

    def imu_callback(self, imu_msg: Imu):
        # log imu msgs
        timestamp = Time.from_msg(imu_msg.header.stamp).nanoseconds
        
        imu_data = [
            imu_msg.linear_acceleration.x,
            imu_msg.linear_acceleration.y,
            imu_msg.linear_acceleration.z,
            imu_msg.angular_velocity.x,
            imu_msg.angular_velocity.y,
            imu_msg.angular_velocity.z,
            imu_msg.orientation.x,
            imu_msg.orientation.y,
            imu_msg.orientation.z,
            imu_msg.orientation.w,
            timestamp
        ]
        
        self.imu_logger.log_values(imu_data)
        self.imu_initialized = True
        
    def odom_callback(self, odom_msg: Odometry):
        # log odom msgs
        timestamp = Time.from_msg(odom_msg.header.stamp).nanoseconds
        
        odom_data = [
            odom_msg.pose.pose.position.x,
            odom_msg.pose.pose.position.y,
            odom_msg.pose.pose.position.z,
            odom_msg.pose.pose.orientation.x,
            odom_msg.pose.pose.orientation.y,
            odom_msg.pose.pose.orientation.z,
            odom_msg.pose.pose.orientation.w,
            odom_msg.twist.twist.linear.x,
            odom_msg.twist.twist.linear.y,
            odom_msg.twist.twist.linear.z,
            odom_msg.twist.twist.angular.x,
            odom_msg.twist.twist.angular.y,
            odom_msg.twist.twist.angular.z,
            timestamp
        ]
        
        self.odom_logger.log_values(odom_data)
        self.odom_initialized = True
                
    def laser_callback(self, laser_msg: LaserScan):
        # log laser msgs with position msg at that time
        timestamp = Time.from_msg(laser_msg.header.stamp).nanoseconds
        
        # Convert ranges array to string for CSV storage
        ranges_str = ';'.join(map(str, laser_msg.ranges))
        
        laser_data = [
            laser_msg.range_min,
            laser_msg.range_max,
            laser_msg.angle_min,
            laser_msg.angle_max,
            laser_msg.angle_increment,
            len(laser_msg.ranges),
            ranges_str,
            timestamp
        ]
        
        self.laser_logger.log_values(laser_data)
        self.laser_initialized = True
                
    def timer_callback(self):
        
        if self.odom_initialized and self.laser_initialized and self.imu_initialized:
            self.successful_init=True
            
        if not self.successful_init:
            return
        
        cmd_vel_msg=Twist()
        
        if self.type==CIRCLE:
            cmd_vel_msg=self.make_circular_twist()
        
        elif self.type==SPIRAL:
            cmd_vel_msg=self.make_spiral_twist()
                        
        elif self.type==ACC_LINE:
            cmd_vel_msg=self.make_acc_line_twist()
            
        else:
            print("type not set successfully, 0: CIRCLE 1: SPIRAL and 2: ACCELERATED LINE")
            raise SystemExit 

        self.vel_publisher.publish(cmd_vel_msg)
        
    
    # TODO Part 4: Motion functions: complete the functions to generate the proper messages corresponding to the desired motions of the robot

    def make_circular_twist(self):
        msg=Twist()
        # Circular motion: constant linear and angular velocity
        msg.linear.x = 0.2  # m/s
        msg.angular.z = 0.5  # rad/s
        return msg

    def make_spiral_twist(self):
        msg=Twist()
        # Spiral motion: constant linear velocity, increasing angular velocity over time
        elapsed_time = time.time() - self.start_time
        # Start with small angular velocity and increase over time
        base_angular = 0.3
        increasing_angular = min(0.7, base_angular + elapsed_time * 0.05)
        
        msg.linear.x = 0.15  # m/s
        msg.angular.z = increasing_angular  # rad/s
        return msg
    
    def make_acc_line_twist(self):
        msg=Twist()
        # Accelerated line motion: increasing linear velocity, no angular velocity
        elapsed_time = time.time() - self.start_time
        # Start slow and accelerate
        linear_speed = min(0.4, 0.1 + elapsed_time * 0.05)
        
        msg.linear.x = linear_speed  # m/s
        msg.angular.z = 0.0  # rad/s
        return msg

import argparse

if __name__=="__main__":
    
    argParser=argparse.ArgumentParser(description="input the motion type")
    argParser.add_argument("--motion", type=str, default="circle")

    rclpy.init()
    args = argParser.parse_args()

    if args.motion.lower() == "circle":
        ME=motion_executioner(motion_type=CIRCLE)
    elif args.motion.lower() == "line":
        ME=motion_executioner(motion_type=ACC_LINE)
    elif args.motion.lower() =="spiral":
        ME=motion_executioner(motion_type=SPIRAL)
    else:
        print(f"we don't have {args.motion.lower()} motion type")
        rclpy.shutdown()
        exit(1)
    
    try:
        rclpy.spin(ME)
    except KeyboardInterrupt:
        print("Exiting")
    finally:
        ME.destroy_node()
        rclpy.shutdown()
