#!/usr/bin/env python3

import rospy

import numpy as np
from std_msgs.msg import Bool, Float32, Float32MultiArray, Int32MultiArray
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Twist, Quaternion
import numpy as np

# Create a Controller class
class Command():
    def __init__(self,name):
        self.consign_lin = 0 # rpm
        self.consign_rot = 0 # rpm
        self.heading = 0
        self.nav_init = 0
        self.control_mode = 0
        self.turn = 0
        self.checkpoint_reached = 0
        self.gps_init = 0
        self.n = 0
        self.e = 0
        self.error_integral = [0,0,0,0]
        
        # Parameters
        self.rate = 20 #frequency in Hz
        self.max_speed = 5 # ms-1 : maximum linear speed
        self.max_rot = 1 # rds-1 : maximum rotational speed
        self.delta_rpm = 500 # tr/min
        self.norm_reached = 0.8
        self.turn_low  = 0.85 # low value of hysteresis on cosinus between heading and objective
        self.turn_high = 0.92 # high value of hysteresis on cosinus between heading and objective
        self.gain_turn_rot = 6
        self.gain_lin = 0.15
        self.gain_rot = 5
        self.Pgain = 1
        self.Igain = 0.2
        self.sat_error = 500
        self.min_rot = 0.3; #rd/s

        # Robot characteristics
        self.wheel_radius = 0.625/2 # m 
        self.dy = 0.955 # m : distance between front and aft wheels
        self.dx = 1.05 # m : distance between right and left wheels
        self.motor_red = 32 # reduction ratio between wheel and motor
        self.rot_conv = np.sqrt(self.dy*self.dy+self.dx*self.dx/np.cos(np.arctan(self.dx/self.dy)))
	
	    #Constants
        self.Radius = 6371
        self.rds2rpm = 60/(2*np.pi)

        self.cmd_msg = Float32MultiArray()
        self.cmd_msg.data = [0,0,0,0]
        self.gps = NavSatFix()
        self.rpm = Float32MultiArray()
        self.rpm.data = [0,0,0,0]
        self.nav_cmd = Float32MultiArray()
        self.nav_cmd.data = [0,0,0] # Waypoint : [x, y, speed]
        self.previous_cmd_vel = Twist()
        
        # Susbcriber
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_cb)
        
        # Publisher
        self.consign_left_pub = rospy.Publisher('/barakuda/consign_left', Float32, queue_size=5)
        self.consign_right_pub = rospy.Publisher('/barakuda/consign_right', Float32, queue_size=5)


    # def cmd_dir(self, cmd):
    #     self.consign_lin = min(max(cmd.data[0],-self.max_speed),self.max_speed)
    #     self.consign_rot = min(max(cmd.data[1],-self.max_rot),self.max_rot)*self.rot_conv
    #     self.control_mode = 0

    def cmd_vel_cb(self, cmd_vel): 
        self.consign_lin = min(max(cmd_vel.linear.x,-self.max_speed),self.max_speed)
        self.consign_rot = min(max(cmd_vel.angular.z,-self.max_rot),self.max_rot)*self.rot_conv

    def cmd_pub(self):

        consign_right = (self.consign_lin + self.consign_rot)/self.wheel_radius*self.motor_red*self.rds2rpm
        consign_left = (self.consign_lin - self.consign_rot)/self.wheel_radius*self.motor_red*self.rds2rpm

        if consign_right > min(self.rpm.data[0],self.rpm.data[2]):
            consign_right_lim = min(consign_right, min(self.rpm.data[0],self.rpm.data[2])+self.delta_rpm)
        else:
            consign_right_lim = max(consign_right, max(self.rpm.data[0],self.rpm.data[2])-self.delta_rpm)
        if consign_left > min(self.rpm.data[1],self.rpm.data[3]):
            consign_left_lim = min(consign_left, min(self.rpm.data[1],self.rpm.data[3])+self.delta_rpm)
        else:
            consign_left_lim = max(consign_left, max(self.rpm.data[1],self.rpm.data[3])-self.delta_rpm)

        rospy.loginfo("Left cmd: %f , Right cmd: %f", consign_left_lim, consign_right_lim)
        self.consign_left_pub.publish(consign_left_lim)
        self.consign_right_pub.publish(consign_right_lim)

if __name__ == '__main__':
    try:
        # Initiate the node
        rospy.init_node('commande_value_node', anonymous=True)
        rospy.loginfo("Command Value: Start")
        barakuda_controller = Command(rospy.get_name())
        while not rospy.is_shutdown():
            barakuda_controller.cmd_pub()
            rate=rospy.Rate(barakuda_controller.rate) #20hz
            rate.sleep()
    except rospy.ROSInterruptException:
        rospy.loginfo("Command Value Node: Stop")
