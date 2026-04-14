#!/usr/bin/env python3

import rospy

import numpy as np
from std_msgs.msg import Bool, Float32, Float32MultiArray, Int32MultiArray
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Twist, Quaternion
from pymap3d import geodetic2enu
import numpy as np
import math

# Create a Controller class
class Controller():
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

        self.merde = 0
        self.val = 0

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

        # Manager safety stop boolean
        self.manager_stop = True
        
        # Susbcriber
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_cb)
        rospy.Subscriber('/barakuda/motor_rpm', Float32MultiArray, self.new_rpm)
        rospy.Subscriber('/imu/data',Imu, self.new_imu)
        rospy.Subscriber('/imu/nav_sat_fix',NavSatFix, self.new_nav)
        rospy.Subscriber('/barakuda/nav_cmd', Float32MultiArray, self.new_nav_cmd)
        rospy.Subscriber("/safety/manager_stop", Bool, self.manager_stop_cb)
        #rospy.Subscriber("/barakuda/cmd_dir", Float32MultiArray, self.cmd_dir)

        # Publisher
        self.cmd_motors_pub = rospy.Publisher('/cmd_rpm', Float32MultiArray, queue_size=5)

    def cmd_pub(self):
        
        # Echellon 0->lim, lim, lim->0
        lim = 1000

        if(self.merde < lim):
            self.val = self.merde
        
        if(self.merde > lim and self.merde < (2*lim)):
            self.val = lim
        
        if(self.merde > (2*lim) and self.merde < (3*lim)):
            self.val = 3*lim - self.merde

        if(self.merde >= (3*lim)):
           self.merde = 0

        self.merde = self.merde + (lim/100)

        print(self.val)
        
        # Choose if rotation or linear
        val_g = self.val
        val_d = -self.val

        self.cmd_msg.data = [val_d, val_g, val_d, val_g]

        # Pub
        self.cmd_motors_pub.publish(self.cmd_msg)

if __name__ == '__main__':
    try:
        # Initiate the node
        rospy.init_node('controller_node', anonymous=True)
        rospy.loginfo("Controller Node: Start")
        barakuda_controller = Controller(rospy.get_name())
        while not rospy.is_shutdown():
            barakuda_controller.cmd_pub()
            rate=rospy.Rate(barakuda_controller.rate) #20hz
            rate.sleep()
    except rospy.ROSInterruptException:
        rospy.loginfo("Controller Node: Stop")
