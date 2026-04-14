#!/usr/bin/env python3

import rospy

import numpy as np
from std_msgs.msg import Bool, Float32, Float32MultiArray, Int32MultiArray
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Twist, Quaternion
from pymap3d import geodetic2enu
import numpy as np

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
        self.cmd_motors_pub = rospy.Publisher('/barakuda/cmd_motors', Float32MultiArray, queue_size=5)


    def manager_stop_cb(self, stop):
        self.manager_stop = stop.data

    def new_imu(self, imu_data):
        quat = imu_data.orientation
        self.heading =  np.arctan2(2*(quat.y*quat.x+quat.w*quat.z),1-2*(quat.x*quat.x + quat.z*quat.z))+0.5*np.pi
        
    def new_rpm(self, rpm):
        self.rpm = rpm

    def new_nav(self, gps):
        if gps.status.status !=-1:
            if self.gps_init == 0:
                self.ref_gps = gps
                self.gps_init = 1
            [self.e,self.n,self.u] = geodetic2enu(gps.latitude, gps.longitude, gps.altitude,self.ref_gps.latitude, self.ref_gps.longitude, self.ref_gps.altitude)

    def new_nav_cmd(self,nav_cmd):
        self.nav_cmd = nav_cmd.data
        self.control_mode = 1
        self.checkpoint_reached = 0

           
    # def cmd_dir(self, cmd):
    #     self.consign_lin = min(max(cmd.data[0],-self.max_speed),self.max_speed)
    #     self.consign_rot = min(max(cmd.data[1],-self.max_rot),self.max_rot)*self.rot_conv
    #     self.control_mode = 0

    def cmd_vel_cb(self, cmd_vel): 
        self.consign_lin = min(max(cmd_vel.linear.x,-self.max_speed),self.max_speed)
        self.consign_rot = min(max(cmd_vel.angular.z,-self.max_rot),self.max_rot)*self.rot_conv
        self.control_mode = 0

        # Reset Integral Cmd mode
        # Reset when cmd_vel is null
        
        # Update last cmd vel
        self.previous_cmd_vel = cmd_vel

    def cmd_pub(self):
        # Reset Integral Autonomous mode
        if self.control_mode == 1: 
            if self.checkpoint_reached == 0:
                vect_dir = [self.nav_cmd[0] - self.e, self.nav_cmd[1] - self.n]
                vect_norm = np.linalg.norm(vect_dir)
                if vect_norm < self.norm_reached:
                    self.checkpoint_reached = 1
                    self.consign_rot = 0
                    self.consign_lin = 0
                    self.error_integral = [0,0,0,0]

                else:
                    scalar_norm = (-np.sin(self.heading)*vect_dir[0]+np.cos(self.heading)*vect_dir[1])/vect_norm
                    vect_prod_norm = (-np.sin(self.heading)*vect_dir[1]-np.cos(self.heading)*vect_dir[0])/vect_norm
                    if (self.turn == 0 and scalar_norm < self.turn_low):
                        self.turn = 1
                        self.error_integral = [0,0,0,0]

                    if (self.turn == 1 and scalar_norm > self.turn_high):
                        self.turn = 0
                        self.error_integral = [0,0,0,0]

                
                    if self.turn == 1 :
                        self.consign_rot = np.sign(vect_prod_norm)*max(self.gain_turn_rot*np.abs(vect_prod_norm),self.min_rot*self.rot_conv)
                        self.consign_lin = 0
                    else:
                        self.consign_rot = self.gain_rot*vect_prod_norm
                        self.consign_lin = min(max(self.gain_lin*vect_norm,self.nav_cmd[2])*scalar_norm, self.max_speed)
            else:
                self.consign_rot = 0
                self.consign_lin = 0
                self.error_integral = [0,0,0,0]

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

        # Manager STOP control
        if(not self.manager_stop):
            print("MOVE OK")
            #PI controller
            for i in [0,1,2,3]:
                if i in [0,2]:
                    error = consign_right_lim-self.rpm.data[i]
                else:
                    error = consign_left_lim-self.rpm.data[i]
                new_error_int = self.error_integral[i] + self.Igain*error/self.rate
                self.error_integral[i] =  min(max(new_error_int,-self.sat_error),self.sat_error)      
                consign_motor = self.Pgain*error + self.error_integral[i]

                #command saturation
                self.cmd_msg.data[i] = int(max(min(consign_motor,5000),-5000))
        else:
            print("MOVE STOP")
            self.cmd_msg.data = [0,0,0,0]
            #self.error_integral = [0,0,0,0]     

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
