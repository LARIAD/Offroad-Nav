#!/usr/bin/env python3

import rospy

import numpy as np
from std_msgs.msg import Bool, Float32, Float32MultiArray, Int32MultiArray
from sensor_msgs.msg import NavSatFix, Imu 
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion
from pymap3d import geodetic2enu
import numpy as np
import math

# Dynamic reconfigure imports
from dynamic_reconfigure.server import Server
from barakuda_node.cfg import PIDConfig

#Import PID controller
from pid_controller import *
from intelligent_pid_controller import *

# Create a Controller class
class Controller():

    # -- Class Cosntructor -- 

    def __init__(self,name):
       
        # Parameters
        self.rate = 20 #frequency in Hz

        # Robot characteristics
        self.wheel_radius = 0.625/2 # m 
        self.dy = 0.955 # m : distance between front and aft wheels
        self.dx = 1.05 # m : distance between right and left wheels
        self.motor_red = 32 # reduction ratio between wheel and motor
        self.rot_conv = np.sqrt(self.dy*self.dy+self.dx*self.dx/np.cos(np.arctan(self.dx/self.dy)))
	
	    # Constants
        self.Radius = 6371
        self.max_rpm = 5000
        self.rds2rpm = 60/(2*np.pi)

        self.dist_center_wheel = 0.7117056976025975
        self.theta = 0.8400699948165985
        self.sin_theta = math.sin(self.theta)
        self.cos_theta =math.cos(self.theta)

        
        self.average_window_barakuda = rospy.get_param("average_window_barakuda", 5)
        self.average_window_imu = rospy.get_param("average_window_imu", 2)

        self.torque_dead_zone = rospy.get_param("torque_dead_zone", 0.01)
        self.gain_rpm = rospy.get_param("gain_rpm", 1)

        # Variables
        self.cmd_vel = Twist() # Twist command target

        self.cmd_rpm = Float32MultiArray() # RPM command target
        self.cmd_rpm.data = [0,0,0,0]

        self.rpm_encoder = Float32MultiArray() # Actual RPM from the barakuda encoders
        self.rpm_encoder.data = [0,0,0,0]

        self.torque = Float32MultiArray()
        self.torque.data = [0,0,0,0]
        
        # -- Average odometry variables
        self.odom = Odometry()
        self.odom_lin = Float32()
        self.odom_rot = Float32()
        self.average_odom_lin = [0] * self.average_window_imu
        self.average_odom_rot = [0] * self.average_window_imu

        self.gps = NavSatFix()

        self.cmd_rpm_thresh = Float32MultiArray() # RPM command with a 5000 RPM threshold
        self.cmd_rpm_thresh.data = [0,0,0,0]

        self.model_rpm = Float32MultiArray() # RPM from the model
        self.model_rpm.data = [0,0,0,0]

        self.cmd_out = Float32MultiArray() # RPM Published to the barakuda_node
        self.cmd_out.data = [0,0,0,0]

        self.average_torque = [[0] * self.average_window_barakuda,[0] * self.average_window_barakuda,[0] * self.average_window_barakuda,[0] * self.average_window_barakuda]
        self.average_rpm = [[0] * self.average_window_barakuda,[0] * self.average_window_barakuda,[0] * self.average_window_barakuda,[0] * self.average_window_barakuda]

        # -- motor_rpm PID 

        self.torque_gain_value = 50 # Default value at t=0 for the torque gain
        self.max_torque_gain = 100 # Torque gain max value
        self.torque_gain = Float32MultiArray()
        self.torque_gain.data = [self.torque_gain_value,self.torque_gain_value,self.torque_gain_value,self.torque_gain_value]

        self.rpm_proportional_gain_value = rospy.get_param("rpm_P_gain", 5.) # P
        self.rpm_integral_gain_value = rospy.get_param("rpm_I_gain", 0. )   # I
        self.rpm_derivation_gain_value = rospy.get_param("rpm_D_gain", 0. )    # D

        self.rpm_integral = Float32MultiArray() # RPM integral error
        self.rpm_integral.data = [0,0,0,0]

        self.use_antiwindup = rospy.get_param("use_antiwindup", False)
        self.windup_max = rospy.get_param("windup_max", 1500.0)

        self.last_error_time = [rospy.Time.now(), rospy.Time.now(), rospy.Time.now(), rospy.Time.now()] # ROStime of the last error computed
        self.last_error = Float32MultiArray() # Value of the last error computed
        self.last_error.data = [0,0,0,0]

        self.rpm_derivative = Float32MultiArray()  # RPM derivative error
        self.rpm_derivative.data = [0,0,0,0]

        self.rpm_proportional = Float32MultiArray()  # RPM proportional error
        self.rpm_proportional.data = [0,0,0,0]

        # -- cmd_vel PID parameters using the PID library

        self.use_cmd_vel_pid = Bool(True)
        self.cmd_vel_correct = Twist()
        self.last_cmd_vel = Twist()
        self.last_cmd_vel_time = rospy.Time.now()
        self.last_odom_time = rospy.Time.now()

        self.vel_lin_pid = PID(Kp = 0.1, Ki = 0.05, Kd = 0.02, dt = 1/20)
        self.vel_lin_pid.use_lim = False
        self.vel_lin_pid.use_antiwindup = False
        self.vel_lin_pid.windupMax = 1.0

        self.vel_rot_pid = I_PID(Kp = 0.75, Ki = 0.15, Kd = 0.02, dt = 1/20, order=2, alpha=1, W=10)
        self.vel_rot_pid.use_lim = False
        self.vel_rot_pid.use_antiwindup = False
        self.vel_rot_pid.windupMax = 1.0

        # -- Acceleration theshold
        self.acc_lim = 500 # Acceleration limit in rpm / sec
        self.last_cmd = Float32MultiArray() # Last Cmd out published by the barakuda
        self.last_cmd.data = [0,0,0,0]

        self.last_cmd_rpm_time = rospy.Time.now() # ROStime of the last cmd_vel command (use for acceleration max mesurement and timeout computing)
        self.last_rpm_pub_time = rospy.Time.now() # ROStime of the last rpm command (use for acceleration max mesurement and timeout computing)
        self.current_rpm_time = rospy.Time.now() # ROStime of the current rpm command
        
        # -- Safety 
        self.motor_target_null = Bool(False) # Check if the barakuda motor target is null (no cmd | no listening | emergency stop on)
        self.manager_stop = Bool(False) # From the Barakuda Manager
        self.use_acc_limit = True 
        self.cmd_pub_timeout = rospy.Duration.from_sec(2/self.rate) # Define the timeout between two  publication of cmd_out

        # Susbcriber
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_cb)
        rospy.Subscriber("/cmd_rpm", Float32MultiArray, self.cmd_rpm_cb)
        rospy.Subscriber('/barakuda/motor_rpm', Float32MultiArray, self.rpm_cb)
        rospy.Subscriber("/barakuda/motor_torque", Float32MultiArray, self.torque_cb)
        rospy.Subscriber("/barakuda/motor_rpm_target", Float32MultiArray, self.motor_target_cb)

        rospy.Subscriber("/imu/odometry", Odometry, self.odom_cb)
        rospy.Subscriber('/imu/data',Imu, self.imu_cb)

        # -- Safety
        rospy.Subscriber("/safety/manager_stop", Bool, self.manager_stop_cb)


        # Publisher
        self.cmd_motors_pub = rospy.Publisher('/barakuda/cmd_motors', Float32MultiArray, queue_size=5)

        # -- Debug purpose
        self.model_rpm_pub = rospy.Publisher('/barakuda/model_rpm', Float32MultiArray, queue_size=5)
        self.torque_gain_pub = rospy.Publisher('/barakuda/torque_gain', Float32MultiArray, queue_size=5)
        self.rpm_integral_pub = rospy.Publisher('/barakuda/rpm_integral', Float32MultiArray, queue_size=5)
        self.rpm_derivative_pub = rospy.Publisher('/barakuda/rpm_derivative', Float32MultiArray, queue_size=5)
        self.cmd_vel_correct_pub = rospy.Publisher('/barakuda/cmd_vel_correct', Twist, queue_size=5)
        self.inv_model_pub = rospy.Publisher('/barakuda/inv_model_pub', Twist, queue_size=5)

        self.average_odom_lin_pub = rospy.Publisher('/barakuda/odom_lin', Float32, queue_size=5)
        self.average_odom_rot_pub = rospy.Publisher('/barakuda/odom_rot', Float32, queue_size=5)
        
    # -- Class Methods -- 

    def cmd_vel_cb(self, cmd_vel):
        # If the manager stop is ON, the command is set to 0
        if(not self.manager_stop.data):
            self.cmd_vel= cmd_vel
            
            if((self.last_cmd_vel.angular.z * cmd_vel.angular.z) < 0.000001): # If there is a change in rotation, reset the error
                self.vel_rot_pid.reset_error()
            
            if((self.last_cmd_vel.linear.x * cmd_vel.linear.x) < 0.000001): # ame for linear mouvement
                self.vel_lin_pid.reset_error()

            if((self.odom != Odometry()) & self.use_cmd_vel_pid.data): # Check is the Odometry is not null   TO DO: ADD Other safety such as timeout
                self.cmd_vel_correct.linear.x = self.vel_lin_pid.update(cmd_vel.linear.x, self.odom.twist.twist.linear.x)
                self.cmd_vel_correct.angular.z = self.vel_rot_pid.update(cmd_vel.angular.z, self.odom.twist.twist.angular.z,self.cmd_vel_correct.angular.z )
            else:
                self.cmd_vel_correct = cmd_vel # No correction if the Odometry is not received
                print("/!\ No cmd _vel PID /!\ ")

            self.cmd_vel_correct_pub.publish(self.cmd_vel_correct) # Publish cmd_vel corrected value
            self.cmd_rpm = self.model_twist_to_rpm(self.cmd_vel_correct) # Send cmd_vel correct to the model
        else:
            self.cmd_vel = Twist()
            self.cmd_vel_correct = self.cmd_vel
            self.cmd_rpm = Float32MultiArray()
            self.cmd_rpm.data = [0,0,0,0]
            print("Emergency Stop")

        self.last_cmd_vel= self.cmd_vel_correct
        self.last_cmd_vel_time = rospy.Time.now()
        self.last_cmd_rpm_time = rospy.Time.now()
        
    def cmd_rpm_cb(self, cmd_rpm):
        self.cmd_rpm = cmd_rpm
        self.last_cmd_rpm_time = rospy.Time.now()

    def motor_target_cb(self, motor_target):
        self.motor_target_null.data = False
        if((motor_target.data[0]==0) and (motor_target.data[1]==0) and (motor_target.data[2]==0) and (motor_target.data[3]==0) ):
            self.reset_rpm_integral()

    def imu_cb(self, imu_data):
        quat = imu_data.orientation
        self.heading =  np.arctan2(2*(quat.y*quat.x+quat.w*quat.z),1-2*(quat.x*quat.x + quat.z*quat.z))+0.5*np.pi

    def odom_cb(self, odom_data):
        self.odom = odom_data

        # Average odom linear data
        self.average_odom_lin.pop(0)
        self.average_odom_lin.append(odom_data.twist.twist.linear.x)
        self.odom_lin.data = sum(self.average_odom_lin)/len(self.average_odom_lin)

        # Average odom rotation data
        self.average_odom_rot.pop(0)
        self.average_odom_rot.append(odom_data.twist.twist.angular.z)
        self.odom_rot.data = sum(self.average_odom_rot)/len(self.average_odom_rot)

        self.average_odom_lin_pub.publish(self.odom_lin)
        self.average_odom_rot_pub.publish(self.odom_rot)

        self.last_odom_time = rospy.Time.now()

    def rpm_cb(self, rpm):

        # For each wheels
        for i in [0,1,2,3]:
            # Remove the first element
            self.average_rpm[i].pop(0)
            # Add the last knowned value
            self.average_rpm[i].append(rpm.data[i])

            # Average
            self.rpm_encoder.data[i]= sum(self.average_rpm[i])/len(self.average_rpm[i])
        
        # publish inverted rpm model
        self.inv_model_pub.publish(self.model_rpm_to_twist(self.rpm_encoder.data))


    def torque_cb(self, torque):
        
        # For each wheels
        for i in [0,1,2,3]:
            # Remove the first element
            self.average_torque[i].pop(0)
            # Add the last knowned value
            self.average_torque[i].append(torque.data[i])

            # Average
            self.torque.data[i]= sum(self.average_torque[i])/len(self.average_torque[i])

        
    def manager_stop_cb(self, stop):
        self.manager_stop = stop

        # Reset all gain and error if there is some manager_stop
        if(self.manager_stop.data):
            self.reset_rpm_integral()
            self.vel_lin_pid.reset_error()
            self.vel_rot_pid.reset_error()
            self.torque_gain.data = [self.torque_gain_value,self.torque_gain_value,self.torque_gain_value,self.torque_gain_value]
            self.cmd_rpm.data = [0,0,0,0] # Command is null

    # -- Class Functions -- 

    def do_threshold(self,limit,list_of_4_elem):
        return [math.copysign(min(limit,abs(list_of_4_elem[0])),list_of_4_elem[0]),math.copysign(min(limit,abs(list_of_4_elem[1])),list_of_4_elem[1]),math.copysign(min(limit,abs(list_of_4_elem[2])),list_of_4_elem[2]),math.copysign(min(limit,abs(list_of_4_elem[3])),list_of_4_elem[3]) ]

    # Exail model
    def model_trans(self, cmd_vel):

        cmd_right = (cmd_vel.linear.x + cmd_vel.angular.z)/self.wheel_radius*self.motor_red*self.rds2rpm
        cmd_left = (cmd_vel.linear.x  - cmd_vel.angular.z)/self.wheel_radius*self.motor_red*self.rds2rpm

        self.cmd_rpm.data = [cmd_right, cmd_left, cmd_right, cmd_left]

        # Publish rpm at the output of the model 
        
        self.model_rpm = self.cmd_rpm
        self.model_rpm_pub.publish(self.model_rpm)

    # Translate motor_rpm to cmd_vel
    def model_rpm_to_twist(self, motor_rpm):
        inv_cmd_vel = Twist()

        wheel_vel_linear = (motor_rpm[0] + motor_rpm[1]) /2
        wheel_vel_rot = (motor_rpm[0]- motor_rpm[1]) /2

        inv_cmd_vel.linear.x = wheel_vel_linear / 1033.472622967
        inv_cmd_vel.angular.z = wheel_vel_rot / (736.264618701+258.626782524)
        
        return inv_cmd_vel

    # Translate cmd_vel to motor_rpm
    def model_twist_to_rpm(self, cmd_vel):

        vel=0.475*cmd_vel.angular.z

        alpha=math.atan2(vel,self.dist_center_wheel)

        #velocity=math.copysign((self.cos_theta*vel)/math.tan(alpha+self.theta),cmd_vel.angular.z)
        #rospy.loginfo("Velocity %f",velocity)

        beta=abs(cmd_vel.angular.z)+self.theta
        velocity=math.copysign((self.dist_center_wheel*self.cos_theta/math.cos(beta))*math.sin(beta)-(self.dist_center_wheel*self.sin_theta),cmd_vel.angular.z)
        #rospy.loginfo("Velocity %f",velocity)

        wheel_vel_rot=vel*1033.472622967 
        wheel_vel_linear=cmd_vel.linear.x*1033.472622967

        vR=wheel_vel_linear + wheel_vel_rot
        vL=wheel_vel_linear - wheel_vel_rot

        maxi = self.gain_rpm *max(abs( vR),abs(vL))
        velo = [self.gain_rpm * (vR), self.gain_rpm * (vL), self.gain_rpm * (vR), self.gain_rpm * (vL)]

        if maxi>self.max_rpm:
            velo=[velo[0]*self.max_rpm/maxi,velo[1]*self.max_rpm/maxi,velo[2]*self.max_rpm/maxi,velo[3]*self.max_rpm/maxi]
            
        velo=[self.gain_rpm * (wheel_vel_linear + wheel_vel_rot), self.gain_rpm * (wheel_vel_linear - wheel_vel_rot), self.gain_rpm * (wheel_vel_linear + wheel_vel_rot), self.gain_rpm * (wheel_vel_linear - wheel_vel_rot)]
        
        self.model_rpm.data = self.do_threshold(self.max_rpm,velo)
        self.model_rpm_pub.publish(self.model_rpm)

        return self.model_rpm

    def reset_rpm_integral(self):
        self.rpm_integral.data = [0,0,0,0]

    def cmd_pub(self):

        self.current_rpm_time = rospy.Time.now()

        # If the command is Null, nothing is published
        if((self.cmd_rpm.data[0] == 0) & (self.cmd_rpm.data[1] == 0) & (self.cmd_rpm.data[2] == 0) & (self.cmd_rpm.data[3] == 0 )):
            self.cmd_out.data = [0,0,0,0]
            #print("Command is Null")

        # Otherwise we compute the motor speed depending on the torque
        else:
            for i in [0,1,2,3]:
                # -- Check for the max RPM limit
                error = (self.cmd_rpm.data[i] - self.rpm_encoder.data[i]) # Compute error
                
                # RPM Derivative error computation 
                self.rpm_derivative.data[i] = self.rpm_derivation_gain_value * (error - self.last_error.data[i])/(rospy.Time.now() - self.last_error_time[i]).to_sec() # Compute derivative term

                # Last error update fot the next loop
                self.last_error_time[i] = rospy.Time.now() # Last error compute send Time
                self.last_error.data[i] = error

                # RPM Integral error computation 
                self.rpm_integral.data[i] += error * self.rpm_integral_gain_value * (self.current_rpm_time - self.last_rpm_pub_time).to_sec()  # Compute Integral term
                
                # RPM Proportional error computation 
                self.rpm_proportional.data[i] =  self.rpm_proportional_gain_value *self.torque_gain.data[i]*self.torque.data[i]

                # Antiwindup
                if (self.use_antiwindup & (self.windup_max != 0)):
                    self.rpm_integral.data[i] = min(abs(self.rpm_integral.data[i]),self.windup_max)*np.sign(self.rpm_integral.data[i])
                
                # Add Actuel RPM + P + I + D
                self.cmd_rpm_thresh.data[i] = (min(abs(self.cmd_rpm.data[i] + self.rpm_proportional.data[i] + self.rpm_integral.data[i] + self.rpm_derivative.data[i]),self.max_rpm))*np.sign(self.cmd_rpm.data[i] + self.rpm_proportional.data[i] + self.rpm_integral.data[i] + self.rpm_derivative.data[i])

                # -- Compute Torque based gain for the next loop
                #if((self.torque.data[i] != 0) & (abs(self.torque.data[i]) > self.torque_dead_zone)):
                if(self.torque.data[i] != 0):
                    self.torque_gain.data[i] = min((error/self.torque.data[i]),self.max_torque_gain)

            # Integral and Derivative value publication (for debug)
            self.rpm_integral_pub.publish(self.rpm_integral)
            self.rpm_derivative_pub.publish(self.rpm_derivative)

        # Acceleration limit check
        if (self.use_acc_limit):
            # Compute Max acceleration for each cmd loop
            max_acc=self.acc_lim*(self.current_rpm_time - self.last_rpm_pub_time).to_sec() 

            # Compute the difference between the current cmd and the last cmd
            speed_diff = [self.cmd_rpm_thresh.data[0] - self.last_cmd.data[0], self.cmd_rpm_thresh.data[1] - self.last_cmd.data[1], self.cmd_rpm_thresh.data[2] - self.last_cmd.data[2], self.cmd_rpm_thresh.data[3] - self.last_cmd.data[3]]
            # Compute the threshold between therrore max acceleration and the speed difference
            add_speed = self.do_threshold(float(max_acc),speed_diff)

            # Add the maxed acceleration to the last command
            self.cmd_out.data = [self.last_cmd.data[0] + add_speed[0], self.last_cmd.data[1] + add_speed[1], self.last_cmd.data[2] + add_speed[2], self.last_cmd.data[3] + add_speed[3]]
            
        else:
            self.cmd_out = self.cmd_rpm_thresh

        # If the sign of a cmd_out is deffirent than the previous, the integral rpm is reset
        for i in [0,1,2,3]:
            if(np.sign(self.cmd_out.data[i]) != np.sign(self.last_cmd.data[i])):
                self.rpm_integral.data[i] = 0
        
        # Command publication timeout Check
        if((self.current_rpm_time - self.last_cmd_rpm_time) > self.cmd_pub_timeout):
            self.cmd_rpm.data = [0,0,0,0] # Command is null
            self.cmd_out.data = [0,0,0,0]
            self.reset_rpm_integral()
            self.vel_lin_pid.reset_error()
            self.vel_rot_pid.reset_error()

        # Cmd_vel publication
        self.cmd_motors_pub.publish(self.cmd_out)

        # Torque gain publication (for debug)
        self.torque_gain_pub.publish(self.torque_gain)

        # Last RPM update fot the next loop
        self.last_rpm_pub_time = rospy.Time.now() # Last RPM send Time
        self.last_cmd = self.cmd_out # Last RPM send Command

    def reconfigure_cb(self, config, level):

        self.rpm_proportional_gain_value = config.rpm_P_gain
        self.rpm_integral_gain_value = config.rpm_I_gain
        self.rpm_derivation_gain_value = config.rpm_D_gain

        self.vel_lin_pid.Kp = config.vel_lin_P_gain
        self.vel_lin_pid.Ki = config.vel_lin_I_gain
        self.vel_lin_pid.Kd = config.vel_lin_D_gain

        self.vel_rot_pid.Kp = config.vel_rot_P_gain
        self.vel_rot_pid.Ki = config.vel_rot_I_gain
        self.vel_rot_pid.Kd = config.vel_rot_D_gain
        self.vel_rot_pid.alpha = config.vel_rot_alpha
        self.vel_rot_pid.update_w_param(config.vel_rot_W)

        self.max_rpm = config.max_rpm
        self.acc_lim = config.max_acc
        
        return config

if __name__ == '__main__':
    try:
        # Initiate the node
        rospy.init_node('controller_node', anonymous=True)
        rospy.loginfo("Controller Node: Start")
        barakuda_controller = Controller(rospy.get_name())

        srv = Server(PIDConfig, barakuda_controller.reconfigure_cb)

        while not rospy.is_shutdown():
            barakuda_controller.cmd_pub()
            rate=rospy.Rate(barakuda_controller.rate) #20hz
            rate.sleep()
    except rospy.ROSInterruptException:
        rospy.loginfo("Controller Node: Stop")
