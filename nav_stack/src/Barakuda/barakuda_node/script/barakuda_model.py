#!/usr/bin/env python3

# ROS imports
import rospy
import math

#ROS msgs import
from std_msgs.msg import Int32, Float32MultiArray, Bool
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

#Import PID controller
from pid_controller import *

# Verbose
VERBOSE = False

# Model class
class BarakudaModel:

    def __init__(self,name):
        # ROSparams
        self.max_rpm = rospy.get_param("max_rpm", 5000)
        self.gain_rpm = rospy.get_param("gain_rpm", 1)
        self.lim_acc_rpm = rospy.get_param("acc_lim_rpm", 2000)

        #Default variables
        self.motor_rpm = Float32MultiArray()
        self.add_new_velo=0

        self.current_motor_rpm = [0.,0.,0.,0.]
        self.current_motor_torque = [0.,0.,0.,0.]

        self.min_rpm_linear_from_static = 500
        self.min_rpm_angular_from_static = 800
        self.is_moving = False

        # Cmd requested
        self.cmd_vel_request = Twist()

         # Cmd inverted from current motor rpm
        self.current_cmd_vel = Twist()

        # Cmd corrected after the PID
        self.cmd_vel_correct = Twist()
        
        self.last_rpm=[0,0,0,0]
        self.last_update= rospy.Time.now()
        self.rpm_request=[0,0,0,0]
        
        self.previous_cmd_vel = Twist()

        self.dist_center_wheel = 0.7117056976025975
        self.theta = 0.8400699948165985
        self.sin_theta = math.sin(self.theta)
        self.cos_theta =math.cos(self.theta)

        self.odom = Odometry()

        # Manager gestion Stop
        self.manager_stop = True

        # PID controller
        # X trans PID
        #
        # config sol sec
        # Kp = 3.8, Ki=0.4, Kd=1.4, dt=1/20 béton
        #                                  0.8     1.4
        #                      Kp = 5., Ki=-0.30, Kd=-0.075, dt=1/20 humide
        self.x_trans_pid = PID(Kp = 2., Ki=0., Kd=0.2, dt=1/20)#Kp = 5., Ki=-0.30, Kd=-0.075, dt=1/20)#Kp = 5.9, Ki=0.1, Kd=0.75, dt=1/20)#3 3.8
        self.x_trans_pid.use_lim = True
        self.x_trans_pid.lim_max=6.0
        self.x_trans_pid.lim_min=-6.0

        # Z rot PID
        #
        # config sol sec
        # Kp =4., Ki=0.5, Kd=0.5, dt=1/20 béton
        #                               0.10     0.5
        #                    Kp =6., Ki=1., Kd=4., dt=1/20
        #                    Kp =10., Ki=-0.20, Kd=0.02, dt=1/20 humide
        self.z_rot_pid = PID(Kp =10., Ki=-0.20, Kd=0.02, dt=1/20)#Kp =3., Ki=0., Kd=0.1, dt=1/20)#(Kp =10., Ki=-0.20, Kd=0.02, dt=1/20)#(Kp =1., Ki=1.75, Kd=0.5, dt=1/20)#0,Ki=0.0, Kd=0.0, dt=1/20) #1., Ki=1.75, Kd=0.5, dt=1/20)
        self.z_rot_pid.use_lim = False
        self.z_rot_pid.use_antiwindup = True
        self.z_rot_pid.lim_max=3.1415*20
        self.z_rot_pid.lim_min=-3.1415*20
        #ROS Subscriber init
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_cb)
        rospy.Subscriber("/barakuda/motor_rpm", Float32MultiArray, self.motor_rpm_cb) # Motor rpm
        rospy.Subscriber("/barakuda/motor_torque", Float32MultiArray, self.motor_torque_cb) # Motor torque
        rospy.Subscriber("/barakuda/imu/odometry", Odometry, self.odom_cb) # Motor torque
        rospy.Subscriber("/safety/manager_stop", Bool, self.manager_stop_cb)

        #ROS Publishers init
        self.motors_pub = rospy.Publisher("/barakuda/cmd_motors", Float32MultiArray, queue_size=5)
        self.cmd_vel_correct_pub = rospy.Publisher("/barakuda/cmd_vel_correct", Twist, queue_size=5)

    # -- Subscribers --
    # Twist to Motors RPM function
    def cmd_vel_cb(self, cmd_vel): 
       
        #self.motor_rpm.data = [min(self.max_rpm, self.gain_rpm * (cmd_vel.linear.x + cmd_vel.angular.z)), min(self.max_rpm, self.gain_rpm * (cmd_vel.linear.x - cmd_vel.angular.z)), min(self.max_rpm, self.gain_rpm * (cmd_vel.linear.x + cmd_vel.angular.z)), min(self.max_rpm, self.gain_rpm * (cmd_vel.linear.x - cmd_vel.angular.z))]
        #self.rpm_request = self.compute_vel_low_vel_ok(cmd_vel)
        #print ("rpm request: ",self.rpm_request)
        #self.motors_pub.publish(self.motor_rpm)

        
        if((self.previous_cmd_vel.angular.z * cmd_vel.angular.z) < 0):
            self.z_rot_pid.reset_error()

        # Set cmd_vel request to null if the emergency stop is on
        # Check SHARKSTATION stop, SHARKSTATION listening, close range scanner
        if(not self.manager_stop):
            #print("MOVE OK")
            self.cmd_vel_request=cmd_vel
            self.cmd_vel_request.linear.x=self.cmd_vel_request.linear.x#*0.684210526
            self.cmd_vel_request.angular.z=self.cmd_vel_request.angular.z#*0.65
            # if((self.previous_cmd_vel.angular.z * cmd_vel.angular.z) < 0):
            #     self.z_rot_pid.reset_error()
            #     self.cmd_vel_request=Twist()
            # if((self.previous_cmd_vel.linear.x * cmd_vel.linear.x) < 0):
            #     self.z_rot_pid.reset_error()
            #     self.cmd_vel_request=Twist()
        else:
            #print("MOVE STOP")
            self.cmd_vel_request= Twist()

        # self.cmd_vel_pid(self.cmd_vel_request)

        #print(self.cmd_vel_request)
        self.previous_cmd_vel = cmd_vel

    def motor_rpm_cb(self, rpm):
        self.current_motor_rpm = rpm.data
        self.is_moving=True
        if (abs(self.current_motor_rpm[0])<10 and abs(self.current_motor_rpm[1])<10):
            self.z_rot_pid.reset_error()
            self.x_trans_pid.reset_error()
            self.is_moving=False
            self.add_new_velo=0

    def motor_torque_cb(self, torque):
        self.current_motor_torque = torque.data

    def odom_cb(self, odom):
        self.odom = odom

    def manager_stop_cb(self, stop):
        self.manager_stop = stop.data

    # -- Publishers --

    # Publish robot RPM with a given max acceleration
    def publish_rpm(self):
        self.cmd_vel_pid(self.cmd_vel_request)

        now_=rospy.Time.now()
        acc_lim=self.lim_acc_rpm*(now_-self.last_update).to_sec()
        self.last_update=now_
        zero=True

        for i in self.rpm_request:
            if i <-2 or i>2:
                zero=False
                # break
        if zero:
            self.motor_rpm.data = [0,0,0,0]
            self.last_rpm=[0,0,0,0]
            self.motors_pub.publish(self.motor_rpm)
            self.add_new_velo=0
            return
        #print ("req vel: ",self.rpm_request)
        dif_velo=[self.rpm_request[0]-self.last_rpm[0],self.rpm_request[1]-self.last_rpm[1],self.rpm_request[2]-self.last_rpm[2],self.rpm_request[3]-self.last_rpm[3]]
        add_velo=self.do_threshold(float(acc_lim),dif_velo)#[math.copysign(min(acc_lim,abs(dif_velo[0])),dif_velo[0]),math.copysign(min(acc_lim,abs(dif_velo[1])),dif_velo[1]),math.copysign(min(acc_lim,abs(dif_velo[2])),dif_velo[2]),math.copysign(min(acc_lim,abs(dif_velo[3])),dif_velo[3]) ]
        # diff_to_add=0
        # if (last_rpm==[0,0,0,0]):
        #     maxirpm=max(last_rpm)
        #     diff_to_add=500-maxirpm
        #     if (self.cmd_vel.angular.z>0.001 or self.cmd_vel.angular.z<-0.001) diff_to_add=700-maxirpm
        
        self.last_rpm=[self.last_rpm[0]+add_velo[0],self.last_rpm[1]+add_velo[1],self.last_rpm[2]+add_velo[2],self.last_rpm[3]+add_velo[3]]
        # applyrpm=[self.last_rpm[0]+diff_to_add,self.last_rpm[1]+diff_to_add,self.last_rpm[2]+diff_to_add,self.last_rpm[3]+diff_to_add]
        #print ("do vel: ",self.last_rpm," with acc_lim= ",acc_lim)
        rospy.loginfo("rpm request : %f , %f , %f , %f",self.last_rpm[0],self.last_rpm[1],self.last_rpm[2],self.last_rpm[3])
        self.motor_rpm.data =self.last_rpm
        self.motors_pub.publish(self.motor_rpm)

        #reduction = 32

        # print("bug: ", self.motor_rpm[0])

        # PID controller
        #resultat_0 = self.pid.update(self.motor_rpm.data[0] / reduction, self.current_motor_rpm[0] / reduction)
        #resultat_1 = self.pid.update(self.motor_rpm.data[1] / reduction, self.current_motor_rpm[1] / reduction)
        #resultat_2 = self.pid.update(self.motor_rpm.data[2] / reduction, self.current_motor_rpm[2] / reduction)
        #resultat_3 = self.pid.update(self.motor_rpm.data[3] / reduction, self.current_motor_rpm[3] / reduction)

        # self.motor_rpm.data = [resultat_0* reduction, resultat_1* reduction, resultat_2* reduction, resultat_3* reduction] 
        # self.motor_rpm.data =self.do_threshold(500, self.motor_rpm.data)
        # self.motors_pub.publish(self.motor_rpm)

        #print("Motor 0 pid value: ", self.motor_rpm.data[0]," result: ",resultat_0* reduction)
        #print("Motor 1 pid value: ", self.motor_rpm.data[1]," result: ",resultat_1* reduction)
        #print("Motor 2 pid value: ", self.motor_rpm.data[2]," result: ",resultat_2* reduction)
        #print("Motor 3 pid value: ", self.motor_rpm.data[3]," result: ",resultat_3* reduction)

    # -- Functions --
    def do_threshold(self,limit,list_of_4_elem):
        return [math.copysign(min(limit,abs(list_of_4_elem[0])),list_of_4_elem[0]),math.copysign(min(limit,abs(list_of_4_elem[1])),list_of_4_elem[1]),math.copysign(min(limit,abs(list_of_4_elem[2])),list_of_4_elem[2]),math.copysign(min(limit,abs(list_of_4_elem[3])),list_of_4_elem[3]) ]

    # Translate cmd_vel to motor_rpm
    def compute_vel(self,cmd_vel):
        # wheel_vel_rot=(736.264618701+258.626782524)*cmd_vel.angular.z#+math.copysign(500,cmd_vel.angular.z) #60÷(2×π)×32.5*sqr(0.475*0.475+0.53*0.53)/0.3 (center distance/wheel radius)
        # wheel_vel_linear=cmd_vel.linear.x*1033.472622967#+math.copysign(500,cmd_vel.linear.x)  # 60÷(2×π)×32.5*1/0.3
        vel=0.475*cmd_vel.angular.z
        #vel=self.dist_center_wheel*abs(cmd_vel.angular.z)
        alpha=math.atan2(vel,self.dist_center_wheel)
        velocity=math.copysign((self.cos_theta*vel)/math.tan(alpha+self.theta),cmd_vel.angular.z)
        rospy.loginfo("velocity %f",velocity)
        beta=abs(cmd_vel.angular.z)+self.theta
        velocity=math.copysign((self.dist_center_wheel*self.cos_theta/math.cos(beta))*math.sin(beta)-(self.dist_center_wheel*self.sin_theta),cmd_vel.angular.z)
        rospy.loginfo("velocity true %f",velocity)
        wheel_vel_rot=vel*1033.472622967 
        wheel_vel_linear=cmd_vel.linear.x*1033.472622967
        vR=wheel_vel_linear + wheel_vel_rot
        vL=wheel_vel_linear - wheel_vel_rot
        maxi=self.gain_rpm *max(abs( vR),abs(vL))
        velo=[self.gain_rpm * (vR), self.gain_rpm * (vL), self.gain_rpm * (vR), self.gain_rpm * (vL)]
        if maxi>self.max_rpm:
            velo=[velo[0]*self.max_rpm/maxi,velo[1]*self.max_rpm/maxi,velo[2]*self.max_rpm/maxi,velo[3]*self.max_rpm/maxi]
        velo=[self.gain_rpm * (wheel_vel_linear + wheel_vel_rot), self.gain_rpm * (wheel_vel_linear - wheel_vel_rot), self.gain_rpm * (wheel_vel_linear + wheel_vel_rot), self.gain_rpm * (wheel_vel_linear - wheel_vel_rot)]
        return self.do_threshold(self.max_rpm,velo)#[math.copysign(min(self.max_rpm,abs(velo[0])),velo[0]),math.copysign(min(self.max_rpm,abs(velo[1])),velo[1]),math.copysign(min(self.max_rpm,abs(velo[2])),velo[2]),math.copysign(min(self.max_rpm,abs(velo[3])),velo[3]) ]
    
    # Translate motor_rpm to cmd_vel
    def compute_rpm(self,motor_rpm):
        inv_cmd_vel = Twist()

        wheel_vel_linear = (motor_rpm[0] + motor_rpm[1]) /2
        wheel_vel_rot = (motor_rpm[0]- motor_rpm[1]) /2

        inv_cmd_vel.linear.x = wheel_vel_linear / 1033.472622967
        inv_cmd_vel.angular.z = wheel_vel_rot / (736.264618701+258.626782524)
        
        return inv_cmd_vel

    def compute_vel_new(self,cmd_vel):
        vel=self.dist_center_wheel*abs(cmd_vel.angular.z)
        alpha=math.atan2(vel,self.dist_center_wheel)
        velocity=math.copysign((self.cos_theta*vel)/math.tan(alpha+self.theta),cmd_vel.angular.z)
        diff_to_addangle=0
        if (cmd_vel.angular.z>0.001 or cmd_vel.angular.z<-0.001): diff_to_addangle=math.copysign(min(abs(self.rpm_request[0]-self.last_rpm[0])/(self.rpm_request[0]+0.0001),1)*700,cmd_vel.angular.z)
        diff_to_add=0
        if (cmd_vel.linear.x>0.001 or cmd_vel.linear.x<-0.001): diff_to_add=math.copysign(min(abs(self.rpm_request[0]-self.last_rpm[0])/(self.rpm_request[0]+0.0001),1)*500,cmd_vel.linear.x)
        wheel_vel_rot=velocity*1033.472622967+diff_to_addangle
        wheel_vel_linear=cmd_vel.linear.x*1033.472622967+diff_to_add
        velo=[self.gain_rpm * (wheel_vel_linear + wheel_vel_rot), self.gain_rpm * (wheel_vel_linear - wheel_vel_rot), self.gain_rpm * (wheel_vel_linear + wheel_vel_rot), self.gain_rpm * (wheel_vel_linear - wheel_vel_rot)]
        return self.do_threshold(self.max_rpm,velo)

        # dist_1=math.sin()
        # vel=0.7117056976025975*cmd_vel.angular.z
        # wheel_vel_rot=(736.264618701+258.626782524)*cmd_vel.angular.z#+math.copysign(500,cmd_vel.angular.z) #60÷(2×π)×32.5*sqr(0.475*0.475+0.53*0.53)/0.3 (center distance/wheel radius)
        # wheel_vel_linear=cmd_vel.linear.x*1033.472622967#+math.copysign(500,cmd_vel.linear.x)  # 60÷(2×π)×32.5*1/0.3
        # velo=[self.gain_rpm * (wheel_vel_linear + wheel_vel_rot), self.gain_rpm * (wheel_vel_linear - wheel_vel_rot), self.gain_rpm * (wheel_vel_linear + wheel_vel_rot), self.gain_rpm * (wheel_vel_linear - wheel_vel_rot)]
        # return self.do_threshold(self.max_rpm,velo)#[math.copysign(min(self.max_rpm,abs(velo[0])),velo[0]),math.copysign(min(self.max_rpm,abs(velo[1])),velo[1]),math.copysign(min(self.max_rpm,abs(velo[2])),velo[2]),math.copysign(min(self.max_rpm,abs(velo[3])),velo[3]) ]

    def compute_vel_low_vel_ok(self,cmd_vel):
        vel=self.dist_center_wheel*abs(cmd_vel.angular.z)
        alpha=math.atan2(vel,self.dist_center_wheel)
        velocity=math.copysign((self.cos_theta*vel)/math.tan(alpha+self.theta),cmd_vel.angular.z)
        beta=abs(cmd_vel.angular.z)+self.theta
        velocity=math.copysign((self.dist_center_wheel*self.cos_theta/math.cos(beta))*math.sin(beta)-(self.dist_center_wheel*self.sin_theta),cmd_vel.angular.z)
        wheel_vel_rot=velocity*1033.472622967 
        wheel_vel_linear=cmd_vel.linear.x*1033.472622967
        if (math.copysign(1,wheel_vel_linear-wheel_vel_rot)!=math.copysign(1,wheel_vel_linear+wheel_vel_rot)):
            wheel_vel_rot=wheel_vel_rot+self.min_rpm_angular_from_static
        # if (abs(self.current_motor_rpm[0])<10 and abs(self.current_motor_rpm[0])<10):
        #     if (abs(wheel_vel_linear)>10)

        # if (wheel_vel_linear<
        # if (abs(self.current_motor_rpm[0])>10 or abs(self.current_motor_rpm[0])>10)
        # diff_to_addangle=0
        # if (cmd_vel.angular.z>0.001 or cmd_vel.angular.z<-0.001): diff_to_addangle=math.copysign(min(abs(self.rpm_request[0]-self.last_rpm[0])/(self.rpm_request[0]+0.0001),1)*700,cmd_vel.angular.z)
        # diff_to_add=0
        # if (cmd_vel.linear.x>0.001 or cmd_vel.linear.x<-0.001): diff_to_add=math.copysign(min(abs(self.rpm_request[0]-self.last_rpm[0])/(self.rpm_request[0]+0.0001),1)*500,cmd_vel.linear.x)
        # wheel_vel_rot=velocity*1033.472622967+diff_to_addangle
        # wheel_vel_linear=cmd_vel.linear.x*1033.472622967+diff_to_add
        
        # velo=[self.gain_rpm * (wheel_vel_linear + wheel_vel_rot*min(1,math.copysign(1.2,wheel_vel_rot))), self.gain_rpm * (wheel_vel_linear - wheel_vel_rot*min(1,math.copysign(1.2,-1*wheel_vel_rot))), self.gain_rpm * (wheel_vel_linear + wheel_vel_rot*max(1,math.copysign(1.2,wheel_vel_rot))), self.gain_rpm * (wheel_vel_linear - wheel_vel_rot*max(1,math.copysign(1.2,-1*wheel_vel_rot)))]
        velo=[self.gain_rpm * (wheel_vel_linear + wheel_vel_rot), self.gain_rpm * (wheel_vel_linear - wheel_vel_rot), self.gain_rpm * (wheel_vel_linear + wheel_vel_rot), self.gain_rpm * (wheel_vel_linear - wheel_vel_rot)]
        linear_check=self.do_threshold(self.min_rpm_linear_from_static,velo)
        angular_check=self.do_threshold(self.min_rpm_angular_from_static,velo)
        #print("angular: ",angular_check," linear: ",linear_check)
        if (not self.is_moving and abs(angular_check[0])<self.min_rpm_angular_from_static and abs(wheel_vel_rot)>2):
            #velo=[self.gain_rpm * (wheel_vel_linear+math.copysign(self.min_rpm_angular_from_static,wheel_vel_rot)),self.gain_rpm * (wheel_vel_linear-math.copysign(self.min_rpm_angular_from_static,wheel_vel_rot)),self.gain_rpm * (wheel_vel_linear+math.copysign(self.min_rpm_angular_from_static,wheel_vel_rot)),self.gain_rpm * (wheel_vel_linear-math.copysign(self.min_rpm_angular_from_static,wheel_vel_rot))]
            #velo=[self.gain_rpm * (wheel_vel_linear+math.copysign(self.min_rpm_angular_from_static,wheel_vel_rot))*min(1,math.copysign(1.2,wheel_vel_rot)),self.gain_rpm * (wheel_vel_linear-math.copysign(self.min_rpm_angular_from_static,wheel_vel_rot)*min(1,math.copysign(1.2,-1*wheel_vel_rot))),self.gain_rpm * (wheel_vel_linear+math.copysign(self.min_rpm_angular_from_static,wheel_vel_rot)*max(1,math.copysign(1.2,wheel_vel_rot))),self.gain_rpm * (wheel_vel_linear-math.copysign(self.min_rpm_angular_from_static,wheel_vel_rot)*max(1,math.copysign(1.2,-1*wheel_vel_rot)))]
            #velo=[self.gain_rpm * (wheel_vel_linear+math.copysign(self.min_rpm_angular_from_static,wheel_vel_rot)),self.gain_rpm * (wheel_vel_linear-math.copysign(self.min_rpm_angular_from_static+self.add_new_velo,wheel_vel_rot)),self.gain_rpm * (wheel_vel_linear+math.copysign(self.min_rpm_angular_from_static+self.add_new_velo,wheel_vel_rot)),self.gain_rpm * (wheel_vel_linear-math.copysign(self.min_rpm_angular_from_static,wheel_vel_rot))]
            self.add_new_velo=self.add_new_velo+50
        elif (not self.is_moving and abs(linear_check[0])<self.min_rpm_linear_from_static and abs(wheel_vel_linear)>10 and abs(wheel_vel_rot)<2):
            velo=[self.gain_rpm *math.copysign(self.min_rpm_linear_from_static,wheel_vel_linear),self.gain_rpm * math.copysign(self.min_rpm_linear_from_static,wheel_vel_linear),self.gain_rpm * math.copysign(self.min_rpm_linear_from_static,wheel_vel_linear),self.gain_rpm * math.copysign(self.min_rpm_linear_from_static,wheel_vel_linear)]
        # if (not self.is_moving and abs(velo[0]/self.min_rpm_linear_from_static)<1 and abs(wheel_vel_rot)<10 and >10):
        #     velo=[math.copysign(self.min_rpm_linear_from_static,velo[0]),math.copysign(self.min_rpm_linear_from_static,velo[1]),math.copysign(self.min_rpm_linear_from_static,velo[2]),math.copysign(self.min_rpm_linear_from_static,velo[3])]
        # elif (not self.is_moving and abs((velo[0]-velo[1])/self.min_rpm_angular_from_static)<1 and abs(velo[0]-velo[1])>0.1):
        
        if (math.copysign(1,wheel_vel_linear-wheel_vel_rot)!=math.copysign(1,wheel_vel_linear+wheel_vel_rot)):
            add=math.copysign(self.gain_rpm*self.min_rpm_angular_from_static,wheel_vel_rot)
            factor=1
            if wheel_vel_rot>0:
                if self.is_moving:
                    factor=abs(self.current_motor_rpm[1]/self.current_motor_rpm[3])
                #velo=[self.current_motor_rpm[2],velo[1]-add,velo[2]+add,self.current_motor_rpm[1]]
                #velo=[(velo[2]+add)*factor*1.1,velo[1]-add,velo[2]+add,(velo[1]-add)*factor*1.1]
                velo=[velo[2]+add,velo[1]-add,velo[2]+add,velo[1]-add]
            else:
                if self.is_moving:
                    factor=abs(self.current_motor_rpm[0]/self.current_motor_rpm[2])
                # velo=[velo[0]+add,self.current_motor_rpm[3],self.current_motor_rpm[0],velo[3]-add]
                #velo=[velo[0]+add,(velo[3]-add)*factor*1.1,(velo[0]+add)*factor*1.1,velo[3]-add]
                velo=[velo[0]+add,velo[3]-add,velo[0]+add,velo[3]-add]
            #velo=[velo[0]+add
            #wheel_vel_rot=wheel_vel_rot+self.min_rpm_angular_from_static#     velo=[math.copysign(self.min_rpm_linear_from_static,wheel_vel_rot),math.copysign(self.min_rpm_linear_from_static,-1* wheel_vel_rot),math.copysign(self.min_rpm_linear_from_static, wheel_vel_rot),math.copysign(self.min_rpm_angular_from_static,-1* wheel_vel_rot)]
        return self.do_threshold(self.max_rpm,velo)

    # cmd_vel PID
    def cmd_vel_pid(self, cmd_vel_request):

        # Recive zero is zero!
        if(VERBOSE):
            rospy.loginfo("input linear: %f |input angular: %f",cmd_vel_request.linear.x,cmd_vel_request.angular.z)
        if (abs(cmd_vel_request.linear.x)<0.05 and abs(cmd_vel_request.angular.z)<0.001):
            self.rpm_request = [0,0,0,0]
            return 
        if (abs(cmd_vel_request.linear.x)>0.05 and abs(cmd_vel_request.angular.z)<0.001):
            self.z_rot_pid.reset_error()
        if (abs(cmd_vel_request.linear.x)<0.05 and abs(cmd_vel_request.angular.z)>0.001):
            self.x_trans_pid.reset_error()


        # Translate current rpm to current cmd_vel
        self.current_cmd_vel = self.compute_rpm(self.current_motor_rpm)

        # Average between odom and current vel
        x_trans_current = 0
        z_rot_current  = 0

        if(self.is_moving):
            gain_odom_vel_lin = 0.5
            gain_odom_vel_ang = 0.25
            x_trans_current = (self.current_cmd_vel.linear.x * gain_odom_vel_lin + self.odom.twist.twist.linear.x * (1-gain_odom_vel_lin))
            z_rot_current  = (self.current_cmd_vel.angular.z * gain_odom_vel_ang + self.odom.twist.twist.angular.z * (1-gain_odom_vel_ang))
        
        if(VERBOSE):
            rospy.loginfo("Estimated linear: %f | Estimated angular: %f", x_trans_current, z_rot_current)

        # Update PID and send the corrected value
        self.cmd_vel_correct.linear.x = self.x_trans_pid.update(cmd_vel_request.linear.x, x_trans_current)
        self.cmd_vel_correct.angular.z = self.z_rot_pid.update(cmd_vel_request.angular.z, z_rot_current)

        # Convert cmd_vel in RPM
        self.rpm_request = self.compute_vel(self.cmd_vel_correct)

        # Publishing corrected cmd_vel
        self.cmd_vel_correct_pub.publish(self.cmd_vel_correct)

        # RPM send to the robot using the pub in the main 

if __name__ == "__main__":
    try:
        # Initiate the node
        rospy.init_node('barakuda_model', anonymous=True)

        barakuda_model = BarakudaModel(rospy.get_name())

        rospy.loginfo("Barakuda Model: Start")
        rate = rospy.Rate(20) # 20hz
        while not rospy.is_shutdown():
            barakuda_model.publish_rpm()
            rate.sleep()
        
    except rospy.ROSInterruptException:

        rospy.loginfo("Barakuda Model: Stop")
