#!/usr/bin/env python3

# ROS imports
import rospy

#ROS msgs import
from std_msgs.msg import Int32MultiArray, Bool, Int32, Float32MultiArray
from geometry_msgs.msg import Twist,PoseStamped
from sensor_msgs.msg import Joy
import math
from axis_msgs.msg import Ptz
from rosnode import rosnode_ping

#ROS srvs import
from std_srvs.srv import SetBool

# Joystick command class
class BarakudaControl:

    def __init__(self,name):

        #Default variables
        self.pose= PoseStamped()
        self.last_pose= PoseStamped()
        self.last_pub=rospy.Time()
        self.vel = Twist()
        self.ptz = Ptz()
        self.dms = False # dead man switch
        self.sensivity = 1 #JoyStick gain, up to 5
        self.led = Int32MultiArray()
        self.led.data = [0,0,0,0] #Front, Side, Back, All
        self.led_front = 0
        self.side = 0
        self.led_back = 0
        self.led_all = 0
        self.stop = Bool(False)
        self.ptz_gain_array = [0.5, 1, 1.5, 2.0, 2.6]
        self.ptz_gain = 0
        self.speed_gain_array = [1.4, 2.8, 5.5] #1.4m/s, 2.8m/s, 5.5m/s
        self.speed_gain_array_rot = [0.7, 1.4, 2.5]#[0.1, 0.5, 1, 1.5, 2, 2.5, 3, 3.5, 4, 4.5, 5] #1.4m/s, 2.8m/s, 5m/s
        self.speed_gain = 0 #1.4m/s on start
        self.radius_limit= 5
        self.nav = False
        self.proxy_pause = Bool(False)
        self.switch_on_off_pub=True

        #ROS Publishers init
        self.nav_pub = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=5)
        self.vel_pub = rospy.Publisher("joy_vel", Twist, queue_size=5)
        self.ptz_vel_pub = rospy.Publisher("axis/cmd/velocity", Ptz, queue_size=5)
        self.ptz_pos_pub = rospy.Publisher("axis/cmd/position", Ptz, queue_size=5)

        #self.motor_pub = rospy.Publisher("/barakuda/cmd_motors", Float32MultiArray, queue_size=5)
        self.null_pub = rospy.Publisher("/null_vel", Twist, queue_size=5)
        self.led_pub = rospy.Publisher("/barakuda/cmd_led", Int32MultiArray, queue_size=5)
        self.safety_pub = rospy.Publisher("/safety/joy_stop", Bool, queue_size=5)
        self.proxy_pause_pub = rospy.Publisher("/safety/proxy_pause", Bool, queue_size=5)
        self.frame_for_nav_goal=rospy.get_param("nav_frame", "base_link")
        #self.stop_pub = rospy.Publisher("barakuda/cmd_stop", Bool, queue_size=5)

        self.autofocus_srv = rospy.ServiceProxy("/axis/set_autofocus", SetBool)

        #ROS Subscriber init
        rospy.Subscriber("/joy", Joy, self.joy_cb)

        # rospy.wait_for_service('stop')

    # Joystick callback function
    def joy_cb(self, joy):
        # Update Twist values if one of the DMS (Dead Man Switch) is triggered
        
        if joy.buttons[4] or joy.buttons[5]:
            self.dms = True # Dead Man Switch on the controller triggers
            self.nav=False
            # Speed commands
            self.vel.linear.x =  joy.axes[1] * self.speed_gain_array[self.speed_gain]
            self.vel.angular.z = joy.axes[0] * self.speed_gain_array_rot[self.speed_gain]

            self.ptz.pan =  -joy.axes[3] * self.ptz_gain_array[self.ptz_gain]
            self.ptz.tilt = joy.axes[4] * self.ptz_gain_array[self.ptz_gain]

            if joy.axes[5] != 1:
                self.ptz.zoom  = -(joy.axes[5]-1) * 250
            if joy.axes[2] != 1:
                self.ptz.zoom  = (joy.axes[2]-1) * 250




        # elif joy.buttons[5]:
            
        #     self.nav = True
        #     self.pose.header.frame_id=self.frame_for_nav_goal
        #     self.pose.header.stamp=rospy.Time.now()
        #     self.pose.pose.position.x=joy.axes[1] * self.radius_limit
        #     self.pose.pose.position.y=joy.axes[0] * self.radius_limit
        #     #self.pose.pose.orientation.w=1
        #     yaw = math.atan2((self.pose.pose.position.x),(self.pose.pose.position.y))
        #     cr = math.cos(0 * 0.5)
        #     sr = 0; #sin(0 * 0.5)
        #     cp = math.cos(0 * 0.5)
        #     sp = 0; #sin(0 * 0.5)
        #     cy = math.cos(yaw * 0.5)
        #     sy = math.sin(yaw * 0.5)
        #     self.pose.pose.orientation.w = cr * cp * cy + sr * sp * sy
        #     self.pose.pose.orientation.x = sr * cp * cy - cr * sp * sy
        #     self.pose.pose.orientation.y = cr * sp * cy + sr * cp * sy
        #     self.pose.pose.orientation.z = cr * cp * sy - sr * sp * cy
        #     if (math.sqrt(pow(self.last_pose.x-self.pose.x,2)+pow(self.last_pose.y-self.pose.y,2))<self.radius_limit*0.1):
        #         self.pose=self.last_pose


            #self.last_pose=self.pose


        else:
            self.nav=False
            self.dms = False
            #self.vel_pub.publish(Twist())

        # LED control on the Left arrows buttons
        if joy.buttons[3] == 1: # Y the front
            self.led.data[0] = int(not self.led.data[0])

        if joy.buttons[2] == 1 : # X the sides
            self.led.data[1] = int(not self.led.data[1])

        if joy.buttons[0] == 1: # A for the back
            self.led.data[2] = int(not self.led.data[2])

        if joy.buttons[1] == 1: # Right arrow for reset
                    self.ptz_pos_pub.publish(Ptz())


        if ((joy.buttons[7] and not joy.buttons[6]) and self.stop.data == True): # Start button set the AltStop to False using a service
            self.stop.data = False
             # Safety pub
            self.safety_pub.publish(Bool(False))
            rospy.wait_for_service('/barakuda/stop')
            try:
                set_stop = rospy.ServiceProxy('/barakuda/stop', SetBool)
                resp = set_stop(False)
                rospy.loginfo("Barakuda status: Disarmed")
                return resp.message
            except rospy.ServiceException as e:
                    print("Service call failed: %s"%e)
           

        if ((not joy.buttons[7] and joy.buttons[6]) and self.stop.data == False): # Back button set the AltStop to True using a service
            self.stop.data = True
            # Safety pub
            self.safety_pub.publish(Bool(True))
            rospy.wait_for_service('/barakuda/stop')
            try:
                set_stop = rospy.ServiceProxy('/barakuda/stop', SetBool)
                resp = set_stop(True)
                rospy.loginfo("Barakuda status: Armed")
                return resp.message
            except rospy.ServiceException as e:
                    print("Service call failed: %s"%e)

        # if (joy.buttons[3]): # Pause Proxy Lidar

        #     if self.proxy_pause.data == True:
        #         self.proxy_pause.data = False
        #     else:
        #         self.proxy_pause.data = True

        #     self.proxy_pause_pub.publish(self.proxy_pause)

        # Cmd vel gain       
        if (((joy.axes[7] == 1) and not (joy.axes[7] == -1)) and self.speed_gain < (len(self.speed_gain_array)-1)): # Top arrow increases the speed gain
            self.speed_gain += 1
            rospy.loginfo("Barakuda speed: {:.1f} m/s".format(self.speed_gain_array[self.speed_gain]))

        if  (((joy.axes[7] == -1) and not (joy.axes[7] == 1))  and self.speed_gain > 0): # Bot arrow decreases the speed gain
            self.speed_gain -= 1
            rospy.loginfo("Barakuda speed: {:.1f} m/s".format(self.speed_gain_array[self.speed_gain]))

        # Cmd PTZ gain
        if (((joy.axes[6] == -1) and not (joy.axes[6] == 1)) and self.ptz_gain < (len(self.ptz_gain_array)-1)): # Right arrow increases the speed gain
            self.ptz_gain += 1
            rospy.loginfo("Barakuda PTZ speed: {:.1f} rad/s".format(self.ptz_gain_array[self.ptz_gain]))

        if  (((joy.axes[6] == 1) and not (joy.axes[6] == -1))  and self.ptz_gain > 0): # Left Arrow decreases the speed gain
            self.ptz_gain -= 1
            rospy.loginfo("Barakuda PTZ speed: {:.1f} rad/s".format(self.ptz_gain_array[self.ptz_gain]))

        if joy.buttons[10] == 1:
            #rospy.wait_for_service("/axis/autofocus")
            #resp=self.autofocus_srv(False)
            rospy.logwarn("responce1: ",resp)
            #resp=self.autofocus_srv(True)
            rospy.logwarn("responce2: ",resp)

        if ( not self.nav):
            self.last_pose.header.stamp=rospy.Time.now()
            self.last_pose.pose.position.x=joy.axes[1] * self.radius_limit
            self.last_pose.pose.position.y=joy.axes[0] * self.radius_limit
            yaw = math.atan2((self.last_pose.pose.position.x),(self.last_pose.pose.position.y))
            cr = math.cos(0 * 0.5)
            sr = 0; #sin(0 * 0.5)
            cp = math.cos(0 * 0.5)
            sp = 0; #sin(0 * 0.5)
            cy = math.cos(yaw * 0.5)
            sy = math.sin(yaw * 0.5)
            self.last_pose.pose.orientation.w = cr * cp * cy + sr * sp * sy
            self.last_pose.pose.orientation.x = sr * cp * cy - cr * sp * sy
            self.last_pose.pose.orientation.y = cr * sp * cy + sr * cp * sy
            self.last_pose.pose.orientation.z = cr * cp * sy - sr * sp * cy



    def joy_vel_pub(self):

        if (not self.check_connection()):
            self.ptz_vel_pub.publish(Ptz())
            self.vel_pub.publish(Twist())
            return
        # Only publish if the dead man switch is on
        if self.dms:
            if (self.vel.linear.x!=0.0 or self.vel.angular.z!=0.0):
                self.switch_on_off_pub=True
                self.vel_pub.publish(self.vel)
            elif (self.switch_on_off_pub):
                self.switch_on_off_pub=False
                self.vel_pub.publish(Twist())
            print("velo to send: ",self.vel.linear.x," ",self.vel.angular.z )
            self.ptz_vel_pub.publish(self.ptz)
        elif self.nav:
            if (self.pose!=self.last_pose or rospy.Time.now()-self.last_pub>rospy.Duration(1)):
                self.nav_pub.publish(self.pose)
                self.last_pub=rospy.Time.now()
                self.last_pose=self.pose
        else:
            if self.vel != Twist():
                self.vel = Twist()
                self.vel_pub.publish(Twist())
            if self.ptz != Ptz() :
                self.ptz = Ptz()
                self.ptz_vel_pub.publish(Ptz())

        # Continuously publishing
        self.led_pub.publish(self.led)
        #self.stop_pub.publish(self.stop)
    def null_vel_pub(self):
        # Publish empty twist by default
        self.null_pub.publish(Twist())

    def check_connection(self):
        return True #rosnode_ping("/joy_node",0)

if __name__ == "__main__":

    try:
        # Initiate the node
        rospy.init_node('barakuda_joy', anonymous=True)

        barakuda_control = BarakudaControl(rospy.get_name())

        rospy.loginfo("Barakuda Control: Start")

        while not rospy.is_shutdown():
            barakuda_control.joy_vel_pub()

            #barakuda_control.null_vel_pub()
            rate = rospy.Rate(20) # 20hz
            rate.sleep()

    except rospy.ROSInterruptException:

        rospy.loginfo("Barakuda Control: Stop")
