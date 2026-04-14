#!/usr/bin/env python3
import rospy
from sbg_driver.msg import SbgEkfNav,SbgGpsPos

# global num_max
# global sum_gps
# global sum_ekf
# global count_gps
# global count_ekf
num_max=50
sum_gps=SbgGpsPos()
sum_ekf=SbgGpsPos()
count_gps=0
count_ekf=0
class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

def callback_gps(data):
    global num_max
    global sum_gps
    global sum_ekf
    global count_gps
    global count_ekf
    if (count_gps<num_max):
        sum_gps.latitude=sum_gps.latitude+data.latitude/num_max
        sum_gps.longitude=sum_gps.longitude+data.longitude/num_max
        sum_gps.altitude=sum_gps.altitude+data.altitude/num_max
        count_gps=count_gps+1
        print(f"{bcolors.WARNING}GPS ",count_gps,f"{bcolors.ENDC}")
    else:
        diff_lat=sum_gps.latitude-data.latitude
        diff_lon=sum_gps.longitude-data.longitude
        diff_alt=sum_gps.altitude-data.altitude
        print(f"{bcolors.WARNING}GPS moy: ",sum_gps.latitude," ",sum_gps.longitude," ",sum_gps.altitude," diff: ",diff_lat," ",diff_lon," ",diff_alt,f"{bcolors.ENDC}")

def callback_ekf(data):
    global num_max
    global sum_gps
    global sum_ekf
    global count_gps
    global count_ekf
    if (count_ekf<num_max):
        sum_ekf.latitude=sum_ekf.latitude+data.latitude/num_max
        sum_ekf.longitude=sum_ekf.longitude+data.longitude/num_max
        sum_ekf.altitude=sum_ekf.altitude+data.altitude/num_max
        count_ekf=count_ekf+1
        print("EKF ",count_ekf)
    else:
        diff_lat=sum_ekf.latitude-data.latitude
        diff_lon=sum_ekf.longitude-data.longitude
        diff_alt=sum_ekf.altitude-data.altitude
        print("EKF moy: ",sum_ekf.latitude," ",sum_ekf.longitude," ",sum_ekf.altitude," diff: ",diff_lat," ",diff_lon," ",diff_alt)
# def callback_(data):
#     rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
# def callback(data):
#     rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    
def listener():
    num_max=50
    sum_gps=SbgGpsPos()
    sum_ekf=SbgGpsPos()
    count_gps=0
    count_ekf=0
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/sbg/ekf_nav", SbgEkfNav, callback_ekf)
    rospy.Subscriber("/sbg/gps_pos", SbgGpsPos, callback_gps)
    # rospy.Subscriber("chatter", String, callback)
    # rospy.Subscriber("chatter", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
