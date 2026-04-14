#!/usr/bin/env python3

import rospy
import sys
import tf

#ROS msgs import
from geometry_msgs.msg import Twist, Point32
from std_msgs.msg import Time, Duration
from barakuda_manager.msg import *
from std_msgs.msg import Int32MultiArray

VERBOSE = False

# Create a Brakuda class
class BarakudaMissionLoader():

    def __init__(self,name):

        # Load params
        self.ZI_request = rospy.get_param('~ZEI_array', [])
        self.ZEI_contatenate = Polygon2Array()

        self.listener = tf.TransformListener()
        # Publisher def
        self.ZEI_publisher = rospy.Publisher('/manager/exclusion_poly', Polygon2Array, queue_size=10)

        #Subscriber def
        rospy.Subscriber('/mirador/ZEI_array', Int32MultiArray, self.ZEI_callback)

        # Tf static init
        rospy.loginfo("Wait for /tf between UTM and map...")
        self.listener.waitForTransform('UTM', 'map', rospy.Time(), rospy.Duration(200.0))
        rospy.loginfo("... /tf OK")

        # Obtenir la transformation entre le frame "UTM" et "map"
        (self.trans, self.rot) = self.listener.lookupTransform('UTM', 'map', rospy.Time(0))
        rospy.loginfo("Translation: %s, Rotation: %s", self.trans, self.rot)
    

        #Zones interdites
        self.ZI_points = [[0,0], #Null to match
        [458785.062,5212152.811],
        [458791.102,5212145.247],
        [458713.096,5212056.216],
        [458706.473,5212064.249],
        [458619.008,5212010.375],
        [458570.631,5212048.423],
        [458464.115,5212096.717],
        [458383.977,5212120.157],
        [458356.377,5212064.1],
        [458566.452,5211969.461],
        [458605.181,5212008.628],
        [458566.817,5212033.108],
        [458557.186,5212045.632],
        [458529.143,5212060.906],
        [458472.213,5212084.59],
        [458441.973,5212088.494],
        [458389.14,5212106.741],
        [458374.145,5212077.667],
        [458389.754,5212071.852],
        [458385.853,5212060.991],
        [458462.42,5212027.017],
        [458469.257,5212030.735],
        [458474.617,5212030.478],
        [458513.452,5212013.091],
        [458521.037,5212007.178],
        [458520.795,5211999.755],
        [458568.163,5211979.186],
        [458742.463,5212213.409],
        [458744.717,5212220.39],
        [458712.702,5212228.797],
        [458710.046,5212221.466]]

        self.ZE_points = [[0,0], #Null to match
        [458389.938,5212082.775],
        [458393.870,5212091.929],
        [458402.984,5212087.805],
        [458399.043,5212078.627],
        [458427.408,5212093.392],
        [458429.221,5212092.584],
        [458411.150,5212049.828],
        [458409.317,5212050.628],
        [458421.251,5212045.519],
        [458424.940,5212043.916],
        [458439.388,5212077.675],
        [458435.715,5212079.126],
        [458454.980,5212087.994],
        [458458.829,5212086.944],
        [458442.754,5212047.881],
        [458439.223,5212049.624],
        [458448.555,5212033.033],
        [458452.257,5212031.406],
        [458469.939,5212073.782],
        [458466.163,5212074.899],
        [458484.006,5212080.146],
        [458487.665,5212078.550],
        [458470.004,5212035.497],
        [458466.402,5212036.989],
        [458481.067,5212027.637],
        [458484.330,5212026.159],
        [458499.487,5212061.921],
        [458495.982,5212063.485],
        [458515.482,5212066.212],
        [458519.347,5212064.727],
        [458504.342,5212029.544],
        [458500.540,5212031.208],
        [458515.372,5212055.527],
        [458510.888,5212044.879],
        [458521.379,5212041.154],
        [458526.517,5212051.844],
        [458508.569,5212039.927],
        [458503.677,5212028.051],
        [458513.207,5212024.634],
        [458517.820,5212036.648],
        [458538.945,5212044.960],
        [458534.944,5212035.664],
        [458542.788,5212031.785],
        [458547.390,5212040.431],
        [458544.117,5212034.247],
        [458547.643,5212032.491],
        [458529.159,5211996.489],
        [458525.416,5211997.847],
        [458705.506,5212132.475],
        [458720.491,5212144.009],
        [458701.104,5212167.713],
        [458686.240,5212154.883],
        [458686.237,5212154.903],
        [458688.787,5212151.895],
        [458677.681,5212142.405],
        [458675.458,5212145.528]]
        
        self.WP_points = [[0,0], #Null to match
        [458415.533,5212053.371],
        [458432.631,5212085.990],
        [458444.210,5212082.601],
        [458435.119,5212045.155],
        [458444.476,5212041.296],
        [458465.789,5212080.096],
        [458473.177,5212078.853],
        [458462.807,5212032.994],
        [458476.411,5212034.889],
        [458491.986,5212069.811],
        [458504.076,5212066.038],
        [458496.486,5212027.220],
        [458508.204,5212018.866],
        [458536.608,5212050.748],
        [458552.882,5212042.402],
        [458536.217,5212000.747],
        [458566.372,5211984.765],
        [458593.852,5212005.883],
        [458774.090,5212151.755],
        [458772.292,5212162.015],
        [458737.154,5212211.922],
        [458709.852,5212218.030],
        [458681.909,5212188.470],
        [458659.238,5212168.854],
        [458704.775,5212090.987],
        [458753.997,5212135.949],
        [458738.332,5212154.075],
        [458732.759,5212145.598],
        [458711.194,5212111.179],
        [458678.496,5212157.663],
        [458699.580,5212174.966],
        [458714.882,5212156.495],
        [458727.869,5212166.048],
        [458714.747,5212168.981],
        [0,0], #To be define 
        [458558.650,5212037.420],
        [458549.476,5212023.779],
        [458385.279,5212079.634], #WP A
        [458412.312,5212090.911]] #WP B


        self.ZI_array = [[0],
        [5,11,27,10],
        [5,6,7,8,17,16,15,14,13,12,11],
        [8,9,18,17],
        [9,10,27,26,25,24,23,22,21,20,19,18],
        [1,2,3,4],
        [28,29,30,31]]

        self.ZE_array = [[0],
        [1,2,3,4],
        [5,6,7,8],
        [9,10,11,12],
        [13,14,15,16],
        [17,18,19,20],
        [21,22,23,24],
        [25,26,27,28],
        [29,30,31,32],
        [33,34,35,36],
        [37,38,39,40],
        [41,42,43,44],
        [45,46,47,48],
        [49,50,51,52],
        [53,54,55,56]]


    def ZEI_callback(self, ZEI):
        #Update tf
        self.update_tf()

        # Split array between ZI and ZE
        ZI, ZE = self.split_array(ZEI)
        print(ZI)
        print(ZE)

        for i in range(len(ZI)): #each zone   i in [1 , 4]
            rospy.loginfo("ZI Zone num: %d", ZI[i])
            
            poly = Polygon2()

            for y in range(len(self.ZI_array[ZI[i]])): # each point y in [1, 2, 3, 4]
                #rospy.loginfo("Point num: %d ", self.ZI_array[ZI[i]][y])
                #print([self.ZI_array[ZI[i]][y]][0])
                #print(self.ZI_points[[self.ZI_array[ZI[i]][y]][0]][0])
                point = Point32()
                point.x = self.ZI_points[[self.ZI_array[ZI[i]][y]][0]][0] - self.trans[0]
                point.y = self.ZI_points[[self.ZI_array[ZI[i]][y]][0]][1] - self.trans[1]
                
                poly.zone.points.append(point)
                #rospy.loginfo("x: %d | y: %d", poly.zone.points[y].x, poly.zone.points[y].y)

            self.ZEI_contatenate.zones.append(poly)
        
        for i in range(len(ZE)): #each zone   i in [1 , 4]
            rospy.loginfo("ZE Zone num: %d", ZE[i])
            
            poly = Polygon2()

            for y in range(len(self.ZE_array[ZE[i]])): # each point y in [1, 2, 3, 4]
                #rospy.loginfo("Point num: %d ", self.ZE_array[ZE[i]][y])
                #print([self.ZE_array[ZE[i]][y]][0])
                #print(self.ZE_points[[self.ZE_array[ZE[i]][y]][0]][0])

                point = Point32()
                point.x = self.ZE_points[[self.ZE_array[ZE[i]][y]][0]][0] - self.trans[0]
                point.y = self.ZE_points[[self.ZE_array[ZE[i]][y]][0]][1] - self.trans[1]
                
                poly.zone.points.append(point)
                #rospy.loginfo("x: %d | y: %d", poly.zone.points[y].x, poly.zone.points[y].y)

            self.ZEI_contatenate.zones.append(poly)

        self.ZEI_publisher.publish(self.ZEI_contatenate)
        rospy.loginfo("Number of zones: %d", len(self.ZEI_contatenate.zones))
        #print(self.ZEI_contatenate)
        self.ZEI_contatenate = Polygon2Array()


    def update_tf(self):
        # Obtenir la transformation entre le frame "UTM" et "map"
        (self.trans, self.rot) = self.listener.lookupTransform('UTM', 'map', rospy.Time(0))
    
    # Split array
    def split_array(self, concatenated_msg, separator=-1):
        try:
            # Trouver l'indice du séparateur
            concatenated_array = concatenated_msg.data
            separator_index = concatenated_array.index(separator)
            
            # Séparer les deux tableaux en utilisant l'indice du séparateur
            array1 = concatenated_array[:separator_index]  # Avant le séparateur
            array2 = concatenated_array[separator_index+1:]  # Après le séparateur
            
            return array1, array2
        
        except ValueError:
            # Si le séparateur n'est pas trouvé
            print("Séparateur non trouvé dans le tableau.")
            return concatenated_array, []  # Retourne tout dans le premier tableau


if __name__ == "__main__":
    try:
        # Initiate the node
        rospy.init_node('barakuda_mission_loader', anonymous=True)

        # Create Barakuda Socket object
        barakuda_mission_loader = BarakudaMissionLoader(rospy.get_name())
        
        rospy.loginfo("Barakuda Mission Loader: Start")

        # Spin at 10Hz while ROS OK
        while not rospy.is_shutdown():
            rate = rospy.Rate(20) # 20hz
            rate.sleep()
        
    except rospy.ROSInterruptException:
        
        rospy.loginfo("Barakuda Mission Loader Mesure: Stop")
