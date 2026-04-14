#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import cv2
import numpy as np

def compressed_image_callback(msg):
    # Créer un objet CvBridge pour la conversion
    bridge = CvBridge()
    
    # Convertir le message CompressedImage en un format OpenCV (np.array)
    np_arr = np.frombuffer(msg.data, np.uint8)  # Convertir les données en un tableau numpy
    cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # Décoder l'image

    # Convertir l'image OpenCV en message Image (format non compressé)
    ros_image = bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")

    # À ce moment, tu peux publier ou traiter ros_image comme tu veux.
    # Par exemple, tu peux publier l'image convertie dans un topic.
    image_pub.publish(ros_image)

def main():
    # Initialisation du nœud ROS
    rospy.init_node('image_converter')
    
    # Création d'un abonné pour le topic de l'image compressée
    rospy.Subscriber('/axis/image_raw/compressed', CompressedImage, compressed_image_callback)
    
    # Création d'un éditeur pour publier l'image convertie
    global image_pub
    image_pub = rospy.Publisher('/axis/image_raw', Image, queue_size=10)
    
    rospy.spin()

if __name__ == '__main__':
    main()
