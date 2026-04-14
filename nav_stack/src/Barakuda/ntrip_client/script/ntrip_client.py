#!/usr/bin/env python3

# Python import
import io
import socket
import base64
import pyrtcm
import pymap3d as pm  # Pour conversion ECEF -> WGS84
from pyproj import Proj, Transformer

# ROS Import
import rospy
import tf2_ros
import geometry_msgs.msg
from rtcm_msgs.msg import Message



class NTRIPClient:
    def __init__(self):
        rospy.init_node("ntrip_client_node")

        """
        Connects to an NTRIP caster and publishes RTCM data as ROS messages.

        :param server: NTRIP caster hostname or IP address.
        :param port: NTRIP caster port (e.g., 2101).
        :param mountpoint: Mountpoint to request from the NTRIP caster.
        :param topic: ROS topic name to publish RTCM messages.
        :param username: Optional username for NTRIP authentication.
        :param password: Optional password for NTRIP authentication.
        """

        # Charger les paramètres depuis le serveur ROS
        self.server = rospy.get_param("~server", "your-ntrip-server.com")
        self.port = rospy.get_param("~port", 2101)
        self.mountpoint = rospy.get_param("~mountpoint", "your-mountpoint")
        self.topic = rospy.get_param("~topic", "/ntrip_client/rtcm")
        self.username = rospy.get_param("~username", None)
        self.password = rospy.get_param("~password", None)
        self.utm_zone = rospy.get_param("~utm_zone", 31)
        self.is_south = rospy.get_param("~is_south", False)
        self.UTM_frame_id = rospy.get_param("~UTM_frame_id", "utm")

        # Création du publisher ROS pour les messages RTCM
        self.pub = rospy.Publisher(self.topic, Message, queue_size=10)

        #Static TF publisher
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster()

        # Initialisation de la connexion NTRIP
        self.sock = None
        self.connect_to_ntrip()

        # Other class attributes
        self.init_base_station = False

    def connect_to_ntrip(self):
        """ Établit une connexion avec le serveur NTRIP """
        rospy.loginfo(f"Connexion au serveur NTRIP {self.server}:{self.port}, mountpoint {self.mountpoint}...")

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.sock.connect((self.server, self.port))

            # Construire et envoyer la requête HTTP NTRIP
            request = f"GET /{self.mountpoint} HTTP/1.1\r\n"
            request += f"Host: {self.server}:{self.port}\r\n"
            request += "User-Agent: NTRIP PythonClient/1.0\r\n"
            request += "Ntrip-Version: Ntrip/1.0\r\n"
            if self.username and self.password:
                credentials = f"{self.username}:{self.password}"
                encoded_credentials = base64.b64encode(credentials.encode()).decode()
                request += f"Authorization: Basic {encoded_credentials}\r\n"
            request += "\r\n"

            self.sock.send(request.encode())

            # Vérifier la réponse du serveur
            response = self.sock.recv(4096).decode()
            if "200 OK" not in response:
                rospy.logerr(f"Échec d'authentification: {response}")
                self.sock.close()
                return
            
            rospy.loginfo("Connecté au caster NTRIP. Streaming RTCM data...")

        except Exception as e:
            rospy.logerr(f"Échec de connexion au serveur NTRIP: {e}")
            return

    def receive_data(self):
     
        data = self.sock.recv(1024)
        if not data:
            rospy.logwarn("Connexion NTRIP fermée.")
        
        # Publier le message RTCM sur ROS
        msg = Message()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "ntrip_rtcm"
        msg.message = data
        self.pub.publish(msg)

        # Extraction de la position de la base
        if not self.init_base_station:
            self.init_base_position(msg.message)

        #rospy.loginfo(f"Publié {len(data)} octets de données RTCM.")

    def close_sock(self):
        self.sock.close()

    def init_base_position(self, rtcm_data):
        """ Extrait et affiche la position de la station de base RTK """
        stream = io.BytesIO(rtcm_data)
        parser = pyrtcm.RTCMReader(stream)

        for raw_data, parsed_data in parser:
            if parsed_data.identity in ["1005", "1006"]:
                
                # Extraction des coordonnées ECEF
                x_ecef = parsed_data.DF025
                y_ecef = parsed_data.DF026
                z_ecef = parsed_data.DF027

                # Vérification des valeurs
                if x_ecef == 0 or y_ecef == 0 or z_ecef == 0:
                    rospy.logwarn("Coordonnées ECEF invalides.")
                    continue

                # Conversion en Latitude, Longitude, Altitude (WGS84)
                lat, lon, alt = pm.ecef2geodetic(x_ecef, y_ecef, z_ecef)

                # Affichage de la position
                rospy.loginfo(f"Position de la base RTK :")
                rospy.loginfo(f"- Latitude  : {lat:.6f}°")
                rospy.loginfo(f"- Longitude : {lon:.6f}°")
                rospy.loginfo(f"- Altitude  : {alt:.2f} m")

                # Set init at true
                self.init_base_station = True

                utm_proj = Proj(proj="utm", zone=self.utm_zone, ellps="WGS84", south=self.is_south)
                transformer = Transformer.from_proj(Proj("epsg:4326"), utm_proj, always_xy=True)

                # Convertir les coordonnées WGS84 en UTM
                east, north = transformer.transform(lon, lat)

                # Construire le message TF statique
                t = geometry_msgs.msg.TransformStamped()
                t.header.stamp = rospy.Time.now()
                t.header.frame_id = self.UTM_frame_id
                t.child_frame_id = "map_rtk"

                # Position de la base RTK en UTM
                t.transform.translation.x = east
                t.transform.translation.y = north
                t.transform.translation.z = alt

                # Pas de rotation (on garde une orientation neutre)
                t.transform.rotation.x = 0.0
                t.transform.rotation.y = 0.0
                t.transform.rotation.z = 0.0
                t.transform.rotation.w = 1.0

                self.static_tf_broadcaster.sendTransform(t)
                rospy.loginfo("TF statique publié entre /UTM et /map_rtk")

if __name__ == "__main__":
    client = NTRIPClient()

    rate = rospy.Rate(10)

    try:
        while not rospy.is_shutdown():
            client.receive_data()
            rate.sleep()
    except rospy.ROSInterruptException:
        rospy.loginfo("Noeud interrompu.")
    finally:
        client.close_sock()
        rospy.loginfo("Connexion fermée.")
