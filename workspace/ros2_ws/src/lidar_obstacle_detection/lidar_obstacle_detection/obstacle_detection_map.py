#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import numpy as np
import cv2  # Stelle sicher, dass OpenCV (python3-opencv) installiert ist

class MapObstacleDetection(Node):
    def __init__(self):
        super().__init__('map_obstacle_detection_node')
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        self.marker_pub = self.create_publisher(MarkerArray, 'map_obstacles', 10)

    def map_callback(self, msg):
        # Informationen zur Karte
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        # Umwandeln der Kartendaten in ein 2D-Numpy-Array
        data = np.array(msg.data).reshape((height, width))
        # Schwellwert: Werte >50 (von 100) werden als Hindernis interpretiert, -1 = unbekannt werden ignoriert
        obstacle_mask = (data > 50)
        # Erstellen eines binären Bildes: Hindernisse = 255, freier Raum = 0
        obstacle_img = np.zeros_like(data, dtype=np.uint8)
        obstacle_img[obstacle_mask] = 255

        # Extrahieren der Konturen der Hindernisse mittels OpenCV
        contours, _ = cv2.findContours(obstacle_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        marker_array = MarkerArray()
        marker_id = 0

        for cnt in contours:
            # Kleine Konturen ignorieren (optional)
            if cv2.contourArea(cnt) < 10:
                continue
            
             # Berechne den Schwerpunkt der Kontur
            M = cv2.moments(cnt)
            if M["m00"] == 0:
                continue
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            
            # Umrechnung von Pixelkoordinaten in Weltkoordinaten:
            # (cX, cY) beziehen sich auf die Bildkoordinaten, wobei cX die Spalte und cY die Zeile ist.
            world_x = origin_x + (cX * resolution)
            # Da in der OccupancyGrid die erste Zeile oben liegt, muss die Zeile invertiert werden:
            world_y = origin_y + ((height - cY) * resolution)

            # Logge die Hindernisposition
            self.get_logger().info(
                f"Hindernis {marker_id}: Position in Weltkoordinaten: ({world_x:.2f}, {world_y:.2f})"
            )

            # Optional: Kontur glätten bzw. vereinfachen
            epsilon = 0.01 * cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, epsilon, True)

            marker = Marker()
            marker.header.frame_id = msg.header.frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "map_obstacles"
            marker.id = marker_id
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.05  # Linienbreite
            marker.color.a = 1.0
            marker.color.r = 1.0  # Rot als Farbe

            # Konvertiere die Pixelkoordinaten (Spalte, Zeile) in Weltkoordinaten
            for point in approx:
                # point hat die Form [[col, row]]
                col = point[0][0]
                row = point[0][1]
                # Umrechnung: x = origin_x + Spalte * resolution
                # Bei y muss häufig die Umkehrung der Reihenfolge berücksichtigt werden, da die Karte
                # typischerweise von oben nach unten durchlaufen wird.
                x = origin_x + (col * resolution)
                y = origin_y + ((height - row) * resolution)
                marker.points.append(self.create_point(x, y))
            # Schließe den Marker (Rechteck/Polygonstruktur)
            if marker.points:
                marker.points.append(marker.points[0])
            marker_array.markers.append(marker)
            marker_id += 1

        self.marker_pub.publish(marker_array)

    def create_point(self, x, y):
        p = Point()
        p.x = x
        p.y = y
        p.z = 0.0
        return p

def main(args=None):
    rclpy.init(args=args)
    node = MapObstacleDetection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
