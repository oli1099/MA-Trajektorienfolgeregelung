#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.duration import Duration

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        # Abonniere das OccupancyGrid-Topic
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.occupancy_callback,
            10)
        # Publisher für die Marker, die in RViz2 angezeigt werden
        self.marker_pub = self.create_publisher(MarkerArray, '/obstacle_markers', 1)

    def occupancy_callback(self, msg):
        # Parameter aus der OccupancyGrid-Nachricht auslesen
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin = msg.info.origin  # geometry_msgs/Pose

        # Die Daten (1D Liste) in ein 2D-Array umwandeln
        grid = np.array(msg.data, dtype=np.int8).reshape((height, width))
        
        # Schwellenwert: Zellen mit Werten >= 50 als Hindernis definieren
        threshold = 50
        binary_grid = np.zeros_like(grid, dtype=np.uint8)
        binary_grid[grid >= threshold] = 255

        # Rauschunterdrückung via morphologischer Öffnung
        kernel = np.ones((3, 3), np.uint8)
        binary_grid = cv2.morphologyEx(binary_grid, cv2.MORPH_OPEN, kernel)

        # Connected-Components-Analyse (8er-Nachbarschaft)
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(binary_grid, connectivity=8)

        obstacles = []
        # Label 0 entspricht dem Hintergrund, daher starten wir bei 1
        for i in range(1, num_labels):
            # Bounding-Box-Parameter aus stats: links, oben, Breite, Höhe
            x = stats[i, cv2.CC_STAT_LEFT]
            y = stats[i, cv2.CC_STAT_TOP]
            w = stats[i, cv2.CC_STAT_WIDTH]
            h = stats[i, cv2.CC_STAT_HEIGHT]
            centroid = centroids[i]

            # Umrechnung der Gitterkoordinaten in Weltkoordinaten:
            world_x = origin.position.x + (centroid[0] * resolution)
            world_y = origin.position.y + (centroid[1] * resolution)
            
            # Abschätzung des Hindernisradius: hier als halbe Diagonale der Bounding Box
            radius = 0.5 * np.sqrt((w * resolution)**2 + (h * resolution)**2)
            obstacles.append({'x': world_x, 'y': world_y, 'radius': radius})
        
        self.publish_markers(obstacles, msg.header)

    def publish_markers(self, obstacles, header):
        marker_array = MarkerArray()
        # Erstelle für jedes Hindernis einen Marker (Sphäre)
        for i, obs in enumerate(obstacles):
            marker = Marker()
            marker.header = header
            marker.ns = 'obstacles'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = obs['x']
            marker.pose.position.y = obs['y']
            marker.pose.position.z = 0.0  # Annahme: 2D-Plan
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            # Skalierung: Durchmesser der Sphäre (2 * Radius)
            marker.scale.x = obs['radius'] * 2
            marker.scale.y = obs['radius'] * 2
            marker.scale.z = 0.2  # Kleine Höhe zur Visualisierung
            # Farbe: Rot, halbtransparent
            marker.color.a = 0.8
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            # Lebensdauer des Markers (dann wird er automatisch gelöscht, wenn er nicht erneuert wird)
            marker.lifetime = Duration(seconds=0.5).to_msg()
            marker_array.markers.append(marker)
        
        self.marker_pub.publish(marker_array)
        self.get_logger().info('Veröffentlichte %d Hindernis-Marker.' % len(obstacles))

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
