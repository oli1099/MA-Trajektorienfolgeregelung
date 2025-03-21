#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from sklearn.cluster import DBSCAN
import math

class LidarClustering(Node):
    def __init__(self):
        super().__init__('lidar_clustering_node')
        # Abonnieren des Lidar-Topics
        self.subscription = self.create_subscription(
            LaserScan,
            '/ldlidar_node/scan',
            self.scan_callback,
            10
        )
        # Publisher für Visualisierungsmarker (z.B. in RViz)
        self.marker_pub = self.create_publisher(MarkerArray, 'lidar_clusters', 10)
    
    def scan_callback(self, msg):
        
        ranges = np.array(msg.ranges)
        print(np.isnan(ranges).sum(), np.isinf(ranges).sum())
        valid = np.isfinite(ranges)
        ranges = ranges[valid]
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        angles_valid = angles[valid]
        
        # Umwandeln der LaserScan-Daten in 2D-Koordinaten
        #angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        xs = np.array(msg.ranges) * np.cos(angles_valid)
        ys = np.array(msg.ranges) * np.sin(angles_valid)
        points = np.vstack((xs, ys)).T
        
        # Anwenden von DBSCAN zum Clustern der Punkte
        clustering = DBSCAN(eps=0.1, min_samples=5).fit(points)
        labels = clustering.labels_
        
        marker_array = MarkerArray()
        marker_id = 0
        # Iteriere über alle gefundenen Cluster
        for cluster_id in set(labels):
            # Ausreißer (Label -1) ignorieren
            if cluster_id == -1:
                continue
            cluster_points = points[labels == cluster_id]
            
            # Berechnung des minimalen umschließenden Rechtecks:
            x_min, y_min = cluster_points.min(axis=0)
            x_max, y_max = cluster_points.max(axis=0)
            # Erzeugen von Marker-Punkten für das Rechteck
            rect_points = [
                self.create_point(x_min, y_min),
                self.create_point(x_max, y_min),
                self.create_point(x_max, y_max),
                self.create_point(x_min, y_max),
                self.create_point(x_min, y_min)  # Schließen des Rechtecks
            ]
            
            marker = Marker()
            marker.header.frame_id = msg.header.frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "clusters"
            marker.id = marker_id
            marker.type = Marker.LINE_STRIP  # Linie, um das Rechteck darzustellen
            marker.action = Marker.ADD
            marker.scale.x = 0.05  # Linienbreite
            marker.color.a = 1.0
            marker.color.r = 1.0  # Rot – kann angepasst werden
            marker.points = rect_points
            marker_array.markers.append(marker)
            marker_id += 1
            
            # Alternativ: Berechnung eines umschließenden Kreises
            # Hier kannst du den Schwerpunkt des Clusters und den maximalen Abstand zu diesem Mittelpunkt berechnen.
            # center = np.mean(cluster_points, axis=0)
            # radius = np.max(np.linalg.norm(cluster_points - center, axis=1))
            # Anschließend kannst du einen Kreis als Marker (z.B. Marker.LINE_STRIP mit kreisförmigen Punkten) darstellen.
        
        self.marker_pub.publish(marker_array)
    
    def create_point(self, x, y):
        from geometry_msgs.msg import Point
        p = Point()
        p.x = x
        p.y = y
        p.z = 0.0
        return p

def main(args=None):
    rclpy.init(args=args)
    node = LidarClustering()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
