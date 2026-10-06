#!/usr/bin/env python3
"""Relay fleet data into a distinct TF tree for a single RViz window."""
from copy import deepcopy
from functools import partial
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path, OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from geometry_msgs.msg import PolygonStamped


ROOT = '/fleet_visualization'


def frame(robot, value):
    value = value.lstrip('/')
    return value if not value or value == 'map' or value.startswith(robot + '/') else robot + '/' + value


class FleetRvizRelay(Node):
    def __init__(self):
        super().__init__('fleet_rviz_relay')
        self.static_cache = {}
        self.subscriptions_kept = []
        static_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                reliability=ReliabilityPolicy.RELIABLE)
        self.tf_pub = self.create_publisher(TFMessage, ROOT + '/tf', 100)
        self.static_pub = self.create_publisher(TFMessage, ROOT + '/tf_static', static_qos)
        for robot in ('tb1', 'tb2', 'tb3', 'tb4', 'tb5'):
            for suffix, qos in (('tf', 100), ('tf_static', static_qos)):
                self.subscriptions_kept.append(self.create_subscription(
                    TFMessage, f'/{robot}/{suffix}', partial(self.transform, robot=robot,
                    static=suffix == 'tf_static'), qos))
            scan_pub = self.create_publisher(LaserScan, f'{ROOT}/{robot}/scan', qos_profile_sensor_data)
            self.subscriptions_kept.append(self.create_subscription(LaserScan, f'/{robot}/scan',
                partial(self.relay, robot=robot, publisher=scan_pub), qos_profile_sensor_data))
            for suffix in ('plan', 'local_plan', 'plan_smoothed'):
                publisher = self.create_publisher(Path, f'{ROOT}/{robot}/{suffix}', 10)
                self.subscriptions_kept.append(self.create_subscription(Path, f'/{robot}/{suffix}',
                    partial(self.relay, robot=robot, publisher=publisher), qos_profile_sensor_data))
            for scope in ('global_costmap', 'local_costmap'):
                footprint_topic = f'{scope}/published_footprint'
                footprint_pub = self.create_publisher(
                    PolygonStamped, f'{ROOT}/{robot}/{footprint_topic}', 10)
                self.subscriptions_kept.append(self.create_subscription(
                    PolygonStamped, f'/{robot}/{footprint_topic}',
                    partial(self.relay, robot=robot, publisher=footprint_pub),
                    qos_profile_sensor_data))
                suffix = f'{scope}/costmap'
                publisher = self.create_publisher(OccupancyGrid, f'{ROOT}/{robot}/{suffix}', static_qos)
                self.subscriptions_kept.append(self.create_subscription(
                    OccupancyGrid, f'/{robot}/{suffix}',
                    partial(self.relay, robot=robot, publisher=publisher), static_qos))
                updates = self.create_publisher(OccupancyGridUpdate, f'{ROOT}/{robot}/{suffix}_updates', 10)
                self.subscriptions_kept.append(self.create_subscription(
                    OccupancyGridUpdate, f'/{robot}/{suffix}_updates',
                    partial(self.relay, robot=robot, publisher=updates), qos_profile_sensor_data))
        self.get_logger().info('Fleet visualization relay ready for tb1–tb5')

    def transform(self, message, robot, static):
        message = deepcopy(message)
        for transform in message.transforms:
            transform.header.frame_id = frame(robot, transform.header.frame_id)
            transform.child_frame_id = frame(robot, transform.child_frame_id)
        if static:
            for transform in message.transforms:
                self.static_cache[transform.child_frame_id] = transform
            self.static_pub.publish(TFMessage(transforms=list(self.static_cache.values())))
        else:
            self.tf_pub.publish(message)

    def relay(self, message, robot, publisher):
        message = deepcopy(message)
        message.header.frame_id = frame(robot, message.header.frame_id)
        if isinstance(message, Path):
            for pose in message.poses:
                pose.header.frame_id = frame(robot, pose.header.frame_id)
        publisher.publish(message)


def main():
    rclpy.init()
    node = FleetRvizRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
