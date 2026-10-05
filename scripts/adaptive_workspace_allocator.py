#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid


# Preferred positions around the Hilti robot.
# These are ONLY preferred positions.
# If one is blocked, the program searches nearby.
PREFERRED_OFFSETS = {
    'tb2': (4.2, 3.4),
    'tb3': (-4.2, -3.4),
    'tb4': (4.2, -3.4),
    'tb5': (-4.2, 3.4),
}

# Space we want around each support robot.
CLEARANCE_RADIUS = 0.35

# Maximum distance from a blocked preferred position
# that we allow the program to search.
MAX_SEARCH_RADIUS = 3.0


class AdaptiveWorkspaceAllocator(Node):

    def __init__(self):
        super().__init__('adaptive_workspace_allocator')

        self.costmap = None

        # Receive Hilti's global Nav2 costmap.
        self.create_subscription(
            OccupancyGrid,
            '/tb1/global_costmap/costmap',
            self.costmap_callback,
            10
        )

        # Test workspace goal.
        # This DOES NOT move any robot.
        self.create_subscription(
            PoseStamped,
            '/adaptive_workspace_goal',
            self.goal_callback,
            10
        )

        self.get_logger().info(
            'Adaptive allocator running in DRY-RUN mode.'
        )

    def costmap_callback(self, msg):
        self.costmap = msg

    def world_to_grid(self, x, y):
        """Convert map coordinates in metres to costmap cells."""

        gx = int(
            (x - self.costmap.info.origin.position.x)
            / self.costmap.info.resolution
        )

        gy = int(
            (y - self.costmap.info.origin.position.y)
            / self.costmap.info.resolution
        )

        return gx, gy

    def cell_free(self, gx, gy):
        """Check whether one costmap cell is completely free."""

        if gx < 0 or gy < 0:
            return False

        if gx >= self.costmap.info.width:
            return False

        if gy >= self.costmap.info.height:
            return False

        index = gy * self.costmap.info.width + gx
        cost = self.costmap.data[index]

        return 0 <= cost <= 40

    def position_safe(self, x, y):
        """
        Check whether a position and the area around it are free.
        """

        gx, gy = self.world_to_grid(x, y)

        radius_cells = int(
            CLEARANCE_RADIUS / self.costmap.info.resolution
        )

        for dx in range(-radius_cells, radius_cells + 1):

            for dy in range(-radius_cells, radius_cells + 1):

                # Use a circular clearance region.
                if math.hypot(dx, dy) > radius_cells:
                    continue

                if not self.cell_free(gx + dx, gy + dy):
                    return False

        return True

    def find_safe_position(self, preferred_x, preferred_y):
        """
        Keep preferred position if possible.
        Otherwise search nearby for the closest safe position.
        """

        if self.position_safe(preferred_x, preferred_y):
            return preferred_x, preferred_y, 0.0

        resolution = self.costmap.info.resolution

        center_gx, center_gy = self.world_to_grid(
            preferred_x,
            preferred_y
        )

        max_cells = int(
            MAX_SEARCH_RADIUS / resolution
        )

        best_position = None
        best_distance = float('inf')

        for dx in range(-max_cells, max_cells + 1):

            for dy in range(-max_cells, max_cells + 1):

                # Ignore positions outside the search circle.
                if math.hypot(dx, dy) > max_cells:
                    continue

                gx = center_gx + dx
                gy = center_gy + dy

                x = (
                    self.costmap.info.origin.position.x
                    + gx * resolution
                )

                y = (
                    self.costmap.info.origin.position.y
                    + gy * resolution
                )

                distance = math.hypot(
                    x - preferred_x,
                    y - preferred_y
                )

                # No reason to check a position farther away
                # than our current best result.
                if distance >= best_distance:
                    continue

                if self.position_safe(x, y):
                    best_position = (x, y)
                    best_distance = distance

        if best_position is None:
            return None

        return (
            best_position[0],
            best_position[1],
            best_distance
        )

    def goal_callback(self, msg):

        if self.costmap is None:
            self.get_logger().warning(
                'Waiting for costmap...'
            )
            return

        hilti_x = msg.pose.position.x
        hilti_y = msg.pose.position.y

        print()
        print('======================================')
        print('      ADAPTIVE WORKSPACE ALLOCATION')
        print('======================================')

        print(
            f'Hilti workspace center: '
            f'({hilti_x:.2f}, {hilti_y:.2f})'
        )

        for robot, offset in PREFERRED_OFFSETS.items():

            dx, dy = offset

            preferred_x = hilti_x + dx
            preferred_y = hilti_y + dy

            result = self.find_safe_position(
                preferred_x,
                preferred_y
            )

            print()
            print(robot)

            print(
                f'  Preferred: '
                f'({preferred_x:.2f}, {preferred_y:.2f})'
            )

            if result is None:

                print(
                    '  Result: NO SAFE POSITION FOUND'
                )

                continue

            safe_x, safe_y, shift = result

            print(
                f'  Selected:  '
                f'({safe_x:.2f}, {safe_y:.2f})'
            )

            if shift < 0.01:

                print(
                    '  Status: preferred position is safe'
                )

            else:

                print(
                    f'  Status: adapted by {shift:.2f} m'
                )

        print()
        print('DRY RUN ONLY - NO ROBOTS WERE MOVED')
        print('======================================')
        print()


def main(args=None):

    rclpy.init(args=args)

    node = AdaptiveWorkspaceAllocator()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()