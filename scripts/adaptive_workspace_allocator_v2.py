#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import ComputePathToPose


PREFERRED_OFFSETS = {
    'tb2': (4.2,  3.4),
    'tb3': (-4.2, -3.4),
    'tb4': (4.2, -3.4),
    'tb5': (-4.2,  3.4),
}

CLEARANCE_RADIUS = 0.35
MAX_SEARCH_RADIUS = 3.0
MAX_ACCEPTABLE_COST = 40


class AdaptiveWorkspaceAllocatorV2(Node):

    def __init__(self):
        super().__init__('adaptive_workspace_allocator_v2')

        self.costmap = None
        self.busy = False
        self.pending_checks = 0
        self.current_results = {}

        self.create_subscription(
            OccupancyGrid,
            '/tb1/global_costmap/costmap',
            self.costmap_callback,
            10
        )

        self.create_subscription(
            PoseStamped,
            '/adaptive_workspace_goal',
            self.workspace_goal_callback,
            10
        )

        self.planner_clients = {
            robot: ActionClient(
                self,
                ComputePathToPose,
                f'/{robot}/compute_path_to_pose'
            )
            for robot in PREFERRED_OFFSETS
        }

        self.get_logger().info(
            'Adaptive allocator V2 running in DRY-RUN mode.'
        )
        self.get_logger().info(
            'It checks costmap safety + Nav2 reachability, but moves no robots.'
        )

    def costmap_callback(self, msg):
        self.costmap = msg

    def world_to_grid(self, x, y):
        gx = int(
            (x - self.costmap.info.origin.position.x)
            / self.costmap.info.resolution
        )
        gy = int(
            (y - self.costmap.info.origin.position.y)
            / self.costmap.info.resolution
        )
        return gx, gy

    def cell_acceptable(self, gx, gy):
        if self.costmap is None:
            return False
        if gx < 0 or gy < 0:
            return False
        if gx >= self.costmap.info.width:
            return False
        if gy >= self.costmap.info.height:
            return False

        index = gy * self.costmap.info.width + gx
        cost = self.costmap.data[index]

        return 0 <= cost <= MAX_ACCEPTABLE_COST

    def position_safe(self, x, y):
        gx, gy = self.world_to_grid(x, y)

        radius_cells = max(
            1,
            int(CLEARANCE_RADIUS / self.costmap.info.resolution)
        )

        for dx in range(-radius_cells, radius_cells + 1):
            for dy in range(-radius_cells, radius_cells + 1):

                if math.hypot(dx, dy) > radius_cells:
                    continue

                if not self.cell_acceptable(gx + dx, gy + dy):
                    return False

        return True

    def find_safe_position(self, preferred_x, preferred_y):

        if self.position_safe(preferred_x, preferred_y):
            return preferred_x, preferred_y, 0.0

        resolution = self.costmap.info.resolution
        center_gx, center_gy = self.world_to_grid(
            preferred_x,
            preferred_y
        )

        max_cells = int(MAX_SEARCH_RADIUS / resolution)

        best_position = None
        best_distance = float('inf')

        for dx in range(-max_cells, max_cells + 1):
            for dy in range(-max_cells, max_cells + 1):

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

                if distance >= best_distance:
                    continue

                if self.position_safe(x, y):
                    best_position = (x, y)
                    best_distance = distance

        if best_position is None:
            return None

        return best_position[0], best_position[1], best_distance

    def workspace_goal_callback(self, msg):

        if self.costmap is None:
            self.get_logger().warning(
                'No costmap received yet. Try again in a moment.'
            )
            return

        if self.busy:
            self.get_logger().warning(
                'A workspace reachability check is already running.'
            )
            return

        self.busy = True
        self.pending_checks = 0
        self.current_results = {}

        hilti_x = msg.pose.position.x
        hilti_y = msg.pose.position.y

        print()
        print('================================================')
        print('   ADAPTIVE WORKSPACE ALLOCATION V2 - DRY RUN')
        print('================================================')
        print(
            f'Hilti workspace center: ({hilti_x:.2f}, {hilti_y:.2f})'
        )
        print()

        for robot, (dx, dy) in PREFERRED_OFFSETS.items():

            preferred_x = hilti_x + dx
            preferred_y = hilti_y + dy

            safe_result = self.find_safe_position(
                preferred_x,
                preferred_y
            )

            print(f'{robot}:')
            print(
                f'  Preferred: ({preferred_x:.2f}, {preferred_y:.2f})'
            )

            if safe_result is None:
                print(
                    f'  Safety:    NO SAFE POSITION within '
                    f'{MAX_SEARCH_RADIUS:.1f} m'
                )
                print('  Reachable: NOT CHECKED')
                print()

                self.current_results[robot] = {
                    'safe': False,
                    'reachable': False,
                    'reason': 'no_safe_position'
                }
                continue

            selected_x, selected_y, shift = safe_result

            print(
                f'  Selected:  ({selected_x:.2f}, {selected_y:.2f})'
            )

            if shift < 0.01:
                print('  Safety:    preferred position accepted')
            else:
                print(
                    f'  Safety:    ADAPTED by {shift:.2f} m'
                )

            self.current_results[robot] = {
                'safe': True,
                'reachable': None,
                'preferred': (preferred_x, preferred_y),
                'selected': (selected_x, selected_y),
                'shift': shift
            }

            self.request_path_check(
                robot,
                selected_x,
                selected_y,
                msg.pose.orientation
            )

        if self.pending_checks == 0:
            self.finish_workspace_check()
        else:
            print('Waiting for Nav2 reachability results...')
            print()

    def request_path_check(
        self,
        robot,
        target_x,
        target_y,
        orientation
    ):
        client = self.planner_clients[robot]

        if not client.server_is_ready():
            print(
                f'  Reachable: planner server for {robot} is not ready'
            )
            self.current_results[robot]['reachable'] = False
            self.current_results[robot]['reason'] = 'planner_not_ready'
            return

        goal_msg = ComputePathToPose.Goal()

        goal_msg.goal.header.frame_id = 'map'
        goal_msg.goal.header.stamp = self.get_clock().now().to_msg()

        goal_msg.goal.pose.position.x = target_x
        goal_msg.goal.pose.position.y = target_y
        goal_msg.goal.pose.position.z = 0.0
        goal_msg.goal.pose.orientation = orientation

        goal_msg.planner_id = ''
        goal_msg.use_start = False

        self.pending_checks += 1

        future = client.send_goal_async(goal_msg)
        future.add_done_callback(
            lambda f, r=robot:
            self.path_goal_response_callback(r, f)
        )

    def path_goal_response_callback(self, robot, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            print(
                f'{robot} reachability: ERROR sending planning goal: {exc}'
            )
            self.current_results[robot]['reachable'] = False
            self.current_results[robot]['reason'] = 'send_goal_exception'
            self.one_check_finished()
            return

        if goal_handle is None or not goal_handle.accepted:
            print(
                f'{robot} reachability: Nav2 planner REJECTED the goal'
            )
            self.current_results[robot]['reachable'] = False
            self.current_results[robot]['reason'] = 'goal_rejected'
            self.one_check_finished()
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f, r=robot:
            self.path_result_callback(r, f)
        )

    def path_result_callback(self, robot, future):
        try:
            wrapped_result = future.result()
            status = wrapped_result.status
            result = wrapped_result.result
        except Exception as exc:
            print(
                f'{robot} reachability: ERROR receiving result: {exc}'
            )
            self.current_results[robot]['reachable'] = False
            self.current_results[robot]['reason'] = 'result_exception'
            self.one_check_finished()
            return

        if status == GoalStatus.STATUS_SUCCEEDED:
            path_points = len(result.path.poses)

            self.current_results[robot]['reachable'] = True
            self.current_results[robot]['path_points'] = path_points
            self.current_results[robot]['reason'] = 'success'

            print(
                f'{robot} reachability: REACHABLE '
                f'({path_points} path poses)'
            )

        else:
            self.current_results[robot]['reachable'] = False
            self.current_results[robot]['reason'] = (
                f'action_status_{status}'
            )

            print(
                f'{robot} reachability: UNREACHABLE '
                f'(Nav2 action status {status})'
            )

        self.one_check_finished()

    def one_check_finished(self):
        self.pending_checks -= 1

        if self.pending_checks <= 0:
            self.finish_workspace_check()

    def finish_workspace_check(self):
        print()
        print('---------------- FINAL DRY-RUN SUMMARY ----------------')

        for robot in PREFERRED_OFFSETS:
            result = self.current_results.get(robot)

            if result is None:
                print(f'{robot}: NO RESULT')
                continue

            if not result.get('safe', False):
                print(f'{robot}: UNSAFE - no usable position selected')
                continue

            x, y = result['selected']

            if result.get('reachable') is True:
                print(
                    f'{robot}: SAFE + REACHABLE at '
                    f'({x:.2f}, {y:.2f})'
                )
            elif result.get('reachable') is False:
                print(
                    f'{robot}: SAFE BUT NOT CONFIRMED REACHABLE at '
                    f'({x:.2f}, {y:.2f})'
                )
            else:
                print(
                    f'{robot}: SAFE, reachability not completed at '
                    f'({x:.2f}, {y:.2f})'
                )

        print('-------------------------------------------------------')
        print('DRY RUN ONLY - NO NAVIGATION GOALS WERE EXECUTED.')
        print('=======================================================')
        print()

        self.busy = False


def main(args=None):
    rclpy.init(args=args)

    node = AdaptiveWorkspaceAllocatorV2()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
