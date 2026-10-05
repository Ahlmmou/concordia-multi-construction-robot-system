#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import ComputePathToPose


# Preferred rectangular support layout around the Hilti workspace center.
# These are preferences, not hard constraints.
PREFERRED_OFFSETS = {
    'tb2': (4.2,  3.4),
    'tb3': (-4.2, -3.4),
    'tb4': (4.2, -3.4),
    'tb5': (-4.2,  3.4),
}

# Prototype safety values. We will later replace/tune these using the
# real robot footprint and Nav2 costmap parameters.
CLEARANCE_RADIUS = 0.35
MAX_SEARCH_RADIUS = 3.0
MAX_ACCEPTABLE_COST = 40

# Candidate positions are sampled in rings around each preferred point.
# Smaller values give finer search but require more planning checks.
CANDIDATE_STEP = 0.20

# For this prototype, treat each selected support location as a clearance
# disc. Two selected support centers must not overlap those discs.
MIN_SUPPORT_SEPARATION = 1.10


class AdaptiveWorkspaceAllocatorV3(Node):

    def __init__(self):
        super().__init__('adaptive_workspace_allocator_v3')

        self.costmaps = {
            robot: None for robot in PREFERRED_OFFSETS
        }
        self.busy = False

        for robot in PREFERRED_OFFSETS:
            self.create_subscription(
                OccupancyGrid,
                f'/{robot}/global_costmap/costmap',
                lambda msg, r=robot: self.costmap_callback(msg, r),
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

        # State for one dry-run allocation.
        self.hilti_goal = None
        self.robot_order = list(PREFERRED_OFFSETS.keys())
        self.robot_index = 0
        self.results = {}
        self.accepted_positions = {}

        self.current_robot = None
        self.current_preferred = None
        self.current_candidates = []
        self.current_candidate_index = 0

        self.get_logger().info(
            'Adaptive allocator V3 running in DRY-RUN mode.'
        )
        self.get_logger().info(
            'V3 selects targets that are safe, Nav2-reachable, '
            'and non-overlapping.'
        )
        self.get_logger().info(
            'No navigation goals are executed.'
        )

    def costmap_callback(self, msg, robot):
        self.costmaps[robot] = msg

    def world_to_grid(self, robot, x, y):
        costmap = self.costmaps[robot]

        gx = int(
            (x - costmap.info.origin.position.x)
            / costmap.info.resolution
        )
        gy = int(
            (y - costmap.info.origin.position.y)
            / costmap.info.resolution
        )
        return gx, gy

    def cell_acceptable(self, robot, gx, gy):
        costmap = self.costmaps[robot]

        if costmap is None:
            return False

        if gx < 0 or gy < 0:
            return False

        if gx >= costmap.info.width or gy >= costmap.info.height:
            return False

        index = gy * costmap.info.width + gx
        cost = costmap.data[index]

        # Unknown cells (-1) and high-cost cells are rejected.
        return 0 <= cost <= MAX_ACCEPTABLE_COST

    def position_safe(self, robot, x, y):
        """
        Require every costmap cell inside a circular clearance region
        around the proposed target to be acceptable.
        """
        costmap = self.costmaps[robot]
        gx, gy = self.world_to_grid(robot, x, y)

        radius_cells = max(
            1,
            int(CLEARANCE_RADIUS / costmap.info.resolution)
        )

        for dx in range(-radius_cells, radius_cells + 1):
            for dy in range(-radius_cells, radius_cells + 1):

                if math.hypot(dx, dy) > radius_cells:
                    continue

                if not self.cell_acceptable(robot, gx + dx, gy + dy):
                    return False

        return True

    def separated_from_accepted(self, x, y):
        """
        Prevent two support robots from being assigned overlapping
        clearance discs.
        """
        for other_robot, (ox, oy) in self.accepted_positions.items():
            distance = math.hypot(x - ox, y - oy)

            if distance < MIN_SUPPORT_SEPARATION:
                return False

        return True

    def generate_candidates(self, robot, preferred_x, preferred_y):
        """
        Generate safe candidate locations ordered approximately from
        closest to farthest from the preferred support position.

        Candidate 0 is the preferred point itself if it is safe.
        Remaining candidates are sampled in expanding circular rings.
        """
        candidates = []
        seen = set()

        def add_candidate(x, y):
            key = (round(x, 3), round(y, 3))

            if key in seen:
                return

            seen.add(key)

            if not self.position_safe(robot, x, y):
                return

            candidates.append((x, y))

        # Always test the preferred location first.
        add_candidate(preferred_x, preferred_y)

        radius = CANDIDATE_STEP

        while radius <= MAX_SEARCH_RADIUS + 1e-9:

            # Keep arc spacing near CANDIDATE_STEP.
            number_of_angles = max(
                8,
                int(math.ceil(
                    (2.0 * math.pi * radius) / CANDIDATE_STEP
                ))
            )

            for i in range(number_of_angles):
                angle = (
                    2.0 * math.pi * i / number_of_angles
                )

                x = preferred_x + radius * math.cos(angle)
                y = preferred_y + radius * math.sin(angle)

                add_candidate(x, y)

            radius += CANDIDATE_STEP

        # Sort by actual Euclidean distance to the preferred point.
        candidates.sort(
            key=lambda p: math.hypot(
                p[0] - preferred_x,
                p[1] - preferred_y
            )
        )

        return candidates

    def workspace_goal_callback(self, msg):

        if any(self.costmaps[r] is None for r in PREFERRED_OFFSETS):
            self.get_logger().warning(
                'No costmap received yet. Try again in a moment.'
            )
            return

        if self.busy:
            self.get_logger().warning(
                'A V3 allocation check is already running.'
            )
            return

        self.busy = True

        self.hilti_goal = msg
        self.robot_index = 0
        self.results = {}
        self.accepted_positions = {}

        print()
        print('====================================================')
        print('   ADAPTIVE WORKSPACE ALLOCATION V3 - DRY RUN')
        print('====================================================')
        print(
            f'Hilti workspace center: '
            f'({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})'
        )
        print()
        print(
            'Searching for SAFE + REACHABLE + NON-OVERLAPPING '
            'support targets...'
        )
        print()

        # Process support robots sequentially. This lets every later
        # robot avoid the targets already reserved by earlier robots.
        self.process_next_robot()

    def process_next_robot(self):

        if self.robot_index >= len(self.robot_order):
            self.finish_workspace_check()
            return

        robot = self.robot_order[self.robot_index]
        dx, dy = PREFERRED_OFFSETS[robot]

        preferred_x = self.hilti_goal.pose.position.x + dx
        preferred_y = self.hilti_goal.pose.position.y + dy

        self.current_robot = robot
        self.current_preferred = (preferred_x, preferred_y)

        all_candidates = self.generate_candidates(
            robot,
            preferred_x,
            preferred_y
        )

        # Remove candidate positions that would overlap targets already
        # accepted for previous support robots.
        self.current_candidates = [
            p for p in all_candidates
            if self.separated_from_accepted(p[0], p[1])
        ]

        self.current_candidate_index = 0

        print(f'{robot}:')
        print(
            f'  Preferred: ({preferred_x:.2f}, {preferred_y:.2f})'
        )
        print(
            f'  Safe/non-overlapping candidates available: '
            f'{len(self.current_candidates)}'
        )

        if not self.current_candidates:
            print(
                f'  Result: NO SAFE NON-OVERLAPPING CANDIDATE '
                f'within {MAX_SEARCH_RADIUS:.1f} m'
            )
            print()

            self.results[robot] = {
                'success': False,
                'reason': 'no_safe_candidate'
            }

            self.robot_index += 1
            self.process_next_robot()
            return

        self.try_current_candidate()

    def try_current_candidate(self):

        robot = self.current_robot

        if self.current_candidate_index >= len(
            self.current_candidates
        ):
            print(
                f'  Result: no candidate was confirmed reachable '
                f'by {robot} Nav2'
            )
            print()

            self.results[robot] = {
                'success': False,
                'reason': 'no_reachable_candidate'
            }

            self.robot_index += 1
            self.process_next_robot()
            return

        target_x, target_y = self.current_candidates[
            self.current_candidate_index
        ]

        client = self.planner_clients[robot]

        if not client.server_is_ready():
            print(
                f'  Result: planner server '
                f'/{robot}/compute_path_to_pose is not ready'
            )
            print()

            self.results[robot] = {
                'success': False,
                'reason': 'planner_not_ready'
            }

            self.robot_index += 1
            self.process_next_robot()
            return

        goal_msg = ComputePathToPose.Goal()

        goal_msg.goal.header.frame_id = 'map'
        goal_msg.goal.header.stamp = (
            self.get_clock().now().to_msg()
        )

        goal_msg.goal.pose.position.x = target_x
        goal_msg.goal.pose.position.y = target_y
        goal_msg.goal.pose.position.z = 0.0

        goal_msg.goal.pose.orientation = (
            self.hilti_goal.pose.orientation
        )

        goal_msg.planner_id = ''
        goal_msg.use_start = False

        future = client.send_goal_async(goal_msg)
        future.add_done_callback(
            self.path_goal_response_callback
        )

    def path_goal_response_callback(self, future):

        robot = self.current_robot

        try:
            goal_handle = future.result()
        except Exception as exc:
            print(
                f'  Candidate planning request error: {exc}'
            )
            self.reject_candidate_and_continue()
            return

        if goal_handle is None or not goal_handle.accepted:
            self.reject_candidate_and_continue()
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            self.path_result_callback
        )

    def path_result_callback(self, future):

        robot = self.current_robot

        try:
            wrapped_result = future.result()
            status = wrapped_result.status
            result = wrapped_result.result
        except Exception as exc:
            print(
                f'  Candidate planning result error: {exc}'
            )
            self.reject_candidate_and_continue()
            return

        target_x, target_y = self.current_candidates[
            self.current_candidate_index
        ]

        preferred_x, preferred_y = self.current_preferred

        if (
            status == GoalStatus.STATUS_SUCCEEDED
            and len(result.path.poses) > 0
        ):
            shift = math.hypot(
                target_x - preferred_x,
                target_y - preferred_y
            )

            self.accepted_positions[robot] = (
                target_x,
                target_y
            )

            self.results[robot] = {
                'success': True,
                'selected': (target_x, target_y),
                'shift': shift,
                'path_points': len(result.path.poses),
                'candidate_number':
                    self.current_candidate_index + 1
            }

            print(
                f'  Selected:  ({target_x:.2f}, {target_y:.2f})'
            )

            if shift < 0.01:
                print(
                    '  Adaptation: preferred position retained'
                )
            else:
                print(
                    f'  Adaptation: moved {shift:.2f} m '
                    f'from preferred'
                )

            print(
                f'  Reachability: CONFIRMED '
                f'({len(result.path.poses)} path poses)'
            )

            print(
                f'  Candidate attempts: '
                f'{self.current_candidate_index + 1}'
            )
            print()

            self.robot_index += 1
            self.process_next_robot()
            return

        self.reject_candidate_and_continue()

    def reject_candidate_and_continue(self):

        target_x, target_y = self.current_candidates[
            self.current_candidate_index
        ]

        attempt = self.current_candidate_index + 1

        # Print the first few rejected candidates for diagnostics,
        # then stay quiet to avoid flooding the terminal.
        if attempt <= 5:
            print(
                f'  Candidate {attempt} '
                f'({target_x:.2f}, {target_y:.2f}) '
                f'not reachable -> trying next'
            )
        elif attempt == 6:
            print(
                '  Additional unreachable candidates omitted '
                'from terminal output...'
            )

        self.current_candidate_index += 1
        self.try_current_candidate()

    def finish_workspace_check(self):

        print()
        print(
            '---------------- FINAL V3 DRY-RUN SUMMARY '
            '----------------'
        )

        all_successful = True

        for robot in self.robot_order:
            result = self.results.get(robot)

            if result is None or not result.get(
                'success',
                False
            ):
                all_successful = False
                reason = (
                    result.get('reason', 'unknown')
                    if result is not None
                    else 'no_result'
                )

                print(
                    f'{robot}: FAILED ({reason})'
                )
                continue

            x, y = result['selected']

            print(
                f'{robot}: SAFE + REACHABLE + RESERVED '
                f'at ({x:.2f}, {y:.2f}) | '
                f'shift={result["shift"]:.2f} m | '
                f'path={result["path_points"]} poses'
            )

        print(
            '------------------------------------------------------'
        )

        if all_successful:
            print(
                'RESULT: all four support targets passed '
                'safety, reachability, and separation checks.'
            )
        else:
            print(
                'RESULT: at least one support robot could not '
                'be assigned a valid target.'
            )

        print(
            'DRY RUN ONLY - NO NAVIGATION GOALS WERE EXECUTED.'
        )
        print(
            '======================================================'
        )
        print()

        self.busy = False


def main(args=None):

    rclpy.init(args=args)

    node = AdaptiveWorkspaceAllocatorV3()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
