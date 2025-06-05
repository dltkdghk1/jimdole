# a_star
#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from math import sqrt
from collections import deque
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

class AStarNode(Node):
    def __init__(self):
        super().__init__('a_star')
        # Subscribers for map, odom, goal
        self.map_sub = self.create_subscription(OccupancyGrid, 'map', self.map_callback, 1)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 1)
        qos_profile = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.goal_sub = self.create_subscription(PoseStamped, 'goal_pose', self.goal_callback, qos_profile)
        # Publisher for global path
        self.a_star_pub = self.create_publisher(Path, 'global_path', 1)

        # 상태 변수
        self.map_msg = None
        self.odom_msg = None
        self.is_map = False
        self.is_odom = False
        self.is_grid_update = False

        # 통신으로 받은 목표 값; 이 값이 없으면 경로 계획을 시도하지 않음.
        self.pending_goal = None

        # 맵 파라미터 (load_map과 일치)
        self.map_size_x = 600
        self.map_size_y = 600
        self.map_resolution = 0.05
        self.map_offset_x = -65.0
        self.map_offset_y = -65.0
        self.GRIDSIZE = 600
        self.grid = None

        # 8방향 (dx, dy, 이동 비용)
        self.dx = [-1, 0, 0, 1, -1, -1, 1, 1]
        self.dy = [0, 1, -1, 0, -1, 1, -1, 1]
        self.dCost = [1, 1, 1, 1, 1.414, 1.414, 1.414, 1.414]

        # 타이머: 0.5초 주기로 pending_goal 존재 여부를 확인하고 처리
        self.timer = self.create_timer(0.5, self.check_pending_goal)

    def map_callback(self, msg):
        self.map_msg = msg
        self.is_map = True

    def odom_callback(self, msg):
        self.odom_msg = msg
        self.is_odom = True

    def goal_callback(self, msg):
        if msg.header.frame_id != 'map':
            self.get_logger().warn(f"[goal_callback] Received goal in frame '{msg.header.frame_id}', expected 'map'.")
            return
        # 외부 통신으로 goal_pose가 들어오면 pending_goal에 저장합니다.
        self.pending_goal = msg
        self.get_logger().info(f"[goal_callback] Stored pending goal: x={msg.pose.position.x}, y={msg.pose.position.y}")

    def check_pending_goal(self):
        # map과 odom 데이터가 준비되었는지 확인
        if not (self.is_map and self.is_odom):
            self.get_logger().info("[check_pending_goal] Waiting for map/odom data...")
            return
        # pending_goal이 없으면 경로 계획을 시도하지 않음.
        if self.pending_goal is None:
            self.get_logger().info("[check_pending_goal] No pending goal received yet; waiting for communication...")
            return

        # pending_goal이 있을 때만 process_goal를 호출합니다.
        success = self.process_goal(self.pending_goal)
        # 경로 계획이 성공하면 pending_goal을 삭제합니다.
        if success:
            self.pending_goal = None

    def process_goal(self, msg: PoseStamped):
        # 필요한 경우 grid를 업데이트
        if not self.is_grid_update:
            self.grid_update()

        sx = self.odom_msg.pose.pose.position.x
        sy = self.odom_msg.pose.pose.position.y
        gx = msg.pose.position.x
        gy = msg.pose.position.y

        start_cell = self.pose_to_grid_cell(sx, sy)
        goal_cell = self.pose_to_grid_cell(gx, gy)

        # 범위 및 장애물 검사
        if not self.is_valid_cell(start_cell):
            self.get_logger().warn(f"[process_goal] Start cell {start_cell} is out of range.")
            return False
        if not self.is_valid_cell(goal_cell):
            self.get_logger().warn(f"[process_goal] Goal cell {goal_cell} is out of range.")
            return False
        if self.grid[start_cell[1]][start_cell[0]] != 0:
            self.get_logger().warn(f"[process_goal] Start cell {start_cell} not free (val={self.grid[start_cell[1]][start_cell[0]]}).")
            return False
        if self.grid[goal_cell[1]][goal_cell[0]] != 0:
            self.get_logger().warn(f"[process_goal] Goal cell {goal_cell} not free (val={self.grid[goal_cell[1]][goal_cell[0]]}).")
            return False
        if start_cell == goal_cell:
            self.get_logger().info("[process_goal] Start equals goal. Nothing to plan.")
            return False

        self.get_logger().info(f"[process_goal] Planning path: start={start_cell}, goal={goal_cell}")

        self.final_path = []
        self.path = [[0 for _ in range(self.GRIDSIZE)] for _ in range(self.GRIDSIZE)]
        self.cost = np.full((self.GRIDSIZE, self.GRIDSIZE), self.GRIDSIZE * self.GRIDSIZE, dtype=float)
        self.dijkstra(start_cell, goal_cell)

        if len(self.final_path) == 0:
            self.get_logger().warn("[process_goal] No path found. Will retry on next timer cycle.")
            return False

        # 글로벌 경로 메시지 생성
        global_path_msg = Path()
        global_path_msg.header.frame_id = 'map'
        for cell in reversed(self.final_path):
            tmp_pose = PoseStamped()
            wx, wy = self.grid_cell_to_pose(cell)
            tmp_pose.pose.position.x = wx
            tmp_pose.pose.position.y = wy
            tmp_pose.pose.orientation.w = 1.0
            global_path_msg.poses.append(tmp_pose)

        self.a_star_pub.publish(global_path_msg)
        self.get_logger().info(f"[process_goal] Path published. Path length = {len(self.final_path)}")
        return True

    def grid_update(self):
        if self.map_msg is None:
            return
        map_array = np.array(self.map_msg.data, dtype=int)
        self.grid = map_array.reshape(self.map_size_y, self.map_size_x)
        self.is_grid_update = True
        self.get_logger().info(f"[grid_update] Grid updated with shape: {self.grid.shape}")

    def pose_to_grid_cell(self, x, y):
        mx = int((x - self.map_offset_x) / self.map_resolution)
        my = int((y - self.map_offset_y) / self.map_resolution)
        return (mx, my)

    def grid_cell_to_pose(self, cell):
        x = cell[0] * self.map_resolution + self.map_offset_x + (self.map_resolution / 2.0)
        y = cell[1] * self.map_resolution + self.map_offset_y + (self.map_resolution / 2.0)
        return x, y

    def is_valid_cell(self, cell):
        x, y = cell
        return 0 <= x < self.map_size_x and 0 <= y < self.map_size_y

    def dijkstra(self, start, goal):
        queue = deque()
        queue.append(start)
        self.cost[start[1]][start[0]] = 1  # cost 배열은 [y][x] 순서로 접근
        found = False

        while queue:
            current = queue.popleft()
            if current == goal:
                found = True
                self.get_logger().info("[dijkstra] Goal reached!")
                break
            for i in range(8):
                nx = current[0] + self.dx[i]
                ny = current[1] + self.dy[i]
                if (0 <= nx < self.map_size_x) and (0 <= ny < self.map_size_y):
                    if self.grid[ny][nx] < 50:  # free cell 검사 (50 미만이면 자유)
                        new_cost = self.cost[current[1]][current[0]] + self.dCost[i]
                        if self.cost[ny][nx] > new_cost:
                            self.cost[ny][nx] = new_cost
                            self.path[ny][nx] = current
                            queue.append((nx, ny))
        if found:
            node = goal
            while node != start:
                self.final_path.append(node)
                node = self.path[node[1]][node[0]]
            self.get_logger().info(f"[dijkstra] Backtracking complete. Path length = {len(self.final_path)}")
        else:
            self.get_logger().warn("[dijkstra] Cannot reach goal.")

def main(args=None):
    rclpy.init(args=args)
    node = AStarNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
