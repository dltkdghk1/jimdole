import rclpy
from rclpy.node import Node
from ssafy_msgs.msg import TurtlebotStatus, HandControl
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
import math

class Handcontrol(Node):
    def __init__(self):
        super().__init__('hand_control')
                
        # Publisher for HandControl messages
        self.hand_control = self.create_publisher(HandControl, '/hand_control', 10)
        
        # Subscribers for TurtlebotStatus, goal_pose, odom, and local_path
        self.turtlebot_status = self.create_subscription(
            TurtlebotStatus, '/turtlebot_status', self.turtlebot_status_cb, 10)
        self.goal_pose_sub = self.create_subscription(
            PoseStamped, 'goal_pose', self.goal_pose_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.path_sub = self.create_subscription(Path, '/local_path', self.path_callback, 10)
        
        # Timer for periodic processing (1초 주기)
        self.timer = self.create_timer(1, self.timer_callback)
        
        # Message and state variables
        self.hand_control_msg = HandControl()        
        self.turtlebot_status_msg = TurtlebotStatus()
        self.is_turtlebot_status = False

        # goal_pose 기반 도착 판단 변수
        self.goal_pose = None         # 최신 goal_pose (PoseStamped)
        self.last_goal_pose = None    # 이전 goal_pose (비교용)
        self.goal_stable_counter = 0  # 목표 좌표가 변화 없이 안정된 횟수
        self.GOAL_STABLE_THRESHOLD = 5  # (참고용: 여기서는 사용하지 않습니다)

        # odom, local_path 관련 (목적지 도착 판단)
        self.odom_msg = None
        self.path_msg = None
        self.is_odom = False
        self.is_path = False
        self.dest_threshold = 0.2  # 목적지 도달 임계 거리 (m)

        # 마지막으로 발행한 control_mode (None이면 미발행)
        self.last_mode = None

    def turtlebot_status_cb(self, msg):
        self.is_turtlebot_status = True
        self.turtlebot_status_msg = msg

    def goal_pose_cb(self, msg: PoseStamped):
        self.goal_pose = msg

    def odom_callback(self, msg: Odometry):
        self.odom_msg = msg
        self.is_odom = True

    def path_callback(self, msg: Path):
        self.path_msg = msg
        self.is_path = True

    def hand_control_preview(self):
        # Preview mode (1번)
        self.hand_control_msg.control_mode = 1
        self.hand_control_msg.put_distance = 2.0
        self.hand_control_msg.put_height = 0.0

    def hand_control_pick_up(self):
        # Pick up mode (2번)
        self.hand_control_msg.control_mode = 2
        self.hand_control_msg.put_distance = 2.0
        self.hand_control_msg.put_height = 2.0

    def hand_control_put_down(self):
        # Put down mode (3번)
        self.hand_control_msg.control_mode = 3
        self.hand_control_msg.put_distance = 2.0
        self.hand_control_msg.put_height = 0.0

    def timer_callback(self):
        # 로그 출력
        print("can_lift = {}".format(self.turtlebot_status_msg.can_lift))
        print("can_put = {}".format(self.turtlebot_status_msg.can_put))
        print("can_use_hand = {}".format(self.turtlebot_status_msg.can_use_hand))
        if self.goal_pose is not None:
            print("Goal pose: ({:.2f}, {:.2f})".format(
                self.goal_pose.pose.position.x, self.goal_pose.pose.position.y))
        else:
            print("No goal pose received")
        
        # 물건 보유 여부 판단: 물건이 잡혔다면 can_lift는 False가 되어야 함.
        # 따라서 object_held는 not can_lift로 계산합니다.
        object_held = not self.turtlebot_status_msg.can_lift
        print("object_held = {}".format(object_held))
        
        # 목적지 도착 판단 (odom과 local_path 이용)
        destination_reached = False
        if self.is_odom and self.is_path and self.path_msg is not None and len(self.path_msg.poses) > 0:
            dest_pose = self.path_msg.poses[-1].pose
            rx = self.odom_msg.pose.pose.position.x
            ry = self.odom_msg.pose.pose.position.y
            dest_distance = math.sqrt((rx - dest_pose.position.x)**2 + (ry - dest_pose.position.y)**2)
            self.get_logger().info("Destination distance: {:.2f}".format(dest_distance))
            if dest_distance < self.dest_threshold:
                destination_reached = True
        
        new_mode = None  # 새로 발행할 명령

        # [목적지 도착 시]
        if destination_reached:
            if object_held:
                # 물건 보유 상태이면 drop-off 시도
                if self.turtlebot_status_msg.can_put:
                    new_mode = 3  # can_put이 True이면 put_down 명령
                    self.get_logger().info("At destination: can_put True, issuing put_down command")
                else:
                    new_mode = 1  # can_put이 False이면 preview 명령
                    print("At destination: can_put False, previewing drop-off")
            else:
                print("At destination but no object held. No drop-off command issued.")
        else:
            # [목적지 미도착 시: pick-up 처리]
            if not object_held:
                # 물건을 잡지 않은 상태이면, can_lift가 True일 때 pick-up 명령을 발행
                if self.turtlebot_status_msg.can_lift:
                    new_mode = 2  # pick-up 명령
                    print("Not at destination: can_lift True, issuing pick-up command")
                else:
                    new_mode = 1  # can_lift가 False이면 preview 명령 (이미 집은 상태라면)
                    print("Not at destination: can_lift False, previewing")
            else:
                print("Not at destination: object held, no pick-up command issued")
        
        # 명령 발행: 새 명령이 결정된 경우에만 발행
        if new_mode is not None:
            # 매번 새 명령을 발행하도록 하며, 이전 모드와 비교하지 않습니다.
            if new_mode == 1:
                self.hand_control_preview()
            elif new_mode == 2:
                self.hand_control_pick_up()
            elif new_mode == 3:
                self.hand_control_put_down()
            self.hand_control.publish(self.hand_control_msg)
            self.last_mode = new_mode

def main(args=None):
    rclpy.init(args=args)
    hand_control_node = Handcontrol()    
    rclpy.spin(hand_control_node)
    hand_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
