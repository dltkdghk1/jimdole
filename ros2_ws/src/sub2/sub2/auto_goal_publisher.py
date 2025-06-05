# auto_goal
#!/usr/bin/env python3
import rclpy
import json
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

class AutoGoal(Node):
    def __init__(self):
        super().__init__('auto_goal')
        # TRANSIENT_LOCAL QoS를 사용하여, 늦게 구독하는 노드도 마지막 메시지를 받을 수 있도록 합니다.
        qos_profile = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', qos_profile)
        # 외부 통신 메시지가 JSON 문자열 형태로 들어오는 토픽을 구독합니다.
        # (예: MQTT에서 '/robot/gate' 토픽으로 들어온 메시지를 사용)
        self.create_subscription(String, 'initial_goal', self.json_callback, 10)
        # 타이머 콜백을 통해 주기적으로 goal_pose를 publish합니다.
        self.create_timer(1.0, self.publish_goal)

        # pending_goal은 통신으로 받은 목표(PoseStamped)를 저장하는 변수입니다.
        # 통신 메시지가 들어오기 전까지는 None 상태로 두어 아무것도 발행하지 않습니다.
        self.pending_goal = None

    def json_callback(self, msg: String):
        """
        JSON 메시지 예시:
          {"reservation_id": "ABC123", "event_type": "MOVED", "robot_id": 1, "x": -61.974, "y": -53.235, "gate": "A"}
        """
        try:
            data = json.loads(msg.data)
            x = data.get("x")
            y = data.get("y")
            if x is None or y is None:
                self.get_logger().warn("[json_callback] JSON missing 'x' or 'y'.")
                return

            # PoseStamped 메시지 생성 (통신으로 받은 좌표를 목표로 사용)
            ps = PoseStamped()
            ps.header.frame_id = data.get("frame", "map")  # 기본적으로 'map' 프레임을 사용
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.position.z = 0.0
            ps.pose.orientation.w = 1.0
            ps.pose.orientation.x = 0.0
            ps.pose.orientation.y = 0.0
            ps.pose.orientation.z = 0.0

            # 새 좌표를 수신하면 이를 pending_goal에 저장합니다.
            self.pending_goal = ps
            self.get_logger().info(f"[json_callback] New goal received from comm: x={x}, y={y}")
        except Exception as e:
            self.get_logger().error(f"[json_callback] JSON parse error: {e}")

    def publish_goal(self):
        # 만약 통신으로부터 목표를 수신한 경우에만 goal_pose를 publish합니다.
        if self.pending_goal is not None:
            goal_msg = PoseStamped()
            goal_msg.header.frame_id = self.pending_goal.header.frame_id
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose = self.pending_goal.pose
            self.goal_pub.publish(goal_msg)
            self.get_logger().info(
                f"[publish_goal] Publishing goal from comm: x={goal_msg.pose.position.x}, y={goal_msg.pose.position.y}"
            )
        else:
            self.get_logger().info("[publish_goal] No goal received yet. Waiting for communication...")

def main(args=None):
    rclpy.init(args=args)
    node = AutoGoal()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
