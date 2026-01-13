import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Bool
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy

class AutomatedTestNode(Node):
    def __init__(self):
        super().__init__('automated_test_node')
        
        # ControlNode의 구독 설정에 맞춘 Best Effort QoS
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)

        ######################### Publisher #########################
        self.target_pub = self.create_publisher(PoseStamped, '/target', qos_profile)
        self.robotA_status_pub = self.create_publisher(Int32, '/robotA/progress', qos_profile)
        self.robotB_status_pub = self.create_publisher(Int32, '/robotB/progress', qos_profile)

        ######################### Subscriber #########################
        self.role_sub = self.create_subscription(Bool, '/robot_role', self.role_callback, 10)
        self.task_a_sub = self.create_subscription(String, '/robotA/task_progress', self.task_a_callback, 10)
        self.task_b_sub = self.create_subscription(String, '/robotB/task_progress', self.task_b_callback, 10)

        # 모니터링 변수
        self.mon_role = "배정 대기 중"
        self.mon_task_a = "None"
        self.mon_task_b = "None"
        
        # 시나리오 제어 변수
        self.step = 0
        self.timer_count = 0
        self.step_duration = 30  # 3초 (0.1초 * 30틱)
        
        # 0.1초 주기로 타이머 실행 (통신 유지 및 상태 관리)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info('=== 3초 간격 자동화 테스트 노드 시작 ===')

    def role_callback(self, msg):
        self.mon_role = "3=A, 5=B (True)" if msg.data else "3=B, 5=A (False)"

    def task_a_callback(self, msg):
        self.mon_task_a = msg.data

    def task_b_callback(self, msg):
        self.mon_task_b = msg.data

    def timer_callback(self):
        self.timer_count += 1
        
        # 3초마다 단계(Step) 증가
        if self.timer_count >= self.step_duration:
            self.step += 1
            self.timer_count = 0
            if self.step > 6: # 4단계 완료 후 다시 처음으로 루프
                self.step = 0
            self.get_logger().info(f'--- Step {self.step} 전환 (3초 대기 시작) ---')

        # 현재 단계에 따른 데이터 결정
        a_status = 0
        b_status = 0

        if self.step == 0:
            # [단계 0] 모든 로봇 0으로 초기화 (역할 배정 조건 충족)
            a_status, b_status = 0, 0
            
        elif self.step == 1:
            # [단계 1] 타겟 위치 전송 (Robot 3과 가까운 곳)
            self.publish_target(0.1, 0.1)
            a_status, b_status = 0, 0
            
        elif self.step >= 2 and self.step <= 5:
            # [단계 2~5] 진행 상황 단계별 상승 (1, 2, 3, 4)
            # 역할 배정 결과에 따라 로봇 A가 되었을 로봇의 상태를 변경
            current_progress = self.step - 1
            a_status = current_progress
            b_status = 0
            
        elif self.step == 6:
            # [단계 6] 모든 공정 완료 및 초기화 대기
            a_status, b_status = 4, 0

        # 데이터 상시 발행 (0.1초마다)
        self.publish_status(a_status, b_status)

        # 모니터링 로그 출력 (1초마다)
        if self.timer_count % 10 == 0:
            print(f"[{self.step}단계] 모니터링 -> 역할: {self.mon_role} | 로봇A 진행: {self.mon_task_a} | 로봇B 진행: {self.mon_task_b}")

    def publish_target(self, x, y):
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.pose.position.x = x
        msg.pose.position.y = y
        self.target_pub.publish(msg)

    def publish_status(self, a_val, b_val):
        msg_a = Int32()
        msg_a.data = a_val
        self.robotA_status_pub.publish(msg_a)
        
        msg_b = Int32()
        msg_b.data = b_val
        self.robotB_status_pub.publish(msg_b)

def main(args=None):
    rclpy.init(args=args)
    node = AutomatedTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()