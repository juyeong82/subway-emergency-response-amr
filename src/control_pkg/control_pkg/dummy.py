import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Bool
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy 

import math

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        
        # --- [데이터 저장 및 상태 관리 변수] ---
        self.patient_target = PoseStamped()  # 수신된 환자의 목표 위치 저장
        # 로봇별 작업 단계(Progress) 문자열 리스트
        self.robotA_progress = ["로봇 대기", "AED 이송", "승객 통제", "도킹 스테이션 복귀"]
        self.robotB_progress = ["로봇 대기", "환자위치로 이동", "구급대원 위치로 이동", "구급대원 대기", "구급대원과 함께 환자위치로 이동", "도킹 스테이션 복귀"]
        
        # 로봇의 현재 단계 및 이전 단계를 비교하기 위한 변수
        self.robotA_raw_status = 0 # 0~3
        self.robotB_raw_status = 0 # 0~5
        self.prev_robotA_status = -1
        self.prev_robotB_status = -1

        # 시스템 제어 플래그 (AED 수령, 역할 결정, 정지 상태 등)
        self.is_aed_taken = False
        self.is_3A_5B = True     # 로봇 역할 할당 기준 (True: 3번이 A, 5번이 B)
        self.stop = False        # 시스템 전체 비상 정지 여부
        self.is_emt_found = False # 구급대원 발견 여부
        
        # 중복 실행 및 안정화를 위한 제어 플래그
        self.emt_signal_sent = False # 구급대원 신호의 1회성 전송 보장
        self.stop_published = False  # 정지 메시지의 중복 발행 방지
        self.publish_count = 0       # 목표 위치 할당 시 발행 횟수 제어
        self.max_publish = 1         # 최대 발행 횟수 설정
        self.is_initialized = False  # 초기화 타이머 완료 여부

        # 로봇 3과 5의 맵 상의 고정 시작 위치 (거리 계산용)
        self.robot3_current_pose = PoseStamped()
        self.robot3_current_pose.header.frame_id = "map"
        self.robot3_current_pose.pose.position.x = 0.05067639212808157
        self.robot3_current_pose.pose.position.y = 0.09044939290564095

        self.robot5_current_pose = PoseStamped()
        self.robot5_current_pose.header.frame_id = "map"
        self.robot5_current_pose.pose.position.x = 1.2030118956795908
        self.robot5_current_pose.pose.position.y = -1.909306122796912

        ######################### Publisher #########################
        self.patient_pose_pub = self.create_publisher(PoseStamped, '/patient_pose', 10)
        self.robot_role_pub = self.create_publisher(Bool, '/robot_role', 10)
        self.robotA_progress_pub = self.create_publisher(String, '/robotA/task_progress', 10)
        self.robotB_progress_pub = self.create_publisher(String, '/robotB/task_progress', 10)
        self.robot_stop_pub = self.create_publisher(Bool, '/robot_stop', 10)
        self.emt_found_pub = self.create_publisher(Bool, '/robot_emt_found', 10)
        self.aed_state_pub = self.create_publisher(Bool, '/aed_complete_found', 10)
        
        ######################### Subscriber #########################
        self.robotA_sub = self.create_subscription(Int32, '/robotA/progress', self.robotA_callback, 10)
        self.robotB_sub = self.create_subscription(Int32, '/robotB/progress', self.robotB_callback, 10)
        self.target_sub = self.create_subscription(PoseStamped, '/target', self.target_callback, 10)
        self.aed_state_sub = self.create_subscription(Bool, '/aed_state', self.aed_state_callback, 10)
        self.emt_found_sub = self.create_subscription(Bool, '/emt_found', self.emt_found_callback, 10)
        self.stop_sub = self.create_subscription(Bool, '/stop', self.stop_callback, 10)
                            
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.init_timer = self.create_timer(1.0, self.initial_publish_callback)

        self.get_logger().info('Control node start.')

    # 노드 실행 1초 후 초기 상태(대기)를 딱 한 번 알리는 콜백
    def initial_publish_callback(self):
        self.get_logger().info("Sending Initial Status Messages...")
        
        msg_a = String()
        msg_a.data = self.robotA_progress[0]
        self.robotA_progress_pub.publish(msg_a)
        
        msg_b = String()
        msg_b.data = self.robotB_progress[0]
        self.robotB_progress_pub.publish(msg_b)
        
        # 초기화 상태를 동기화하여 중복 발행 방지 및 시스템 시작 승인
        self.prev_robotA_status = self.robotA_raw_status
        self.prev_robotB_status = self.robotB_raw_status
        self.is_initialized = True
        self.init_timer.cancel() # 타이머 파괴 (1회성 실행)
        self.get_logger().info("Initial Messages Sent. System Initialized.")

    # 로봇 A, B의 정수형 진행 단계를 업데이트하는 콜백
    def robotA_callback(self, msg):
        self.robotA_raw_status = msg.data 

    def robotB_callback(self, msg):
        self.robotB_raw_status = msg.data 

    # 목표 위치 수신 시 로직 실행 조건 설정
    def target_callback(self, msg):
        # 로봇이 모두 대기 중일 때만 새로운 작업을 수락함
        if self.robotA_raw_status == 0 and self.robotB_raw_status == 0:
            self.patient_target = msg
            self.publish_count = self.max_publish
            self.get_logger().info('Target Position Received (Status: IDLE). Starting Assignment.')
        else:
            self.get_logger().warn('Target Ignored: Robots are currently BUSY.')

    # AED 상태 수신 콜백
    def aed_state_callback(self, msg):
        self.is_aed_taken = msg.data

    # 구급대원 발견 시 로봇 B에게 알림 제어 콜백
    def emt_found_callback(self, msg):
        self.is_emt_found = msg.data
        self.emt_signal_sent = False # 새 신호 수신 시 전송 플래그 초기화

        # 로봇 B가 구급대원을 기다리는 단계(3)인 경우 즉시 신호 전달
        if self.is_emt_found and self.robotB_raw_status == 3:
            pub_msg = Bool()
            pub_msg.data = True
            self.emt_found_pub.publish(pub_msg)
            self.emt_signal_sent = True
            self.get_logger().info(">> EMT Found! Robot B is waiting(3). Sending signal to Robot B.")
        elif self.is_emt_found:
            self.get_logger().warn(f">> EMT Found signal received, but Robot B status is {self.robotB_raw_status}. Buffered for later.")

    # 비상 정지 명령 수신 콜백
    def stop_callback(self, msg):
        self.stop = msg.data
        if self.stop:
            self.get_logger().warn("!!! EMERGENCY STOP SIGNAL RECEIVED !!!")
        else:
            self.stop_published = False

    # 메인 로직 타이머 (역할 배정 및 상태 모니터링)
    def timer_callback(self):
        # 비상 정지 활성화 시 처리 로직
        if self.stop:
            if not self.stop_published:
                # 정지 토픽 발행 및 각 로봇에게 복귀 메시지 1회 발행
                stop_msg = Bool()
                stop_msg.data = True
                self.robot_stop_pub.publish(stop_msg)
                
                return_msg = String()
                return_msg.data = "작업 중단 후 도킹 스테이션으로 복귀 중"
                self.robotA_progress_pub.publish(return_msg)
                self.robotB_progress_pub.publish(return_msg)
                
                self.stop_published = True 
                self.get_logger().info("Emergency Stop & Return Message Published.")
            
            self.publish_count = 0

            # 정지 후 로봇들이 모두 복귀하여 대기 상태(0)가 되면 정지 상태 자동 해제
            if self.robotA_raw_status == 0 and self.robotB_raw_status == 0:
                self.stop = False
                self.stop_published = False 
                
                # 강제로 "대기" 메시지 재발행하여 화면 갱신
                msg_a = String()
                msg_a.data = self.robotA_progress[0]
                self.robotA_progress_pub.publish(msg_a)
                
                msg_b = String()
                msg_b.data = self.robotB_progress[0]
                self.robotB_progress_pub.publish(msg_b)
                
                self.prev_robotA_status = 0
                self.prev_robotB_status = 0
                self.is_emt_found = False
                self.emt_signal_sent = False
                self.get_logger().info("Both robots returned to IDLE (0). Stop Released & Status Reset.")
            
        else:
            # 정상 주행 시 로봇 역할 배정 (거리 계산)
            if self.publish_count > 0:
                # 작업 할당 중 로봇 상태가 대기 중(0)인 경우에만 거리 계산 실행
                if self.robotA_raw_status == 0 and self.robotB_raw_status == 0:
                    if self.patient_target.header.frame_id != "":
                        # math.hypot을 이용해 환자 위치와 각 로봇 시작 위치 간의 직선거리 계산
                        dist3 = math.hypot(
                            self.patient_target.pose.position.x - self.robot3_current_pose.pose.position.x,
                            self.patient_target.pose.position.y - self.robot3_current_pose.pose.position.y
                        )
                        dist5 = math.hypot(
                            self.patient_target.pose.position.x - self.robot5_current_pose.pose.position.x,
                            self.patient_target.pose.position.y - self.robot5_current_pose.pose.position.y
                        )

                        # 가까운 로봇을 기준으로 역할(is_3A_5B) 결정 및 타겟 정보 전송
                        self.is_3A_5B = (dist3 <= dist5)
                        self.get_logger().info(f'[Count: {self.publish_count}] R3 Dist: {dist3:.2f}m, R5 Dist: {dist5:.2f}m, 3A_5B: {self.is_3A_5B}')

                        role_msg = Bool()
                        role_msg.data = self.is_3A_5B
                        self.robot_role_pub.publish(role_msg)
                        self.patient_pose_pub.publish(self.patient_target)

                        self.publish_count -= 1

        # 로봇별 진행 상황 모니터링 및 문자열 메시지 발행
        if self.is_initialized and not self.stop:
            try:
                # 로봇 A의 정수형 상태가 변했을 때만 해당하는 문자열 작업 단계 발행
                if self.robotA_raw_status != self.prev_robotA_status:
                    prog_msg_a = String()
                    prog_msg_a.data = self.robotA_progress[self.robotA_raw_status]
                    self.robotA_progress_pub.publish(prog_msg_a)
                    self.prev_robotA_status = self.robotA_raw_status

                # 로봇 B의 진행 상황 처리 및 구급대원 발견 신호 지연 전송 확인
                if self.robotB_raw_status != self.prev_robotB_status:
                    prog_msg_b = String()
                    prog_msg_b.data = self.robotB_progress[self.robotB_raw_status]
                    self.robotB_progress_pub.publish(prog_msg_b)
                    
                    # 로봇 B가 대기 단계(3)에 진입했을 때, 이미 발견된 구급대원이 있다면 즉시 신호 발행
                    if self.robotB_raw_status == 3 and self.is_emt_found and not self.emt_signal_sent:
                         pub_msg = Bool()
                         pub_msg.data = True
                         self.emt_found_pub.publish(pub_msg)
                         self.emt_signal_sent = True
                         self.get_logger().info(">> Robot B reached step 3. Sending buffered EMT signal.")
                    
                    self.prev_robotB_status = self.robotB_raw_status

            except IndexError:
                self.get_logger().error('Status Index Error: Received status code is out of range.')

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()