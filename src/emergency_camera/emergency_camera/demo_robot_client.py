import sys
import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

# 메시지 타입 임포트
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import Bool

# 터틀봇 네비게이션 라이브러리
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator

# =========================================================
# [설정] 테스트 모드 상수
# =========================================================
MODE_IDLE = 0       # 대기
MODE_PATIENT = 1    # 환자 위치로 이동 (YOLO 감지)
MODE_CLICK = 2      # 클릭 위치로 이동 (웹 UI)
MODE_ARRIVAL = 3    # 도착 알림 수신 (근접 인식)

class DemoScenarioNode(Node):
    def __init__(self):
        super().__init__('demo_scenario_node')

        # --- [TurtleBot4 Navigator 설정] ---
        # robot5 네임스페이스 사용 (사용자 환경 맞춤)
        self.navigator = TurtleBot4Navigator(namespace='/robot3')
        
        # Nav2 활성화 대기
        # (주의: 로봇의 Nav2 스택이 켜져 있어야 함)
        if not self.navigator.getDockedStatus():
            self.get_logger().info('⚠️ 로봇이 도킹되어 있지 않습니다. 초기 위치 설정에 주의하세요.')
        
        # 초기 위치 설정 (필요시 수정)
        # self.navigator.setInitialPose(...) # 이미 맵이 있고 로컬라이제이션이 되어있다면 생략 가능

        # --- [상태 변수] ---
        self.current_mode = MODE_IDLE
        self.is_moving = False

        # --- [Subscribers] ---
        qos = QoSProfile(depth=10)

        # 1. 환자 위치 수신 (Control Tower -> Node)
        self.create_subscription(
            PoseStamped, 
            '/target', 
            self.cb_patient_target, 
            qos
        )

        # 2. 웹 UI 클릭 좌표 수신 (Control Tower -> Node)
        self.create_subscription(
            Point, 
            '/control/goal_point', 
            self.cb_ui_click, 
            qos
        )

        # 3. 도착 완료 신호 수신 (Control Tower -> Node)
        self.create_subscription(
            Bool, 
            '/emt_arrival_status', 
            self.cb_arrival_status, 
            qos
        )

        print("✅ Demo Node Initialized. Waiting for user input...")

    # =====================================================
    # [Callbacks] Control Tower 신호 처리
    # =====================================================
    
    def cb_patient_target(self, msg: PoseStamped):
        # 모드가 '환자 이동'이 아니거나, 이미 이동 중이면 무시
        if self.current_mode != MODE_PATIENT or self.is_moving:
            return

        print(f"\n[EVENT] 환자 감지됨! 좌표: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})")
        print(">>> 환자 위치로 이동 시작...")
        
        self.is_moving = True
        # 이동 명령 (비동기 처리 권장되나 데모용으로 단순 호출)
        self.navigator.startToPose(msg)
        
        # 이동 완료 후 로직 (blocking call이라 완료 후 실행됨)
        print(">>> [완료] 환자 위치 도착.")
        self.is_moving = False
        self.current_mode = MODE_IDLE # 한 번 이동 후 대기 모드로 복귀 (반복 방지)
        print("\n(메인 메뉴로 돌아가려면 엔터를 누르세요)")

    def cb_ui_click(self, msg: Point):
        # 모드가 '클릭 이동'이 아니거나, 이미 이동 중이면 무시
        if self.current_mode != MODE_CLICK or self.is_moving:
            return

        print(f"\n[EVENT] UI 클릭 감지됨! 좌표: ({msg.x:.2f}, {msg.y:.2f})")
        print(">>> 해당 지점으로 이동 시작...")
        
        self.is_moving = True
        
        # Point(x,y) -> PoseStamped 변환
        # 방향(Orientation)은 기본값(North) 또는 마지막 방향 유지
        goal_pose = self.navigator.getPoseStamped([msg.x, msg.y], TurtleBot4Directions.NORTH)
        
        self.navigator.startToPose(goal_pose)
        
        print(">>> [완료] 목표 지점 도착.")
        self.is_moving = False
        self.current_mode = MODE_IDLE
        print("\n(메인 메뉴로 돌아가려면 엔터를 누르세요)")

    def cb_arrival_status(self, msg: Bool):
        # 모드가 '도착 체크'가 아니면 무시
        if self.current_mode != MODE_ARRIVAL:
            return

        if msg.data: # True일 때만
            print("\n🚨 [ALARM] 구조대원-환자 접촉 확인 (Distance Threshold Pass)!")
            print(">>> 시스템: '구조 작업이 시작되었습니다.'")
            # 여기서 로봇에게 소리를 내거나 LED를 켜는 등의 추가 액션 가능
            self.current_mode = MODE_IDLE
            print("\n(메인 메뉴로 돌아가려면 엔터를 누르세요)")


# =========================================================
# [Menu] 사용자 인터페이스 (스레드)
# =========================================================
def run_menu(node: DemoScenarioNode):
    while rclpy.ok():
        print("\n========================================")
        print(f"   Subway Robot 시연 데모 (Namespace: /robot5)")
        print("========================================")
        print(f" 현재 상태: {get_mode_str(node.current_mode)}")
        print("----------------------------------------")
        print(" 1. [준비] Undock (충전 스테이션 분리)")
        print(" 2. [테스트] 환자 감지 시 자동 이동 (Wait for YOLO)")
        print(" 3. [테스트] 관제화면 클릭 시 이동 (Wait for Click)")
        print(" 4. [테스트] 도착 알림 수신 확인 (Wait for Distance)")
        print(" 5. [복귀] Dock (충전 복귀)")
        print(" 6. [종료] 프로그램 종료")
        print("========================================")
        
        try:
            choice = input("선택 >> ")
        except EOFError:
            break

        if choice == '1':
            print(">>> Undocking...")
            node.navigator.undock()
            print(">>> Undock 완료.")
        
        elif choice == '2':
            node.current_mode = MODE_PATIENT
            print("\n>>> [대기 중] YOLO 화면에 'Patient'가 잡히면 이동합니다...")
            print("    (취소하려면 Ctrl+C 후 재시작)")
            while node.current_mode == MODE_PATIENT:
                time.sleep(1) # 콜백이 모드를 바꿀 때까지 대기
            
        elif choice == '3':
            node.current_mode = MODE_CLICK
            print("\n>>> [대기 중] 관제 웹에서 지도를 클릭하세요...")
            while node.current_mode == MODE_CLICK:
                time.sleep(1)

        elif choice == '4':
            node.current_mode = MODE_ARRIVAL
            print("\n>>> [대기 중] 환자와 구조대원이 가까워지기를 기다리는 중...")
            while node.current_mode == MODE_ARRIVAL:
                time.sleep(1)

        elif choice == '5':
            print(">>> Docking...")
            node.navigator.dock()
            print(">>> Dock 완료.")

        elif choice == '6':
            print("종료합니다.")
            rclpy.shutdown()
            sys.exit(0)
        
        else:
            print("잘못된 입력입니다.")

def get_mode_str(mode):
    if mode == MODE_IDLE: return "IDLE (대기)"
    if mode == MODE_PATIENT: return "PATIENT_WAIT (환자 감지 대기중)"
    if mode == MODE_CLICK: return "CLICK_WAIT (클릭 대기중)"
    if mode == MODE_ARRIVAL: return "ARRIVAL_WAIT (도착 신호 대기중)"
    return "UNKNOWN"

# =========================================================
# [Main] 실행부
# =========================================================
def main():
    rclpy.init()
    
    # 노드 생성
    demo_node = DemoScenarioNode()
    
    # ROS2 통신을 위한 스레드 (Spin)
    spin_thread = threading.Thread(target=rclpy.spin, args=(demo_node,), daemon=True)
    spin_thread.start()
    
    # 메뉴 실행 (Main Thread)
    try:
        run_menu(demo_node)
    except KeyboardInterrupt:
        pass
    finally:
        demo_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()