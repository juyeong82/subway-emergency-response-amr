#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import threading

# ==========================================
# 1. 좌표 변환 클래스
# ==========================================
class CoordConverter:
    def __init__(self):
        # 입력: 웹캠 화면 픽셀 좌표 (u, v)
        self.pixel_points = np.array([
            [96, 214],   # 1번
            [297, 81],   # 2번
            [410, 447],  # 3번
            [562, 156]   # 4번
        ], dtype=np.float32)

        # 출력: ROS 맵 좌표 (x, y)
        self.map_points = np.array([
            [-1.76, 0.395],   # 1번
            [-1.46, -1.62],   # 2번
            [-3.21, 0.396],   # 3번
            [-2.94, -1.87]    # 4번
        ], dtype=np.float32)

        # 호모그래피 행렬 계산
        self.H, _ = cv2.findHomography(self.pixel_points, self.map_points)
        print("✅ 좌표 변환 행렬(Homography) 계산 완료")

    def pixel_to_map(self, u, v):
        if self.H is None:
            return None
        pixel_pt = np.array([[[u, v]]], dtype=np.float32)
        map_pt = cv2.perspectiveTransform(pixel_pt, self.H)
        return map_pt[0][0]  # [x, y] 반환

# ==========================================
# 2. Nav2 액션 클라이언트 노드
# ==========================================
class Nav2Controller(Node):
    def __init__(self, namespace='/robot5'):
        super().__init__('nav2_controller')
        
        # NavigateToPose 액션 클라이언트 생성
        action_name = f'{namespace}/navigate_to_pose'
        self._action_client = ActionClient(self, NavigateToPose, action_name)
        
        print(f"✅ Nav2 액션 클라이언트 생성: {action_name}")
        print("⏳ Nav2 액션 서버 연결 대기 중...")
        
        self._action_client.wait_for_server()
        print("✅ Nav2 액션 서버 연결 완료!")
        
        self.is_moving = False
        self._goal_handle = None

    def send_goal(self, x, y):
        """Nav2에 목표 위치 전송"""
        if self.is_moving:
            print("🚫 로봇이 이미 이동 중입니다.")
            return
        
        # 목표 포즈 생성
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0  # 기본 방향
        
        print(f"🚀 목표 전송: x={x:.3f}, y={y:.3f}")
        
        # 액션 전송
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        self.is_moving = True

    def goal_response_callback(self, future):
        """목표 수락 여부 콜백"""
        self._goal_handle = future.result()
        
        if not self._goal_handle.accepted:
            print("❌ 목표가 거부되었습니다.")
            self.is_moving = False
            return
        
        print("✅ 목표가 수락되었습니다. 이동 중...")
        
        # 결과 대기
        self._get_result_future = self._goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """이동 중 피드백 콜백 (선택적)"""
        # feedback = feedback_msg.feedback
        # print(f"📍 현재 거리: {feedback.distance_remaining:.2f}m")
        pass

    def get_result_callback(self, future):
        """최종 결과 콜백"""
        result = future.result().result
        self.is_moving = False
        
        # Nav2 결과 코드 확인
        if result:
            print("🏁 목표 지점에 도착했습니다!")
        else:
            print("❌ 목표 도달 실패")

    def cancel_goal(self):
        """현재 목표 취소"""
        if self._goal_handle:
            print("🛑 목표 취소 중...")
            self._goal_handle.cancel_goal_async()

# ==========================================
# 3. OpenCV 메인 루프
# ==========================================
converter = None
nav2_controller = None
latest_click_data = None  # 화면 표시용 데이터

def mouse_callback(event, x, y, flags, param):
    global latest_click_data, converter, nav2_controller
    
    if event == cv2.EVENT_LBUTTONDOWN:  # 마우스 왼쪽 클릭
        # 픽셀 -> 맵 좌표 변환
        map_pos = converter.pixel_to_map(x, y)
        map_x, map_y = map_pos[0], map_pos[1]
        
        print(f"\n🖱️ 클릭: ({x}, {y}) -> 🗺️ 맵 좌표: ({map_x:.3f}, {map_y:.3f})")
        
        # 화면 표시 데이터 업데이트
        latest_click_data = (x, y, map_x, map_y)
        
        # Nav2로 목표 전송
        if nav2_controller:
            nav2_controller.send_goal(map_x, map_y)

def main():
    global converter, nav2_controller, latest_click_data
    
    # ROS 2 초기화
    rclpy.init()
    
    # Nav2 컨트롤러 생성
    nav2_controller = Nav2Controller(namespace='/robot5')
    
    # 좌표 변환기 생성
    converter = CoordConverter()
    
    # 웹캠 연결
    camera_index = 2
    cap = cv2.VideoCapture(camera_index)
    
    if not cap.isOpened():
        print(f"❌ Error: {camera_index}번 카메라를 열 수 없습니다.")
        rclpy.shutdown()
        return
    
    print(f"✅ {camera_index}번 카메라 연결 성공")
    print("🖱️ 화면을 클릭하면 로봇이 해당 좌표로 이동합니다.")
    print("   'q' 키를 누르면 종료됩니다.\n")
    
    window_name = "Click to Navigate (Nav2 Direct)"
    cv2.namedWindow(window_name)
    cv2.setMouseCallback(window_name, mouse_callback)
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("프레임을 읽을 수 없습니다.")
            break
        
        # 클릭 지점 시각화
        if latest_click_data:
            cx, cy, mx, my = latest_click_data
            cv2.circle(frame, (cx, cy), 10, (0, 0, 255), -1)
            cv2.circle(frame, (cx, cy), 12, (255, 255, 255), 2)
            
            coord_text = f"Goal: ({mx:.2f}, {my:.2f})"
            cv2.putText(frame, coord_text, (cx + 15, cy - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        # 상태 표시
        if nav2_controller and nav2_controller.is_moving:
            cv2.putText(frame, "STATUS: MOVING", (20, 40),
                       cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 165, 255), 3)
        else:
            cv2.putText(frame, "STATUS: READY", (20, 40),
                       cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 3)
        
        cv2.imshow(window_name, frame)
        
        # ROS2 스핀 (논블로킹)
        rclpy.spin_once(nav2_controller, timeout_sec=0)
        
        # 종료 조건
        if cv2.waitKey(30) & 0xFF == ord('q'):
            break
    
    # 자원 해제
    cap.release()
    cv2.destroyAllWindows()
    nav2_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()