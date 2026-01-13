#!/usr/bin/env python3
import cv2
import numpy as np

# ==========================================
# 1. 좌표 변환 클래스
# ==========================================
class CoordConverter:
    def __init__(self):
        # ---------------------------------------------------------
        # [사용자 설정] 측정한 캘리브레이션 좌표 (픽셀 -> 맵)
        # ---------------------------------------------------------
        # 화면상 픽셀 좌표 (u, v) - 카메라 움직이면 이 위치가 틀어짐
        self.pixel_points = np.array([
            [455, 95],     # Point 1
            [819, 91],     # Point 2
            [1225, 658],   # Point 3
            [45, 647]      # Point 4
        ], dtype=np.float32)

        # 대응되는 맵 좌표 (x, y)
        self.map_points = np.array([
            [2.85043, -0.64341],   # Point 1
            [3.27524, -3.79587],   # Point 2
            [-1.63256, -4.55022],  # Point 3
            [-1.99700, -1.62762]   # Point 4
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
# 2. OpenCV 메인 루프
# ==========================================
converter = None
latest_click_data = None  # 화면 표시용 데이터

def mouse_callback(event, x, y, flags, param):
    global latest_click_data, converter
    
    if event == cv2.EVENT_LBUTTONDOWN:  # 마우스 왼쪽 클릭
        map_pos = converter.pixel_to_map(x, y)
        map_x, map_y = map_pos[0], map_pos[1]
        
        print(f"\n🖱️  [Click Pixel]: ({x}, {y})")
        print(f"📍 [Map Coord]  : x={map_x:.4f}, y={map_y:.4f}")
        print("-" * 40)
        
        latest_click_data = (x, y, map_x, map_y)

def main():
    global converter, latest_click_data
    
    # 좌표 변환기 생성
    converter = CoordConverter()
    
    # 웹캠 연결 (환경에 맞게 인덱스 수정)
    camera_index = 2
    cap = cv2.VideoCapture(camera_index)
    
    # MJPG 코덱 및 해상도 설정
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    
    if not cap.isOpened():
        print(f"❌ Error: {camera_index}번 카메라를 열 수 없습니다.")
        return
    
    print(f"✅ {camera_index}번 카메라 연결 성공")
    print("--------------------------------------------------")
    print("🟣 보라색 점 : 캘리브레이션 기준점 (카메라 고정 확인용)")
    print("🖱️ 클릭 지점 : 변환된 Map 좌표 출력")
    print("--------------------------------------------------\n")
    
    window_name = "Coordinate Debug Mode"
    cv2.namedWindow(window_name)
    cv2.setMouseCallback(window_name, mouse_callback)
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("프레임을 읽을 수 없습니다.")
            break
        
        # ---------------------------------------------------------
        # [추가] 캘리브레이션 기준점 4개 시각화 (카메라 위치 확인용)
        # ---------------------------------------------------------
        # 설정해둔 4개 점 위치에 보라색 점을 찍음
        # 이 점들이 실제 바닥의 마커 위치와 일치하는지 항상 확인 가능
        for i, point in enumerate(converter.pixel_points):
            px, py = int(point[0]), int(point[1])
            
            # 기준점 표시 (보라색 원)
            cv2.circle(frame, (px, py), 6, (255, 0, 255), -1) 
            # 시인성을 위한 노란 테두리
            cv2.circle(frame, (px, py), 8, (0, 255, 255), 1)
            # 번호 표시 (#1, #2...)
            cv2.putText(frame, f"#{i+1}", (px + 10, py - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)

        # ---------------------------------------------------------
        # 클릭 지점 시각화 (사용자가 찍은 곳)
        # ---------------------------------------------------------
        if latest_click_data:
            cx, cy, mx, my = latest_click_data
            
            # 클릭 지점 표시 (빨간 점)
            cv2.circle(frame, (cx, cy), 6, (0, 0, 255), -1)
            cv2.circle(frame, (cx, cy), 10, (0, 255, 255), 2)
            
            coord_text = f"Map: ({mx:.3f}, {my:.3f})"
            
            # 텍스트 그림자
            cv2.putText(frame, coord_text, (cx + 15, cy - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4)
            # 메인 텍스트
            cv2.putText(frame, coord_text, (cx + 15, cy - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # 상태 표시
        cv2.putText(frame, "[Check Calibration Points]", (20, 40),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 255), 2)
        
        cv2.imshow(window_name, frame)
        
        if cv2.waitKey(30) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()