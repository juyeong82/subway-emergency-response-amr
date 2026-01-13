import cv2
import numpy as np
import os

# [설정] control_tower.py와 동일한 환경
CAM_LEFT_ID = 0
CAM_RIGHT_ID = 2
CAM_WIDTH = 1280
CAM_HEIGHT = 720

# 클릭한 점들을 저장할 리스트
points = []

def click_event(event, x, y, flags, param):
    global points
    if event == cv2.EVENT_LBUTTONDOWN:
        # 좌표 추가
        points.append((x, y))
        print(f"📍 포인트 추가: ({x}, {y})")

def get_polygon_coordinates(cam_id, window_name):
    global points
    points = [] # 초기화
    
    cap = cv2.VideoCapture(cam_id)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)
    
    if not cap.isOpened():
        print(f"❌ 카메라 {cam_id}를 열 수 없습니다.")
        return None

    cv2.namedWindow(window_name)
    cv2.setMouseCallback(window_name, click_event)

    print(f"\n--- [{window_name}] 설정 모드 ---")
    print("1. 마우스 왼쪽 클릭: 맵의 유효한 경계(꼭짓점)를 찍으세요.")
    print("2. 'z' 키: 마지막 점 취소")
    print("3. 's' 키: 저장 (좌표 출력 + 📷사진 저장) 후 종료")
    print("4. 'q' 키: 저장 없이 종료")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # [시각화] 찍은 점들을 선으로 연결해서 보여줌
        if len(points) > 0:
            # 점 그리기
            for pt in points:
                cv2.circle(frame, pt, 5, (0, 0, 255), -1)
            
            # 선 그리기 (다각형)
            if len(points) > 1:
                pts = np.array(points, np.int32)
                pts = pts.reshape((-1, 1, 2))
                # 닫힌 다각형으로 그리기 (True)
                cv2.polylines(frame, [pts], True, (0, 255, 0), 2)

        cv2.imshow(window_name, frame)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            points = [] # 초기화
            break
        elif key == ord('z'): # 실행 취소
            if points:
                popped = points.pop()
                print(f"↩️ 취소됨: {popped}")
        elif key == ord('s'): # 저장
            # 이미지 파일 저장 로직 추가
            if points:
                filename = f"polygon_cam_{cam_id}.png"
                cv2.imwrite(filename, frame)
                print(f"📷 [저장 완료] 이미지 파일 생성됨: {filename}")
            break
    
    cap.release()
    cv2.destroyAllWindows()
    return points

if __name__ == "__main__":
    print("=== 맵 유효 영역(Polygon) 좌표 따기 도구 (이미지 저장 기능 포함) ===")
    
    # 1. 왼쪽 카메라 설정
    print("\n[1/2] 왼쪽 카메라(CAM_LEFT_ID) 설정을 시작합니다.")
    left_poly = get_polygon_coordinates(CAM_LEFT_ID, "Left Camera Setup")
    
    if left_poly:
        print("\n✅ 왼쪽 카메라 Polygon 좌표 (복사해서 사용하세요):")
        print(f"LEFT_POLY = {left_poly}")
    
    # 2. 오른쪽 카메라 설정
    print("\n[2/2] 오른쪽 카메라(CAM_RIGHT_ID) 설정을 시작하시겠습니까? (y/n)")
    ans = input().strip().lower()
    if ans == 'y':
        right_poly = get_polygon_coordinates(CAM_RIGHT_ID, "Right Camera Setup")
        if right_poly:
            print("\n✅ 오른쪽 카메라 Polygon 좌표 (복사해서 사용하세요):")
            print(f"RIGHT_POLY = {right_poly}")

    print("\n=== 종료 ===")