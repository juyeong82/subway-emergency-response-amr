import cv2
import os
import datetime

# 캡처 저장 폴더 생성
SAVE_DIR = "captures"
if not os.path.exists(SAVE_DIR):
    os.makedirs(SAVE_DIR)

# 테스트할 해상도 리스트 (너비, 높이)
RESOLUTIONS = [
    # (640, 480),
    # (1280, 720),
    # (1920, 1080)
]

def apply_digital_zoom(frame, zoom_factor):
    """
    이미지 중심을 기준으로 잘라내어 줌 효과 구현
    zoom_factor: 1.0(기본) ~ 
    """
    if zoom_factor == 1.0:
        return frame

    h, w = frame.shape[:2]
    
    # 줌 비율에 따른 새로운 너비/높이 계산
    new_w = int(w / zoom_factor)
    new_h = int(h / zoom_factor)
    
    # 중심점 계산
    center_x, center_y = w // 2, h // 2
    
    # 크롭 영역 좌표 계산 (화면 밖으로 나가지 않게 클리핑)
    x1 = max(0, center_x - new_w // 2)
    y1 = max(0, center_y - new_h // 2)
    x2 = min(w, x1 + new_w)
    y2 = min(h, y1 + new_h)
    
    # 이미지 자르기 (Cropping)
    cropped = frame[y1:y2, x1:x2]
    
    # 원본 크기로 다시 확대 (Resize)
    return cv2.resize(cropped, (w, h), interpolation=cv2.INTER_LINEAR)

def main():
    cam_index = 0
    
    cap = cv2.VideoCapture(cam_index)
    
    # 초기 설정 변수
    res_idx = 0     # 해상도 리스트 인덱스
    zoom_val = 1.0  # 줌 배율 (1.0 = 1배)
    
    # 초기 해상도 적용
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, RESOLUTIONS[res_idx][0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, RESOLUTIONS[res_idx][1])
    
    print("=== 조작법 ===")
    print("[Z]: 줌 인 (+0.1)")
    print("[X]: 줌 아웃 (-0.1)")
    print("[R]: 해상도 변경 (순환)")
    print("[C]: 사진 캡처")
    print("[Q]: 종료")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("프레임 수신 실패")
            break
            
        # 1. 디지털 줌 적용
        zoomed_frame = apply_digital_zoom(frame, zoom_val)
        
        # 2. 현재 상태 텍스트 오버레이 (화면 좌상단)
        curr_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        curr_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        
        info_text = f"Res: {curr_w}x{curr_h} | Zoom: x{zoom_val:.1f}"
        cv2.putText(zoomed_frame, info_text, (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        cv2.imshow('Custom Camera Controller', zoomed_frame)
        
        # 키 입력 처리
        key = cv2.waitKey(1) & 0xFF
        
        # 종료
        if key == ord('q'):
            break
            
        # 줌 조절 (0.1 단위)
        elif key == ord('z'):
            zoom_val += 0.1
            if zoom_val > 5.0: zoom_val = 5.0 # 최대 5배 제한
            
        elif key == ord('x'):
            zoom_val -= 0.1
            if zoom_val < 1.0: zoom_val = 1.0 # 최소 1배 제한
            
        # 해상도 변경
        elif key == ord('r'):
            res_idx = (res_idx + 1) % len(RESOLUTIONS)
            w, h = RESOLUTIONS[res_idx]
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
            print(f"해상도 변경 시도: {w}x{h}")
            
        # 캡처 (현재 줌/해상도 상태 그대로 저장)
        elif key == ord('c'):
            filename = datetime.datetime.now().strftime("%Y%m%d_%H%M%S.jpg")
            path = os.path.join(SAVE_DIR, filename)
            cv2.imwrite(path, zoomed_frame)
            print(f"캡처 저장됨: {path}")

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()