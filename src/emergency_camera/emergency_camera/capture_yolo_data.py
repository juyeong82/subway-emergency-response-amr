import cv2
import os
import datetime

# ==========================================
# [설정 영역]
# ==========================================
CAM_INDEX = 2 # 0 또는 2 (찍을 카메라 번호 변경)
SAVE_DIR = "train_data"  # 저장할 폴더명
TARGET_W = 640          # 목표 너비
TARGET_H = 480           # 목표 높이
# ==========================================

# 저장 폴더 없으면 생성
if not os.path.exists(SAVE_DIR):
    os.makedirs(SAVE_DIR)
    print(f"폴더 확인: {SAVE_DIR}")

def run_clean_collector():
    # 카메라 연결
    cap = cv2.VideoCapture(CAM_INDEX)
    
    if not cap.isOpened():
        print(f"Error: {CAM_INDEX}번 카메라 연결 실패")
        return

    # 1. MJPG 코덱 설정 (버퍼 걸림 방지 및 고해상도 전송용)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    
    # 2. 1280x720 해상도 강제 적용
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, TARGET_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, TARGET_H)

    # 실제 적용된 해상도 확인
    real_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    real_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    
    count = 0
    print(f"=== 데이터 수집 시작 (Camera {CAM_INDEX}) ===")
    print(f"해상도: {real_w}x{real_h}")
    print("------------------------------------------------")
    print("[S] 또는 [Space]: 이미지 저장")
    print("[Q]: 종료")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("프레임 수신 실패")
            break
            
        # 화면 출력용 복사본 생성 (여기에는 정보를 적어도 됨)
        display = frame.copy()
        
        # 현재 저장된 장수 표시 (십자선은 제거함)
        cv2.putText(display, f"Saved: {count}", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        # 화면 출력
        cv2.imshow('Clean Data Collector', display)
        
        key = cv2.waitKey(1) & 0xFF
        
        # 종료
        if key == ord('q'):
            break
            
        # 저장 (S키 또는 스페이스바)
        elif key == ord('s') or key == 32:
            # 저장할 때는 글씨가 없는 'frame' 원본을 사용함
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"cam{CAM_INDEX}_{timestamp}.jpg"
            save_path = os.path.join(SAVE_DIR, filename)
            
            cv2.imwrite(save_path, frame)
            print(f"📸 저장됨: {filename}")
            count += 1

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    run_clean_collector()