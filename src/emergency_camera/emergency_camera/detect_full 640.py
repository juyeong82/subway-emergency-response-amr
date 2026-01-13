import cv2
import time
from ultralytics import YOLO

# =========================================================
# [설정] 사용자 정의 변수
# =========================================================
MODEL_PATH = "pt_data/result8.pt" 
CAM1_IDX = 3
CAM2_IDX = 2

# =========================================================
# 1. 모델 로드
# =========================================================
print(f"[{MODEL_PATH}] 모델 로딩 중...")
model = YOLO(MODEL_PATH)

# =========================================================
# 2. 카메라 연결 (예외 처리 적용)
# =========================================================
print("카메라 연결 시도 중...")

# 활성화된 카메라를 관리할 리스트
active_caps = []

# 함수: 카메라 연결 시도 및 리스트 추가
def try_connect_camera(idx, name):
    cap = cv2.VideoCapture(idx)
    if cap.isOpened():
        print(f"✅ {name} 연결 성공")
        return {'cap': cap, 'name': name}
    else:
        print(f"⚠️ {name} 연결 실패 (건너뜀)")
        return None

# 각각 연결 시도
cam1_info = try_connect_camera(CAM1_IDX, "Camera 2")
if cam1_info: active_caps.append(cam1_info)

cam2_info = try_connect_camera(CAM2_IDX, "Camera 4")
if cam2_info: active_caps.append(cam2_info)

# 연결된 카메라가 하나도 없으면 종료
if not active_caps:
    print("❌ 연결된 카메라가 없습니다. 프로그램을 종료합니다.")
    exit()

print(f"🚀 총 {len(active_caps)}대 카메라로 실시간 추론 시작! (종료: 'q')")

# =========================================================
# 3. 실시간 루프
# =========================================================
prev_time = 0

while True:
    frames = []
    valid_caps_info = [] # 이번 프레임에서 읽기 성공한 카메라 정보

    # -----------------------------------------------------
    # [읽기] 활성화된 카메라들만 루프 돌며 프레임 수집
    # -----------------------------------------------------
    for cam_info in active_caps:
        ret, frame = cam_info['cap'].read()
        if ret:
            frames.append(frame)
            valid_caps_info.append(cam_info)
        else:
            # 일시적 끊김 혹은 연결 해제 시 그냥 패스 (프로그램 안 죽음)
            pass

    # 만약 모든 카메라에서 프레임을 못 받았다면 종료 혹은 대기
    if not frames:
        print("모든 카메라 프레임 수신 실패 (재시도 중...)")
        if cv2.waitKey(1) & 0xFF == ord('q'): break
        continue

    # -----------------------------------------------------
    # [추론] 수집된 프레임 일괄 추론 (Batch Inference)
    # -----------------------------------------------------
    # 기본 설정 유지 (imgsz 자동, conf 기본값)
    results = model(frames, verbose=False)

    # -----------------------------------------------------
    # [시각화] 결과 그리기 및 FPS 표시
    # -----------------------------------------------------
    display_frames = []
    
    # FPS 계산
    curr_time = time.time()
    fps = 1 / (curr_time - prev_time) if prev_time != 0 else 0
    prev_time = curr_time

    for i, res in enumerate(results):
        # 결과 이미지 생성
        res_plot = res.plot()
        
        # 카메라 이름 가져오기
        cam_name = valid_caps_info[i]['name']

        # 텍스트 추가
        cv2.putText(res_plot, f"FPS: {fps:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                    1, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(res_plot, cam_name, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 
                    0.7, (255, 255, 255), 2)
        
        display_frames.append(res_plot)

    # -----------------------------------------------------
    # [화면 병합 및 출력] 개수에 따라 유동적으로 처리
    # -----------------------------------------------------
    if len(display_frames) == 1:
        # 카메라가 1대만 작동 중일 때
        cv2.imshow("Real-time Inference", display_frames[0])
        
    elif len(display_frames) >= 2:
        # 카메라가 2대 이상일 때 가로 병합
        try:
            combined_frame = cv2.hconcat(display_frames)
            cv2.imshow("Real-time Inference", combined_frame)
        except Exception as e:
            print(f"화면 병합 오류: {e}")

    # 'q' 키 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 종료 정리
for cam_info in active_caps:
    cam_info['cap'].release()
cv2.destroyAllWindows()
print("프로그램 종료")