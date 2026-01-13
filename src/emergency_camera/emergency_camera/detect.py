import cv2
from ultralytics import YOLO

# 1. 학습된 모델 불러오기 (경로 수정 필요)
# 보통 'runs/detect/train/weights/best.pt'에 저장됨
model_path = '/home/juyeong/subway_robot_ws/src/emergency_camera/emergency_camera/subway_project/train_result8/weights/result08.pt' 
# model_path = '/home/juyeong/subway_robot_ws/src/emergency_camera/emergency_camera/models/yolo11n.pt' 


model = YOLO(model_path)

# 2. 웹캠 연결 (사용자 요청: 2번 카메라)
camera_index = 2
cap = cv2.VideoCapture(camera_index)

# 카메라 연결 확인
if not cap.isOpened():
    print(f"❌ Error: {camera_index}번 카메라를 열 수 없습니다.")
    exit()

print(f"✅ {camera_index}번 카메라 연결 성공. 'q'를 누르면 종료됩니다.")

# 3. 실시간 추론 루프
while True:
    # 프레임 읽기
    ret, frame = cap.read()
    if not ret:
        print("프레임을 읽을 수 없습니다.")
        break

    # 모델 추론 (conf: 신뢰도 임계값 조절 가능)
    # stream=True는 메모리 효율을 위해 권장됨
    results = model(frame, conf=0.5, verbose=False)

    # 결과 시각화 (YOLO 내장 함수 사용)
    # 추론 결과를 이미지 위에 그려서 리턴함
    annotated_frame = results[0].plot()

    # 화면 출력
    cv2.imshow("YOLOv8 Webcam Inference", annotated_frame)

    # 종료 조건 ('q' 키 누르면 종료)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 4. 자원 해제
cap.release()
cv2.destroyAllWindows()