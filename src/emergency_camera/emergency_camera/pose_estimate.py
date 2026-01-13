import cv2
import math
import numpy as np
from ultralytics import YOLO

def calculate_angle(p1, p2):
    """두 점(p1, p2) 사이의 각도를 계산 (수직선 기준)"""
    x1, y1 = p1
    x2, y2 = p2
    dx = x2 - x1
    dy = y2 - y1
    rad = math.atan2(dy, dx)
    deg = math.degrees(rad)
    return abs(deg)

def main():
    # 1. YOLOv11 Pose 모델 로드 (자동 다운로드됨)
    # n(nano) 모델이 가장 빠름. 정확도가 필요하면 'yolo11s-pose.pt' 사용
    model = YOLO('yolo11n-pose.pt') 

    # 2. 웹캠 연결 (0번 또는 사용자 환경에 맞게 변경)
    cap = cv2.VideoCapture(0)
    
    # 임계값 설정
    FALL_RATIO_THRES = 1.1      # 너비/높이 비율이 이보다 크면 의심
    FALL_ANGLE_THRES = 45       # 몸통 각도가 45도보다 누워있으면 의심

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        # 3. 추론 실행 (conf: 확신도, verbose: 로그 끔)
        results = model.predict(frame, conf=0.5, verbose=False)
        
        # 결과 시각화 용 이미지
        annotated_frame = frame.copy()

        for result in results:
            # 박스 정보 가져오기
            boxes = result.boxes.xyxy.cpu().numpy()
            
            # 키포인트 정보 가져오기 (x, y, conf)
            if result.keypoints is not None:
                keypoints = result.keypoints.xy.cpu().numpy()
            else:
                keypoints = []

            for i, box in enumerate(boxes):
                x1, y1, x2, y2 = box
                w = x2 - x1
                h = y2 - y1
                
                # 키포인트 추출 (5:왼쪽어깨, 6:오른쪽어깨, 11:왼쪽골반, 12:오른쪽골반)
                # COCO 데이터셋 기준 인덱스
                kpts = keypoints[i]
                
                if len(kpts) > 0:
                    # 유효한 키포인트인지 확인 (0,0 이면 감지 안된 것)
                    left_shoulder = kpts[5]
                    right_shoulder = kpts[6]
                    left_hip = kpts[11]
                    right_hip = kpts[12]

                    # 1. Bounding Box 비율 검사 (너비가 높이보다 큰지)
                    aspect_ratio = w / h
                    
                    # 2. 몸통 각도 검사 (어깨 중앙과 골반 중앙을 잇는 선)
                    is_falling_angle = False
                    if (left_shoulder[0] > 0 and left_hip[0] > 0): # 왼쪽 감지됨
                         angle = calculate_angle(left_shoulder, left_hip)
                         # 수직(90도)에서 멀어질수록 누운 것. (0도나 180도에 가까우면 누운 것)
                         # atan2 결과상 y가 증가하는 방향이 아래라 각도 계산이 좀 다를 수 있음.
                         # 간단히: dx가 dy보다 크면 누운 것임.
                         if abs(left_shoulder[0] - left_hip[0]) > abs(left_shoulder[1] - left_hip[1]):
                             is_falling_angle = True
                    
                    # 판단 로직: 비율이 넓거나 or 몸통이 가로로 누웠을 때
                    # (비율만 쓰면 앉아있는 사람도 오인식 할 수 있으니 각도와 함께 보는게 좋음)
                    # 여기선 심플하게 '비율'이 옆으로 길면 Fall로 간주
                    if aspect_ratio > FALL_RATIO_THRES or is_falling_angle:
                        color = (0, 0, 255) # 빨간색 (위험)
                        status = "FALL DETECTED!"
                    else:
                        color = (0, 255, 0) # 초록색 (정상)
                        status = "Standing"

                    # 박스 그리기
                    cv2.rectangle(annotated_frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
                    cv2.putText(annotated_frame, status, (int(x1), int(y1)-10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

        # 4. 뼈대 그리기 (YOLO 기본 기능 활용)
        annotated_frame = results[0].plot(img=annotated_frame)

        cv2.imshow("YOLOv11 Pose Fall Detection", annotated_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
    