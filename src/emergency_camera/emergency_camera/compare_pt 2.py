import cv2
import os
import pandas as pd
import time
# [설정] GUI 충돌 방지를 위한 Headless 모드
import matplotlib
matplotlib.use('Agg') 
import matplotlib.pyplot as plt
import seaborn as sns
from ultralytics import YOLO

# =========================================================
# [설정] 환경 설정
# =========================================================

# [변경] 모델 인덱스와 표시할 이름 매핑 (X축 라벨용)
# 실제 파일이 result4.pt -> yolo11, result9.pt -> yolo8 인지 확인 필요
MODEL_MAPPING = {
    4: "yolo11",
    9: "yolo8"
}

# [변경] 연속으로 추론할 프레임 수 (평균 측정용)
TEST_FRAME_COUNT = 16

# 카메라 인덱스 리스트
target_cameras = [0, 2]

# 해상도 및 추론 설정
TARGET_W = 1280
TARGET_H = 720
INFERENCE_SIZE = 1280 

# 파일 경로 패턴
model_path_pattern = "pt_data/result{}.pt"

# 결과 저장 폴더
output_dir = "comparison_result_avg_16frames3"
os.makedirs(output_dir, exist_ok=True)

# =========================================================
# 1. 카메라 초기화 함수
# =========================================================
def init_camera(idx, width, height):
    cap = cv2.VideoCapture(idx)
    if not cap.isOpened():
        return None
    
    # MJPG 코덱 설정
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 10) # 버퍼 약간 늘림
    return cap

# =========================================================
# 2. 멀티 웹캠 연속 이미지 캡처 (16장)
# =========================================================
# {카메라ID: [이미지1, 이미지2, ... 이미지16]} 형태로 저장
captured_sequences = {} 

print(f"📷 카메라 {target_cameras} 연속 캡처 시작 ({TEST_FRAME_COUNT} frames, {TARGET_W}x{TARGET_H})...")

for cam_idx in target_cameras:
    cap = init_camera(cam_idx, TARGET_W, TARGET_H)
    
    if cap is None or not cap.isOpened():
        print(f"⚠️ [Skip] 카메라 {cam_idx}번 연결 실패.")
        continue

    # 카메라 노출 안정화
    print(f"   - 카메라 {cam_idx} 안정화 중...")
    for _ in range(15): 
        cap.read()
        time.sleep(0.05)
    
    frames = []
    print(f"   - 카메라 {cam_idx} {TEST_FRAME_COUNT}장 연속 촬영 중...")
    
    for i in range(TEST_FRAME_COUNT):
        ret, frame = cap.read()
        if ret:
            frames.append(frame)
            # 첫 번째 프레임만 원본 저장 (확인용)
            if i == 0:
                save_path = os.path.join(output_dir, f"original_sample_cam{cam_idx}.jpg")
                cv2.imwrite(save_path, frame)
        else:
            print(f"❌ [Fail] 프레임 {i} 캡처 실패")
            
    cap.release()

    if len(frames) == TEST_FRAME_COUNT:
        captured_sequences[cam_idx] = frames
        print(f"✅ [Success] 카메라 {cam_idx} : {len(frames)}장 저장 완료")
    else:
        print(f"⚠️ [Warning] 카메라 {cam_idx} : {len(frames)}장만 캡처됨 (목표: {TEST_FRAME_COUNT})")
        if frames: # 적게라도 찍혔으면 저장
            captured_sequences[cam_idx] = frames

if not captured_sequences:
    print("❌ 캡처된 이미지가 없습니다. 프로그램을 종료합니다.")
    exit()

# =========================================================
# 3. 모델별 추론 및 성능 측정 (16장 평균)
# =========================================================
results_data = [] 

print(f"\n🚀 모델 성능 비교 시작 (총 {len(MODEL_MAPPING)}개 모델, 모델당 {TEST_FRAME_COUNT}장 추론)...")

for idx, model_name in MODEL_MAPPING.items():
    model_file = model_path_pattern.format(idx)
    
    if not os.path.exists(model_file):
        print(f"⚠️ [Skip] 모델 파일 없음: {model_file}")
        continue
        
    print(f"🔹 [{model_name}] (File: {model_file}) 로드 및 추론 중...")
    
    try:
        model = YOLO(model_file)
        
        # 캡처해둔 카메라별 시퀀스에 대해 추론
        for cam_idx, frames_list in captured_sequences.items():
            
            # 각 프레임 반복 추론
            for frame_num, frame in enumerate(frames_list):
                
                # 추론 (imgsz=1280, half=True)
                results = model(frame, imgsz=INFERENCE_SIZE, verbose=False, half=True)
                
                for r in results:
                    # 속도 추출 (inference time)
                    speed = r.speed
                    inference_time = speed['inference']
                    
                    # 마지막 프레임인 경우에만 결과 이미지 저장 (덮어쓰기 방지 및 속도 최적화)
                    if frame_num == TEST_FRAME_COUNT - 1:
                        im_array = r.plot()
                        info_text = f"{model_name} | Cam:{cam_idx} | {inference_time:.1f}ms"
                        cv2.putText(im_array, info_text, (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 
                                    1, (0, 0, 255), 2, cv2.LINE_AA)
                        save_filename = f"result_{model_name}_cam{cam_idx}_sample.jpg"
                        cv2.imwrite(os.path.join(output_dir, save_filename), im_array)

                    # 데이터 수집 (모든 프레임 기록 -> 나중에 평균 계산)
                    boxes = r.boxes
                    
                    # 검출된 객체가 없어도 추론 시간은 기록해야 함
                    if len(boxes) == 0:
                         results_data.append({
                            "Model": model_name,     # yolo11, yolo8 등
                            "Camera": f"Cam {cam_idx}",
                            "Class": "None",
                            "Confidence": 0.0,
                            "Inference Time (ms)": inference_time,
                            "Frame Index": frame_num
                        })
                    else:
                        for box in boxes:
                            cls_id = int(box.cls[0])
                            cls_name = model.names[cls_id]
                            conf = float(box.conf[0])
                            
                            results_data.append({
                                "Model": model_name,
                                "Camera": f"Cam {cam_idx}",
                                "Class": cls_name,
                                "Confidence": conf,
                                "Inference Time (ms)": inference_time,
                                "Frame Index": frame_num
                            })
                
    except Exception as e:
        print(f"❌ Error processing model {model_name}: {e}")

# =========================================================
# 4. 결과 시각화 및 저장
# =========================================================
if not results_data:
    print("데이터가 없어 종료합니다.")
    exit()

df = pd.DataFrame(results_data)

# 4-1. 정확도(Confidence) 그래프
plt.figure(figsize=(14, 7))
sns.set_style("whitegrid")
# 모델 이름(yolo11, yolo8)이 X축으로 표시됨
ax1 = sns.barplot(data=df[df["Class"] != "None"], x="Model", y="Confidence", hue="Class", palette="viridis", errorbar=None)
plt.title(f"Model Accuracy Comparison ({TEST_FRAME_COUNT} frames avg)", fontsize=15)
plt.ylim(0, 1.1)
plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
for container in ax1.containers:
    ax1.bar_label(container, fmt='%.2f', padding=3)
plt.tight_layout()
plt.savefig(os.path.join(output_dir, "graph_accuracy_avg.png"))
plt.close()

# 4-2. 추론 시간(Inference Time) 그래프 (평균값 표시)
# 프레임별 중복 데이터 제거 (한 프레임에 객체가 여러 개여도 추론 시간은 1개)
df_time = df[["Model", "Camera", "Inference Time (ms)", "Frame Index"]].drop_duplicates()

plt.figure(figsize=(12, 6))
sns.set_style("whitegrid")
# barplot은 기본적으로 평균(estimator='mean')을 보여줌. 검은색 선(errorbar)은 표준편차/신뢰구간.
ax2 = sns.barplot(data=df_time, x="Model", y="Inference Time (ms)", hue="Camera", palette="rocket")

plt.title(f"Average Inference Speed ({TEST_FRAME_COUNT} frames, half=True)", fontsize=15)
plt.ylabel("Inference Time (ms)", fontsize=12)

# 바 위에 평균값 숫자 표시
for container in ax2.containers:
    ax2.bar_label(container, fmt='%.1f', padding=3)

plt.tight_layout()
plt.savefig(os.path.join(output_dir, "graph_inference_time_avg.png"))
plt.close()

# 4-3. CSV 저장
csv_save_path = os.path.join(output_dir, "detection_results_avg.csv")
df.sort_values(by=["Model", "Camera", "Class"], inplace=True)
df.to_csv(csv_save_path, index=False)

print(f"\n✅ 모든 작업 완료. 결과는 '{output_dir}' 폴더를 확인하세요.")