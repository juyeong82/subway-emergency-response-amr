import cv2
import os
import pandas as pd
# [설정] GUI 충돌 방지를 위한 Headless 모드 (가장 먼저 실행)
import matplotlib
matplotlib.use('Agg') 
import matplotlib.pyplot as plt
import seaborn as sns
from ultralytics import YOLO

# ---------------------------------------------------------
# 설정값 정의
# ---------------------------------------------------------
# 모델 인덱스 리스트
target_indices = [2, 3, 6, 9, 10, 11, 12, 13, 15]

# 파일 경로 패턴
model_path_pattern = "pt_data/result{}.pt"

# 결과 저장 폴더 생성
output_dir = "comparison_result"
os.makedirs(output_dir, exist_ok=True)

# ---------------------------------------------------------
# 1. 웹캠 이미지 캡처 (동일 조건 비교용)
# ---------------------------------------------------------
cap = cv2.VideoCapture(2)

if not cap.isOpened():
    print("웹캠을 열 수 없습니다. 카메라 연결을 확인하세요.")
    exit()

print("웹캠 초기화 중... (3초 대기)")
cv2.waitKey(3000) # 카메라 노출 안정화 대기

ret, frame = cap.read()
cap.release()

if not ret:
    print("프레임을 캡처하지 못했습니다.")
    exit()

# 원본 이미지 저장
original_img_path = os.path.join(output_dir, "original_capture.jpg")
cv2.imwrite(original_img_path, frame)
print(f"테스트 이미지 캡처 완료: {original_img_path}")

# ---------------------------------------------------------
# 2. 모델별 추론 및 데이터 수집 (시간 측정 추가)
# ---------------------------------------------------------
results_data = [] # 결과 저장용 리스트

print("모델 성능 및 추론 시간 측정 시작...")

for idx in target_indices:
    model_file = model_path_pattern.format(idx)
    
    if not os.path.exists(model_file):
        print(f"[Skip] 모델 파일 없음: {model_file}")
        continue
        
    print(f"[{model_file}] 모델 로드 및 추론 중...")
    
    try:
        # 모델 로드
        model = YOLO(model_file)
        
        # 추론 실행
        results = model(original_img_path, verbose=False)
        
        # 결과 처리
        for r in results:
            # 추론 시간 계산 (전처리 + 추론 + 후처리)
            speed = r.speed
            inference_time = speed['inference'] # 순수 추론 시간
            total_time = speed['preprocess'] + speed['inference'] + speed['postprocess'] # 전체 소요 시간
            
            # 결과 이미지 생성 (bbox)
            im_array = r.plot()
            
            # [추가] 이미지에 추론 시간 텍스트 표시 (좌측 상단, 빨간색)
            time_text = f"Inference: {inference_time:.1f}ms"
            cv2.putText(im_array, time_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                        1, (0, 0, 255), 2, cv2.LINE_AA)
            
            # 결과 이미지 파일 저장
            save_path = os.path.join(output_dir, f"result_img_model_{idx}.jpg")
            cv2.imwrite(save_path, im_array)
            
            # 탐지된 객체 정보 추출 및 데이터 저장
            boxes = r.boxes
            if len(boxes) == 0:
                # 탐지된 객체가 없어도 추론 시간은 기록
                 results_data.append({
                    "Model": f"Result {idx}",
                    "Class": "None",
                    "Confidence": 0.0,
                    "Inference Time (ms)": inference_time
                })
            else:
                for box in boxes:
                    cls_id = int(box.cls[0])
                    cls_name = model.names[cls_id]
                    conf = float(box.conf[0])
                    
                    # 데이터 추가
                    results_data.append({
                        "Model": f"Result {idx}",
                        "Class": cls_name,
                        "Confidence": conf,
                        "Inference Time (ms)": inference_time
                    })
                
    except Exception as e:
        print(f"Error processing model {idx}: {e}")

# ---------------------------------------------------------
# 3. 결과 시각화 (정확도 & 추론 시간)
# ---------------------------------------------------------
if not results_data:
    print("탐지된 데이터가 없습니다.")
    exit()

df = pd.DataFrame(results_data)

# 3-1. 정확도(Confidence) 그래프
plt.figure(figsize=(12, 6))
sns.set_style("whitegrid")
ax1 = sns.barplot(data=df[df["Class"] != "None"], x="Model", y="Confidence", hue="Class", palette="viridis", errorbar=None)
plt.title("YOLO Model Accuracy Comparison (Confidence)", fontsize=15)
plt.ylim(0, 1.1)
plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
for container in ax1.containers:
    ax1.bar_label(container, fmt='%.2f', padding=3)
plt.tight_layout()
plt.savefig(os.path.join(output_dir, "graph_accuracy.png"))
plt.close()

# 3-2. 추론 시간(Inference Time) 그래프 [추가됨]
# 모델별 중복 데이터 제거 (이미지 1장당 시간은 동일하므로)
df_time = df[["Model", "Inference Time (ms)"]].drop_duplicates()

plt.figure(figsize=(12, 6))
sns.set_style("whitegrid")
# 색상은 파스텔톤 'rocket' 팔레트 사용
ax2 = sns.barplot(data=df_time, x="Model", y="Inference Time (ms)", palette="rocket", hue="Model", legend=False)

plt.title("YOLO Model Inference Speed Comparison (Lower is Better)", fontsize=15)
plt.ylabel("Inference Time (ms)", fontsize=12)
plt.xlabel("Model Version", fontsize=12)

# 막대 위에 시간(ms) 표시
for container in ax2.containers:
    ax2.bar_label(container, fmt='%.1f ms', padding=3)

plt.tight_layout()
plt.savefig(os.path.join(output_dir, "graph_inference_time.png"))
plt.close()

print("그래프 저장 완료: graph_accuracy.png, graph_inference_time.png")

# ---------------------------------------------------------
# 4. 요약 CSV 저장
# ---------------------------------------------------------
csv_save_path = os.path.join(output_dir, "detection_results_with_time.csv")
df.sort_values(by=["Model", "Class"], inplace=True)
df.to_csv(csv_save_path, index=False)
print(f"상세 데이터 저장 완료: {csv_save_path}")

print("모든 작업이 완료되었습니다.")