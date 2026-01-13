import pandas as pd
import matplotlib.pyplot as plt
import os

# 스타일 설정
plt.style.use('ggplot')

# 실험 인덱스 및 설정 매핑 (PDF 내용 기반)
experiments = {
    1: "epoch 100, Batch 32",
    2: "epoch 100, Batch 32, 웹캠 위치 확정 후 학습데이터 추가",
    3: "epoch 100, Batch 16, 웹캠 위치 확정 후 학습데이터 추가",
    4: "epoch 200, Batch 32, 웹캠 위치 확정 후 학습데이터 추가"
}

# 비교할 지표 설정 (csv 컬럼명: 표시할 이름)
metrics = {
    "metrics/mAP50(B)": "mAP50",
    "metrics/mAP50-95(B)": "mAP50-95",
    "metrics/recall(B)": "Recall"
}

# 데이터 로드용 경로 설정
base_path = "csv_data"  # csv 파일이 위치한 폴더명

# 그래프 그리기
for metric_col, metric_name in metrics.items():
    plt.figure(figsize=(14, 8))
    
    # 각 실험 결과 로드 및 플롯
    for idx, label in experiments.items():
        file_path = os.path.join(base_path, f"result{idx}.csv")
        
        if os.path.exists(file_path):
            try:
                # 데이터 읽기
                df = pd.read_csv(file_path)
                
                # 컬럼명 공백 제거
                df.columns = [c.strip() for c in df.columns]
                
                # 해당 지표가 있는지 확인 후 플롯
                if metric_col in df.columns:
                    plt.plot(df['epoch'], df[metric_col], label=f"#{idx} {label}", linewidth=2, alpha=0.8)
            except Exception as e:
                print(f"Error reading {file_path}: {e}")
        else:
            print(f"File not found: {file_path}")
            
    # 그래프 꾸미기
    plt.title(f"Performance Comparison: {metric_name}", fontsize=16, pad=20)
    plt.xlabel("Epochs", fontsize=12)
    plt.ylabel(metric_name, fontsize=12)
    plt.legend(bbox_to_anchor=(1.02, 1), loc='upper left', borderaxespad=0) # 범례 우측 배치
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.tight_layout()
    
    # 저장
    save_name = f"comparison_{metric_name}.png"
    plt.savefig(save_name, dpi=300)
    print(f"Saved {save_name}")
    plt.show()