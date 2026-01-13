
import cv2

def return_camera_indexes():
    # 연결된 카메라 인덱스 리스트
    available_cameras = []
    
    # 0번부터 9번까지 포트 확인
    for index in range(10):
        cap = cv2.VideoCapture(index)
        
        # 카메라 장치 연결 성공 여부 확인
        if cap.isOpened():
            print(f"카메라 발견: 인덱스 {index}")
            available_cameras.append(index)
            # 테스트 종료 후 자원 해제
            cap.release()
            
    if not available_cameras:
        print("연결된 카메라 없음")
        
    return available_cameras

if __name__ == "__main__":
    print("웹캠 검색 시작...")
    cams = return_camera_indexes()
    print(f"사용 가능한 카메라 인덱스: {cams}")