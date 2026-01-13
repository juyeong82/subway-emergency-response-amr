
import cv2

def return_camera_indexes():
    # ì—°ê²°ëœ ì¹´ë©”ë¼ ì¸ë±ìŠ¤ ë¦¬ìŠ¤íŠ¸
    available_cameras = []
    
    # 0ë²ˆë¶€í„° 9ë²ˆê¹Œì§€ í¬íŠ¸ í™•ì¸
    for index in range(10):
        cap = cv2.VideoCapture(index)
        
        # ì¹´ë©”ë¼ ìž¥ì¹˜ ì—°ê²° ì„±ê³µ ì—¬ë¶€ í™•ì¸
        if cap.isOpened():
            print(f"ì¹´ë©”ë¼ ë°œê²¬: ì¸ë±ìŠ¤ {index}")
            available_cameras.append(index)
            # í…ŒìŠ¤íŠ¸ ì¢…ë£Œ í›„ ìžì› í•´ì œ
            cap.release()
            
    if not available_cameras:
        print("ì—°ê²°ëœ ì¹´ë©”ë¼ ì—†ìŒ")
        
    return available_cameras

if __name__ == "__main__":
    print("ì›¹ìº  ê²€ìƒ‰ ì‹œìž‘...")
    cams = return_camera_indexes()
    print(f"ì‚¬ìš© ê°€ëŠ¥í•œ ì¹´ë©”ë¼ ì¸ë±ìŠ¤: {cams}")