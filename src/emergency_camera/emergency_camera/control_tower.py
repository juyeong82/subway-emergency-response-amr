import sys
import os
import sqlite3
import datetime
import time
import threading
import cv2
import numpy as np
import json
import math
import webbrowser
from threading import Timer
from glob import glob

# --- [라이브러리 임포트] ---
from flask import Flask, render_template, request, redirect, url_for, session, Response, jsonify, flash
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Point, PoseStamped  # PoseStamped 추가
from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO  # YOLO 추가

# =========================================================
# [설정] 사용자 정의 상수 (변경 필요 시 여기만 수정)
# =========================================================
# 1. 카메라 설정 (사용자 요청: 좌=4, 우=0)
CAM_LEFT_ID = 0   # 좌측 카메라 (Robot 5 구역 예상)
CAM_RIGHT_ID = 5  # 우측 카메라 (Robot 3 구역 예상)

# 2. YOLO 설정
# 모델 경로 (절대 경로로 수정 권장)
YOLO_MODEL_PATH = '/home/juyeong/subway_robot_ws/src/emergency_camera/emergency_camera/models/result04.pt'
CONF_THRESHOLD = 0.5     # 감지 신뢰도 임계값
DIST_THRESHOLD = 200.0   # [Pixel] 환자-구급대원 근접 인식 거리 (도착 판정)

# 3. 클래스 ID 매핑 (학습된 모델의 class id 확인 필요)
# TODO: data.yaml 확인 후 번호 수정하세요. (현재는 가상의 ID 0, 1로 설정)
CLASS_ID_PATIENT = 2    # 환자 (fallen, patient 등)
CLASS_ID_RESPONDER = 3  # 구급대원/로봇 (responder, robot 등)

# 4. 카메라 해상도 (Detection 성능 유지를 위해 고정)
CAM_WIDTH = 1280
CAM_HEIGHT = 720

# =========================================================

# --- [Flask 애플리케이션 설정] ---
package_name = 'subway_control'
try:
    template_dir = os.path.join(get_package_share_directory(package_name), 'templates')
    app = Flask(__name__, template_folder=template_dir)
except:
    app = Flask(__name__, template_folder='templates')

app.secret_key = 'subway_secret_key'

# --- [데이터베이스 경로] ---
user_home = os.path.expanduser('~')
DB_NAME = os.path.join(user_home, '/home/juyeong/subway_robot_ws/src/emergency_camera/emergency_camera/subway_log.db')

# --- [전역 변수] ---
robots_data = {"robot3": {"bat": 0, "x": 0.0, "y": 0.0, "status": "연결 대기"},
               "robot5": {"bat": 0, "x": 0.0, "y": 0.0, "status": "연결 대기"}}
camera_status = {1: False, 2: False} # 1: Left(4), 2: Right(0) 로 매핑 예정

# 영상 공유를 위한 전역 프레임 버퍼 (VisionSystem에서 갱신 -> Flask에서 송출)
global_frame_left = None
global_frame_right = None
frame_lock = threading.Lock() # 스레드 충돌 방지

# --- [DB 함수 생략 (기존과 동일)] ---
def init_db():
    db_dir = os.path.dirname(DB_NAME)
    if not os.path.exists(db_dir): os.makedirs(db_dir)
    conn = sqlite3.connect(DB_NAME, check_same_thread=False)
    c = conn.cursor()
    c.execute("CREATE TABLE IF NOT EXISTS emergency_history (id INTEGER PRIMARY KEY AUTOINCREMENT, content TEXT, timestamp TEXT)")
    c.execute("CREATE TABLE IF NOT EXISTS robot_logs (id INTEGER PRIMARY KEY AUTOINCREMENT, content TEXT, timestamp TEXT)")
    c.execute("CREATE TABLE IF NOT EXISTS users (username TEXT PRIMARY KEY, password TEXT)")
    try:
        c.execute("INSERT OR IGNORE INTO users (username, password) VALUES (?, ?)", ('rokey', 'rokey1234'))
        conn.commit()
    except: pass
    conn.close()

def save_log(table, content):
    try:
        conn = sqlite3.connect(DB_NAME)
        c = conn.cursor()
        now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3] 
        c.execute(f"INSERT INTO {table} (content, timestamp) VALUES (?, ?)", (content, now))
        conn.commit()
        conn.close()
    except Exception as e: print(f"[DB Error] {e}")


# =========================================================
# [Core 1] 호모그래피 변환 클래스 (파일 기반 통합)
# =========================================================
class HomographyConverter:
    def __init__(self, cam_id):
        self.cam_id = cam_id
        self.H = None
        self.init_matrix()

    def init_matrix(self):
        # ---------------------------------------------------------
        # [수정] 좌/우 카메라 데이터 교체 및 포맷 변경
        # ---------------------------------------------------------
        
        if self.cam_id == CAM_LEFT_ID: 
            # 좌측 카메라 (기존 Right 데이터였던 ID 0번 데이터 적용)
            # webcam_debug_left.py 데이터
            pixel_pts = np.array([
                [329, 241],    # 1번 점 (좌상)
                [949, 214],    # 2번 점 (우상)
                [1242, 594],   # 3번 점 (우하)
                [137, 702]     # 4번 점 (좌하)
            ], dtype=np.float32)

            map_pts = np.array([
                [-0.40825, 2.43331],   # 1번 점 매핑
                [-0.00317, -0.00247],  # 2번 점 매핑
                [-2.20023, -0.35673],  # 3번 점 매핑
                [-2.65047, 2.09635]    # 4번 점 매핑
            ], dtype=np.float32)
            
            print(f"✅ [Cam {self.cam_id}] Left 카메라 호모그래피 로드됨 (ID 0 데이터)")

        elif self.cam_id == CAM_RIGHT_ID: 
            # 우측 카메라 (기존 Left 데이터였던 ID 4번 데이터 적용)
            # webcam_debug_right.py 데이터
            pixel_pts = np.array([
                [455, 95],     # 1번 점 (좌상)
                [819, 91],     # 2번 점 (우상)
                [1225, 658],   # 3번 점 (우하)
                [45, 647]      # 4번 점 (좌하)
            ], dtype=np.float32)

            map_pts = np.array([
                [2.85043, -0.64341],   # 1번 점 매핑
                [3.27524, -3.79587],   # 2번 점 매핑
                [-1.63256, -4.55022],  # 3번 점 매핑
                [-1.99700, -1.62762]   # 4번 점 매핑
            ], dtype=np.float32)
            
            print(f"✅ [Cam {self.cam_id}] Right 카메라 호모그래피 로드됨 (ID 4 데이터)")
            
        else:
            print(f"⚠️ [Cam {self.cam_id}] 알 수 없는 카메라 ID. 변환 행렬 없음.")
            return

        # 2. 행렬 계산
        self.H, _ = cv2.findHomography(pixel_pts, map_pts)

    def pixel_to_map(self, u, v):
        """ 픽셀(u, v) -> 맵(x, y) 변환 """
        if self.H is None: return 0.0, 0.0
        pixel_pt = np.array([[[u, v]]], dtype=np.float32)
        map_pt = cv2.perspectiveTransform(pixel_pt, self.H)
        return map_pt[0][0][0], map_pt[0][0][1]


# =========================================================
# [Core 2] 비전 시스템 (YOLO + Camera Thread)
# =========================================================
class VisionSystem(threading.Thread):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.running = True
        self.daemon = True # 메인 종료 시 자동 종료
        
        # 모델 로드
        print(f"🚀 YOLO 모델 로딩 중... : {YOLO_MODEL_PATH}")
        try:
            self.model = YOLO(YOLO_MODEL_PATH)
        except Exception as e:
            print(f"❌ 모델 로드 실패: {e}")
            self.running = False
            
        # 변환기 초기화
        self.converter_left = HomographyConverter(CAM_LEFT_ID)
        self.converter_right = HomographyConverter(CAM_RIGHT_ID)

    def init_camera(self, index):
        cap = cv2.VideoCapture(index)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)
        if not cap.isOpened():
            print(f"❌ Error: {index}번 카메라 오픈 실패")
            return None
        return cap

    def run(self):
        global global_frame_left, global_frame_right, camera_status
        
        # 1. 카메라 연결
        cap_l = self.init_camera(CAM_LEFT_ID)
        cap_r = self.init_camera(CAM_RIGHT_ID)
        
        while self.running:
            # 프레임 읽기
            ret_l, frame_l = cap_l.read() if cap_l else (False, None)
            ret_r, frame_r = cap_r.read() if cap_r else (False, None)
            
            # 상태 업데이트
            camera_status[1] = ret_l # GUI용 (Left)
            camera_status[2] = ret_r # GUI용 (Right)

            if not ret_l and not ret_r:
                time.sleep(1) # 둘 다 없으면 대기
                continue

            # 2. YOLO 추론 (배치 처리로 속도 최적화)
            frames_to_process = []
            if ret_l: frames_to_process.append(frame_l)
            if ret_r: frames_to_process.append(frame_r)
            
            if frames_to_process:
                # YOLO 실행
                results = self.model(frames_to_process, conf=CONF_THRESHOLD, verbose=False)
                
                # 결과 처리
                idx = 0
                if ret_l:
                    self.process_detection(results[idx], frame_l, "LEFT", self.converter_left)
                    with frame_lock: global_frame_left = frame_l.copy()
                    idx += 1
                if ret_r:
                    self.process_detection(results[idx], frame_r, "RIGHT", self.converter_right)
                    with frame_lock: global_frame_right = frame_r.copy()

            # CPU 과부하 방지 (적절히 조절)
            time.sleep(0.01)

        if cap_l: cap_l.release()
        if cap_r: cap_r.release()

    def process_detection(self, result, frame, cam_name, converter):
        """ 감지된 객체 분석, 그리기, ROS 퍼블리싱 """
        
        patient_center = None
        responder_center = None
        
        # Boxes 순회
        for box in result.boxes:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            
            # 1. 그리기 (Bounding Box)
            label = f"{self.model.names[cls_id]} {conf:.2f}"
            color = (0, 255, 0) if cls_id == CLASS_ID_RESPONDER else (0, 0, 255) # Responder=Green, Patient=Red
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            # 2. 로직 처리
            # (A) 환자 감지 시 -> 좌표 변환 및 퍼블리시
            if cls_id == CLASS_ID_PATIENT:
                patient_center = (cx, cy)
                map_x, map_y = converter.pixel_to_map(cx, cy)
                
                # 좌표 그리기
                coord_text = f"Map: ({map_x:.2f}, {map_y:.2f})"
                cv2.putText(frame, coord_text, (x1, y2 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                
                # ROS Publish
                if self.ros_node:
                    self.ros_node.publish_target_pose(map_x, map_y)

            # (B) 구조자 감지 시
            elif cls_id == CLASS_ID_RESPONDER:
                responder_center = (cx, cy)

        # 3. 도착 판정 (환자와 구조자가 동시에 화면에 있고, 거리가 가까울 때)
        if patient_center and responder_center:
            dist = math.sqrt((patient_center[0]-responder_center[0])**2 + (patient_center[1]-responder_center[1])**2)
            
            # 거리 표시
            cv2.line(frame, patient_center, responder_center, (255, 255, 255), 2)
            cv2.putText(frame, f"Dist: {dist:.1f}px", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            
            if dist <= DIST_THRESHOLD:
                cv2.putText(frame, "!!! ARRIVED !!!", (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 3)
                if self.ros_node:
                    self.ros_node.publish_arrival_status(True)


# =========================================================
# [Core 3] ROS2 노드 (Control Tower)
# =========================================================
class ControlTowerNode(Node):
    def __init__(self):
        super().__init__('subway_control_tower')
        
        # --- [Publishers] ---
        # 1. UI 클릭 목표 지점
        self.pub_goal = self.create_publisher(Point, '/control/goal_point', 10)
        # 2. 작업 종료 신호
        self.pub_task_end = self.create_publisher(Bool, '/control/task_end', 10)
        
        # [NEW] 3. 환자 감지 좌표 퍼블리시
        self.pub_target_pose = self.create_publisher(PoseStamped, '/target', 10)
        # [NEW] 4. 구조대 도착 여부 퍼블리시
        self.pub_arrival = self.create_publisher(Bool, '/emt_arrival_status', 10)

        # --- [Subscribers] ---
        self.create_subscription(String, '/system/robot_status', self.cb_robot_status, 10)
        self.create_subscription(String, '/system/alert', self.cb_emergency, 10)

        # UI 클릭용 호모그래피는 YOLO쪽 변환기를 재사용하거나 간단히 0번(Right) 기준 등으로 고정
        # 여기서는 VisionSystem이 있으므로 그쪽 변환 로직을 타거나, 단순화를 위해 별도 유지
        # (기존 코드는 유지하되, VisionSystem의 데이터 활용 권장)

    def cb_robot_status(self, msg):
        global robots_data
        try:
            data = json.loads(msg.data)
            for r_id, r_info in data.items():
                if r_id in robots_data: robots_data[r_id] = r_info
        except: pass 

    def cb_emergency(self, msg):
        save_log('emergency_history', msg.data)

    def send_goal_command(self, x, y, cam_id):
        """ UI 클릭 -> VisionSystem 변환기 사용 -> 토픽 발행 """
        # 편의상 여기서 간단히 Point 메시지로 보냄 (z=0)
        # 실제로는 cam_id에 따라 vision_system.converter_left/right 를 써야 함.
        # 이 부분은 VisionSystem 인스턴스가 전역으로 필요함.
        pass # 아래 click_event API에서 처리하도록 구조 변경

    def send_task_end_signal(self):
        msg = Bool(); msg.data = True
        self.pub_task_end.publish(msg)
        save_log('robot_logs', "명령 전송: 작업 종료 (Task End)")

    # [NEW] 환자 위치 전송 함수
    def publish_target_pose(self, map_x, map_y):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(map_x)
        msg.pose.position.y = float(map_y)
        msg.pose.position.z = 0.0
        # 방향(Orientation)은 알 수 없으므로 Identity(0,0,0,1) 유지
        msg.pose.orientation.w = 1.0
        
        self.pub_target_pose.publish(msg)
        # 너무 잦은 로그 방지를 위해 print는 생략하거나 조건부 출력

    # [NEW] 도착 신호 전송 함수
    def publish_arrival_status(self, arrived):
        msg = Bool()
        msg.data = arrived
        self.pub_arrival.publish(msg)
        print(">>> [알림] 구조대원 도착 확인! (Distance Condition Met)")


# =========================================================
# [Core 4] Flask 라우팅 및 유틸
# =========================================================
@app.route('/')
def home():
    if 'user' in session: return redirect(url_for('dashboard'))
    return redirect(url_for('login_page'))

@app.route('/login', methods=['GET', 'POST'])
def login_page():
    if request.method == 'POST':
        username = request.form.get('username')
        password = request.form.get('password')
        conn = sqlite3.connect(DB_NAME)
        c = conn.cursor()
        c.execute("SELECT * FROM users WHERE username = ? AND password = ?", (username, password))
        if c.fetchone():
            session['user'] = username; conn.close()
            return redirect(url_for('dashboard'))
        conn.close()
        flash("❌ 로그인 실패")
    return render_template('login_center.html')

@app.route('/dashboard')
def dashboard():
    if 'user' not in session: return redirect(url_for('login_page'))
    return render_template('sysmon.html', username=session['user'])

@app.route('/logout')
def logout():
    session.pop('user', None)
    return redirect(url_for('login_page'))

@app.route('/api/status')
def get_status_api():
    conn = sqlite3.connect(DB_NAME)
    c = conn.cursor()
    c.execute("SELECT * FROM emergency_history ORDER BY id DESC LIMIT 50")
    emer_logs = c.fetchall()
    c.execute("SELECT * FROM robot_logs ORDER BY id DESC LIMIT 50")
    sys_logs = c.fetchall()
    conn.close()
    return jsonify({"robots": robots_data, "logs": {"emergency": emer_logs, "system": sys_logs}, "cam_status": camera_status})

@app.route('/api/click', methods=['POST'])
def click_event():
    # UI에서 클릭 시 해당 좌표를 map 좌표로 변환해 이동 명령 내리는 부분
    # vision_system 객체에 접근 필요
    data = request.json
    cam_id = data.get('id') # 1 or 2 (HTML 기준)
    u, v = data.get('x'), data.get('y')
    
    # HTML ID -> Real Camera ID 매핑
    real_cam_id = CAM_LEFT_ID if cam_id == 1 else CAM_RIGHT_ID
    
    # 변환기 선택
    if vision_system:
        converter = vision_system.converter_left if cam_id == 1 else vision_system.converter_right
        rx, ry = converter.pixel_to_map(u, v)
        
        # ROS로 목표 발행
        msg = Point(); msg.x = float(rx); msg.y = float(ry)
        ros_node.pub_goal.publish(msg)
        
        log = f"클릭 이동 명령: Cam{cam_id}({u},{v}) -> Map({rx:.2f}, {ry:.2f})"
        save_log('robot_logs', log)
        print(log)
        
    return jsonify({"status": "success"})

@app.route('/api/task_end', methods=['POST'])
def task_end_event():
    ros_node.send_task_end_signal()
    return jsonify({"status": "success"})

# --- [영상 스트리밍 제너레이터] ---
# VisionSystem이 업데이트한 global_frame을 가져와 송출
def generate_mjpeg(cam_type):
    global global_frame_left, global_frame_right
    
    while True:
        frame = None
        with frame_lock:
            if cam_type == 'LEFT': frame = global_frame_left
            elif cam_type == 'RIGHT': frame = global_frame_right
        
        if frame is None:
            # 대기 화면 (검은색)
            img = np.zeros((CAM_HEIGHT, CAM_WIDTH, 3), dtype=np.uint8)
            cv2.putText(img, "WAITING FOR CAM...", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
            _, buf = cv2.imencode('.jpg', img)
        else:
            # YOLO가 그려진 프레임을 압축
            # 전송 대역폭 절약을 위해 리사이즈 가능하지만, 요청대로 원본 유지
            _, buf = cv2.imencode('.jpg', frame)
            
        yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + buf.tobytes() + b'\r\n')
        time.sleep(0.033) # 약 30FPS 제한

@app.route('/video/1') # HTML에서 CAM 1 (Left)
def video_feed_1(): return Response(generate_mjpeg('LEFT'), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/video/2') # HTML에서 CAM 2 (Right)
def video_feed_2(): return Response(generate_mjpeg('RIGHT'), mimetype='multipart/x-mixed-replace; boundary=frame')

# [통계 페이지] 화면 렌더링
@app.route('/analytics')
def analytics():
    # 로그인 확인
    if 'user' not in session: return redirect(url_for('login_page'))
    return render_template('analytics.html', username=session['user'])

# [API] 통계 데이터 (차트용 JSON 반환)
@app.route('/api/analytics/data')
def get_analytics_data():
    conn = sqlite3.connect(DB_NAME)
    c = conn.cursor()
    
    # Robot 3 로그 개수 조회
    c.execute("SELECT count(*) FROM robot_logs WHERE content LIKE '%Robot 3%'")
    r3_count = c.fetchone()[0]
    
    # Robot 5 로그 개수 조회
    c.execute("SELECT count(*) FROM robot_logs WHERE content LIKE '%Robot 5%'")
    r5_count = c.fetchone()[0]
    
    # 시간대별 로그 발생량 조회 (최근 10개 시간대)
    c.execute("SELECT substr(timestamp, 12, 2) as hour, count(*) FROM robot_logs GROUP BY hour ORDER BY hour DESC LIMIT 10")
    time_rows = c.fetchall()
    
    hours = []; counts = []
    for row in time_rows: 
        hours.append(f"{row[0]}시")
        counts.append(row[1])
    
    # 차트 순서를 위해 역정렬
    hours.reverse()
    counts.reverse()
    
    # 최근 로그 100개 조회
    c.execute("SELECT * FROM robot_logs ORDER BY id DESC LIMIT 100")
    logs = c.fetchall()
    
    conn.close()
    
    # JSON 데이터 반환
    return jsonify({
        "pie_data": [r3_count, r5_count], 
        "line_data": {"labels": hours, "values": counts}, 
        "logs": logs
    })


# =========================================================
# [Main Execution]
# =========================================================
ros_node = None
vision_system = None

def ros_thread_job():
    rclpy.spin(ros_node)

def main(args=None):
    global ros_node, vision_system
    
    # 1. 초기화
    init_db()
    rclpy.init(args=args)
    
    # 2. 노드 및 비전 시스템 생성
    ros_node = ControlTowerNode()
    vision_system = VisionSystem(ros_node)
    
    # 3. 스레드 시작
    # (A) ROS2 Spin 스레드
    t_ros = threading.Thread(target=ros_thread_job, daemon=True)
    t_ros.start()
    
    # (B) Vision System (YOLO) 스레드
    vision_system.start()
    
    # 4. 브라우저 자동 실행
    def open_browser():
        if not os.environ.get("WERKZEUG_RUN_MAIN"): 
            webbrowser.open_new('http://localhost:5000')
    Timer(1.5, open_browser).start()
    
    print(">>> [System] 지하철 안전 관제 시스템 (YOLO Integrated) 가동 시작...")
    
    # 5. Flask 서버 실행 (Main Thread 점유)
    try:
        app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)
    except KeyboardInterrupt:
        pass
    finally:
        print(">>> [System] 종료 중...")
        vision_system.running = False
        vision_system.join()
        ros_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()