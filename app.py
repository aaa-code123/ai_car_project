import cv2
import json
import os
import time
import logging
import threading
import psutil
from flask import Flask, render_template, Response, request, jsonify
from picamera2 import Picamera2

SETTINGS_PATH = os.path.join(os.path.dirname(__file__), 'settings.json')

def _load_settings():
    try:
        with open(SETTINGS_PATH, 'r') as f:
            return json.load(f)
    except Exception:
        return {}

def _save_settings(data):
    try:
        with open(SETTINGS_PATH, 'w') as f:
            json.dump(data, f, indent=2)
    except Exception as e:
        print(f"[警告] 設定儲存失敗: {e}")

# ---- 馬達控制 ----
try:
    from motor_control import MotorController
    motor = MotorController()
    MOTOR_AVAILABLE = True
    print("✅ 馬達模組載入成功")
except Exception as e:
    print(f"[警告] 馬達模組載入失敗: {e}")
    MOTOR_AVAILABLE = False

# ---- 車道偵測 + PID ----
try:
    from lane_detection import detect_lane, get_roi_ratio, set_roi_ratio, get_lane_params, set_lane_params
    from pid_controller import PIDController, pid_to_speeds
    LANE_AVAILABLE = True
    print("✅ 車道偵測模組載入成功")
except Exception as e:
    print(f"[警告] 車道偵測模組載入失敗: {e}")
    LANE_AVAILABLE = False
    def get_roi_ratio(): return []
    def set_roi_ratio(pts): pass
    def get_lane_params(): return {}
    def set_lane_params(p): pass

# ---- PiCamera2 初始化 ----
picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration(
    main={"size": (640, 480), "format": "RGB888"}
))
picam2.start()
print("📷 相機啟動成功")

# ---- 全域狀態 ----
current_status  = "等待辨識..."
current_speed   = 0
pan_angle       = 90
tilt_angle      = 90
SERVO_STEP      = 5
auto_mode       = False

# ---- 可調參數（前端可動態修改）----
auto_base_speed   = 25       # 自動模式速度 0~100
auto_fps          = 20       # 自動模式推論幀率（每秒幾次）
auto_kp           = 0.2      # PID Kp（方向矯正力度）
auto_correct_skip = 1        # 每幾幀執行一次馬達修正（1=每幀都修正）
show_lane_debug   = True     # 是否在串流上顯示偵測線

# ---- 從 settings.json 還原上次設定 ----
_s = _load_settings()
if _s:
    auto_base_speed   = _s.get('speed',         auto_base_speed)
    auto_fps          = _s.get('fps',            auto_fps)
    auto_kp           = _s.get('kp',             auto_kp)
    auto_correct_skip = _s.get('correct_skip',   auto_correct_skip)
    show_lane_debug   = _s.get('show_debug',     show_lane_debug)
    _lane = {k: _s[k] for k in ('white_s_min','white_s_max','white_v_min','white_v_max',
                                  'hough_threshold','hough_min_len','hough_max_gap','seg_min_len',
                                  'lane_half_width','single_target_ratio','horizontal_slope',
                                  'single_e_clamp') if k in _s}
    if _lane and LANE_AVAILABLE:
        set_lane_params(_lane)
    _roi = _s.get('roi')
    if _roi and LANE_AVAILABLE:
        set_roi_ratio(_roi)
    print(f"[設定] 已從 settings.json 還原")

pid = PIDController(kp=auto_kp, ki=0.0, kd=0.0) if LANE_AVAILABLE else None

# ---- 舵機角度快取 ----
_last_pan  = -1
_last_tilt = -1

# ---- 執行緒鎖 ----
frame_lock = threading.Lock()
i2c_lock   = threading.Lock()

# ---- 背景推論共用資料 ----
annotated_frame = None

app = Flask(__name__)

# ---- 過濾 /status 的 access log ----
class NoStatusFilter(logging.Filter):
    def filter(self, record):
        return '/status' not in record.getMessage()

logging.getLogger('werkzeug').addFilter(NoStatusFilter())


# ---- Servo 控制 ----
def set_servo(channel, angle):
    global _last_pan, _last_tilt
    angle = max(0, min(180, angle))
    if channel == 9  and angle == _last_pan:  return angle
    if channel == 10 and angle == _last_tilt: return angle
    pulse_width_us = (angle * 11) + 500
    duty_cycle = int(4096 * pulse_width_us / 20000)
    motor.pwm.setPWM(channel, 0, duty_cycle)
    if channel == 9:  _last_pan  = angle
    if channel == 10: _last_tilt = angle
    return angle


def stop_servo(channel):
    global _last_pan, _last_tilt
    motor.pwm.setPWM(channel, 0, 0)
    if channel == 9:  _last_pan  = -1
    if channel == 10: _last_tilt = -1


# ---- Servo 初始化 ----
try:
    with i2c_lock:
        pan_angle  = set_servo(9,  pan_angle)
        tilt_angle = set_servo(10, tilt_angle)
    print("✅ Servo 初始化成功（Pan=90, Tilt=90）")
except Exception as e:
    print(f"[警告] Servo 初始化失敗: {e}")


# ---- 在畫面上疊加即時參數 ----
def overlay_params(img, e, l=None, r=None):
    h, w = img.shape[:2]
    mode_str = "AUTO" if auto_mode else "MANUAL"
    mode_color = (0, 255, 120) if auto_mode else (80, 180, 255)

    lines = [
        (f"MODE : {mode_str}",          mode_color),
        (f"e    : {e:+d} px" if e is not None else "e    : -- px", (0, 255, 255)),
        (f"Kp   : {auto_kp:.2f}",       (200, 200, 255)),
        (f"SPD  : {auto_base_speed}%",  (100, 220, 255)),
        (f"FPS  : {auto_fps}",          (180, 180, 180)),
    ]
    if l is not None and r is not None:
        lines.append((f"L={l}  R={r}", (160, 255, 160)))

    x, y0 = w - 170, 20
    for i, (text, color) in enumerate(lines):
        y = y0 + i * 24
        cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color,      1, cv2.LINE_AA)
    return img


# ---- 背景執行緒：擷取影像（隨時顯示車道偵測線）----
def inference_thread():
    global current_status, annotated_frame
    while True:
        frame = picam2.capture_array()
        frame = cv2.flip(frame, -1)

        # 自動循線模式由 lane_follow_thread 負責更新 annotated_frame，這裡只擷取原始幀
        if auto_mode:
            with frame_lock:
                annotated_frame = frame
            time.sleep(0.033)
            continue

        # 非自動循線模式：也顯示車道偵測線
        if LANE_AVAILABLE and show_lane_debug:
            e, debug_img = detect_lane(frame)
            current_status = f"偵測中 e={e}px"
            overlay_params(debug_img, e)
            with frame_lock:
                annotated_frame = debug_img
        else:
            with frame_lock:
                annotated_frame = frame

        time.sleep(0.05)

threading.Thread(target=inference_thread, daemon=True).start()


# ---- 背景執行緒：自動循線 ----
def lane_follow_thread():
    global auto_mode, current_status, current_speed, annotated_frame
    correct_counter = 0

    while True:
        if not auto_mode or not LANE_AVAILABLE or not MOTOR_AVAILABLE:
            time.sleep(0.05)
            correct_counter = 0
            continue

        loop_interval = 1.0 / max(1, auto_fps)

        with frame_lock:
            frame = annotated_frame
        if frame is None:
            time.sleep(loop_interval)
            continue

        e, debug_img = detect_lane(frame)

        # 偵測失敗（無線）→ 停車
        if e is None:
            if MOTOR_AVAILABLE:
                with i2c_lock:
                    motor.stop()
            current_status = "no lane - stopped"
            if show_lane_debug:
                overlay_params(debug_img, 0)
                with frame_lock:
                    annotated_frame = debug_img
            time.sleep(loop_interval)
            continue

        # 根據開關決定是否把偵測畫面疊回串流
        if show_lane_debug:
            overlay_params(debug_img, e)
            with frame_lock:
                annotated_frame = debug_img

        correct_counter += 1
        if correct_counter >= auto_correct_skip:
            correct_counter = 0
            with i2c_lock:
                if not auto_mode:
                    continue
                pid.kp = auto_kp        # 即時套用前端調整的 Kp
                out    = pid.compute(e)
                l, r   = pid_to_speeds(out, base_speed=auto_base_speed)

                # 左側馬達 A、C 前進
                motor.pwm.setDutycycle(motor.CHANNELS['A_PWM'], l)
                motor.pwm.setLevel(motor.CHANNELS['A_IN1'], 0)
                motor.pwm.setLevel(motor.CHANNELS['A_IN2'], 1)

                motor.pwm.setDutycycle(motor.CHANNELS['C_PWM'], l)
                motor.pwm.setLevel(motor.CHANNELS['C_IN1'], 1)
                motor.pwm.setLevel(motor.CHANNELS['C_IN2'], 0)

                # 右側馬達 B、D 前進
                motor.pwm.setDutycycle(motor.CHANNELS['B_PWM'], r)
                motor.pwm.setLevel(motor.CHANNELS['B_IN1'], 1)
                motor.pwm.setLevel(motor.CHANNELS['B_IN2'], 0)

                motor.pwm.setDutycycle(motor.CHANNELS['D_PWM'], r)
                motor.motorD1.off()
                motor.motorD2.on()

                current_speed = (l + r) // 2

            # 更新疊加（加入馬達速度）
            with frame_lock:
                if annotated_frame is not None:
                    overlay_params(annotated_frame, e, l, r)

            current_status = f"e={e}px Kp={auto_kp} L={l} R={r}"
            print(f"[LANE] e={e:4d} kp={auto_kp} out={out:6.1f} L={l:3d} R={r:3d}")

        time.sleep(loop_interval)

threading.Thread(target=lane_follow_thread, daemon=True).start()


# ---- 路由 ----
@app.route('/')
def index():
    return render_template('index.html')


@app.route('/control', methods=['POST'])
def control():
    global current_speed, auto_mode
    action = request.form.get('action')
    if action != 'stop' and auto_mode:
        auto_mode = False
        if pid: pid.reset()
        print("⚠️ 手動介入，自動循線已停止")
    print(f'收到指令: {action}')
    if MOTOR_AVAILABLE:
        with i2c_lock:
            if action == 'forward':
                motor.forward();  current_speed = motor.speed
            elif action == 'backward':
                motor.backward(); current_speed = motor.speed
            elif action == 'left':
                motor.left();     current_speed = motor.speed
            elif action == 'right':
                motor.right();    current_speed = motor.speed
            elif action == 'stop':
                motor.stop();     current_speed = 0
    return 'OK'


@app.route('/auto', methods=['POST'])
def auto():
    global auto_mode
    action = request.form.get('action')
    if action == 'start' and LANE_AVAILABLE and MOTOR_AVAILABLE:
        auto_mode = True
        pid.reset()
        print("🤖 自動循線啟動")
    elif action == 'stop':
        auto_mode = False
        if pid: pid.reset()
        if MOTOR_AVAILABLE:
            with i2c_lock:
                motor.stop()
        print("🛑 自動循線停止")
    return jsonify({'auto': auto_mode, 'lane_available': LANE_AVAILABLE})


def _collect_settings():
    """收集所有可調參數，用於持久化儲存"""
    return {
        'speed':        auto_base_speed,
        'fps':          auto_fps,
        'kp':           auto_kp,
        'correct_skip': auto_correct_skip,
        'show_debug':   show_lane_debug,
        'roi':          get_roi_ratio(),
        **get_lane_params(),
    }


@app.route('/settings', methods=['POST'])
def settings():
    """前端動態調整自動循線參數"""
    global auto_base_speed, auto_fps, auto_kp, auto_correct_skip, show_lane_debug
    data = request.get_json(force=True)

    if 'speed' in data:
        auto_base_speed = max(0, min(100, int(data['speed'])))
    if 'fps' in data:
        auto_fps = max(1, min(30, int(data['fps'])))
    if 'kp' in data:
        auto_kp = max(0.0, min(2.0, float(data['kp'])))
        if pid: pid.kp = auto_kp
    if 'correct_skip' in data:
        auto_correct_skip = max(1, min(10, int(data['correct_skip'])))
    if 'show_debug' in data:
        show_lane_debug = bool(data['show_debug'])

    # 車道偵測參數
    lane_keys = {'white_s_min', 'white_s_max', 'white_v_min', 'white_v_max', 'hough_threshold', 'hough_min_len', 'hough_max_gap', 'seg_min_len', 'lane_half_width', 'single_target_ratio', 'horizontal_slope', 'single_e_clamp', 'overlay_alpha'}
    lane_data = {k: data[k] for k in lane_keys if k in data}
    if lane_data:
        set_lane_params(lane_data)

    _save_settings(_collect_settings())
    print(f"[SETTINGS] speed={auto_base_speed} fps={auto_fps} kp={auto_kp} skip={auto_correct_skip} debug={show_lane_debug}")
    return jsonify({
        'speed':        auto_base_speed,
        'fps':          auto_fps,
        'kp':           auto_kp,
        'correct_skip': auto_correct_skip,
        'show_debug':   show_lane_debug,
        **get_lane_params(),
    })


@app.route("/camera", methods=["POST"])
def camera():
    global pan_angle, tilt_angle
    direction = request.form.get("direction")
    if MOTOR_AVAILABLE:
        with i2c_lock:
            if direction == "cam_left":
                pan_angle  = set_servo(9,  pan_angle  + SERVO_STEP)
            elif direction == "cam_right":
                pan_angle  = set_servo(9,  pan_angle  - SERVO_STEP)
            elif direction == "cam_up":
                tilt_angle = set_servo(10, tilt_angle - SERVO_STEP)
            elif direction == "cam_down":
                tilt_angle = set_servo(10, tilt_angle + SERVO_STEP)
            elif direction == "cam_center":
                pan_angle  = set_servo(9,  90)
                tilt_angle = set_servo(10, 90)
            elif direction == "cam_release":
                stop_servo(9)
                stop_servo(10)
    return jsonify({"pan": pan_angle, "tilt": tilt_angle})


@app.route('/roi', methods=['GET', 'POST'])
def roi():
    if request.method == 'GET':
        return jsonify({'roi': get_roi_ratio()})
    data = request.get_json(force=True)
    if 'roi' in data:
        pts = data['roi']
        if len(pts) == 6:
            set_roi_ratio(pts)
            _save_settings(_collect_settings())
            print(f"[ROI] 已更新 6 頂點: {pts}")
    return jsonify({'roi': get_roi_ratio()})


@app.route('/status')
def status():
    cpu = psutil.cpu_percent(interval=None)
    mem = psutil.virtual_memory().percent
    return jsonify({
        'status':       current_status,
        'speed':        current_speed,
        'pan':          pan_angle,
        'tilt':         tilt_angle,
        'auto':         auto_mode,
        'cpu':          cpu,
        'mem':          mem,
        'settings': {
            'speed':        auto_base_speed,
            'fps':          auto_fps,
            'kp':           auto_kp,
            'correct_skip': auto_correct_skip,
            'show_debug':   show_lane_debug,
            **get_lane_params(),
        }
    })


def gen_frames():
    while True:
        with frame_lock:
            frame = annotated_frame
        if frame is None:
            continue
        ret, buffer = cv2.imencode('.jpg', frame)
        if not ret:
            continue
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')


@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, threaded=True)
