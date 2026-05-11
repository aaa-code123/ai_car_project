import cv2
import json
import os
import time
import logging
import threading
import traceback
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

# ---- 紅綠燈/物件偵測 ----
try:
    import traffic_light_detection as tld
    TL_AVAILABLE = tld.load_model()
except Exception as e:
    print(f"[警告] 紅綠燈偵測模組載入失敗: {e}")
    TL_AVAILABLE = False

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
show_tl_detect    = True     # 是否在串流上顯示紅綠燈偵測框
tl_enforce        = True     # 是否執行紅燈停車

# ---- 紅綠燈狀態（inference_thread 寫，lane_follow_thread 讀）----
tl_state            = None   # 'red' | 'green' | 'yellow' | None
tl_no_detect_frames = 0      # 連續幾幀無穩定偵測（inference_thread 專屬）
TL_NO_DETECT_RESUME = 10     # 達到此幀數後才解除停車狀態
tl_lock             = threading.Lock()

# ---- 從 settings.json 還原上次設定 ----
_s = _load_settings()
if _s:
    auto_base_speed   = _s.get('speed',         auto_base_speed)
    auto_fps          = _s.get('fps',            auto_fps)
    auto_kp           = _s.get('kp',             auto_kp)
    auto_correct_skip = _s.get('correct_skip',   auto_correct_skip)
    show_lane_debug   = _s.get('show_debug',     show_lane_debug)
    show_tl_detect    = _s.get('show_tl_detect', show_tl_detect)
    tl_enforce        = _s.get('tl_enforce',     tl_enforce)
    _tl = {k: _s[k] for k in ('tl_conf', 'tl_conf_red', 'tl_conf_yellow', 'tl_conf_green', 'tl_iou', 'tl_smooth_win', 'tl_smooth_hits', 'tl_min_area_ratio') if k in _s}
    if _tl and TL_AVAILABLE:
        tld.set_params(_tl)
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
annotated_frame = None  # 最終顯示用（已標注）
raw_frame       = None  # 相機原始幀（inference_thread 寫，其他執行緒讀）
raw_frame_lock  = threading.Lock()

# ---- Watchdog：追蹤各執行緒最後活躍時間 ----
_thread_heartbeat = {
    'capture':   0.0,
    'inference': 0.0,
    'lane_follow': 0.0,
}
_WATCHDOG_TIMEOUT = 10.0  # 超過 10 秒沒心跳就印警告

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


def _update_tl_state(detection):
    """inference_thread 專用：根據最新偵測結果更新 tl_state"""
    global tl_state, tl_no_detect_frames
    with tl_lock:
        if detection is not None:
            tl_state = detection['label'].lower()
            tl_no_detect_frames = 0
        else:
            tl_no_detect_frames += 1
            if tl_no_detect_frames >= TL_NO_DETECT_RESUME:
                tl_state = None
            # 未達門檻：維持上一個狀態，避免短暫遺失就解除停車


def _draw_tl_stop_overlay(img):
    """在畫面頂部畫紅色半透明 banner，標示紅燈停車"""
    h, w = img.shape[:2]
    overlay = img.copy()
    cv2.rectangle(overlay, (0, 0), (w, 50), (0, 0, 160), -1)
    cv2.addWeighted(overlay, 0.6, img, 0.4, 0, img)
    cv2.putText(img, "STOPPED - RED LIGHT", (10, 35),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2, cv2.LINE_AA)


# ---- Watchdog 執行緒：定期檢查各執行緒是否還活著 ----
def watchdog_thread():
    time.sleep(5)  # 等啟動完成再開始監控
    while True:
        now = time.time()
        for name, last in list(_thread_heartbeat.items()):
            if last == 0.0:
                continue  # 還沒啟動
            elapsed = now - last
            if elapsed > _WATCHDOG_TIMEOUT:
                print(f"[WATCHDOG] ⚠️  {name} 已 {elapsed:.1f}s 無心跳！可能卡死")
                # 印出所有執行緒目前狀態
                for t in threading.enumerate():
                    print(f"[WATCHDOG]   thread={t.name} alive={t.is_alive()} daemon={t.daemon}")
                # 印出所有 lock 狀態
                print(f"[WATCHDOG]   frame_lock locked={frame_lock.locked()}")
                print(f"[WATCHDOG]   i2c_lock   locked={i2c_lock.locked()}")
                print(f"[WATCHDOG]   raw_frame_lock locked={raw_frame_lock.locked()}")
        time.sleep(3)

threading.Thread(target=watchdog_thread, daemon=True, name='watchdog').start()


# ---- 背景執行緒：擷取影像 ----
# 只負責從相機抓幀並存到 raw_frame，不做任何耗時運算
def capture_thread():
    global raw_frame
    print("[capture_thread] 啟動")
    loop_count = 0
    consecutive_errors = 0
    while True:
        t0 = time.time()
        try:
            frame = picam2.capture_array()
            dt = time.time() - t0
            if dt > 0.5:
                print(f"[capture_thread] ⚠️  capture_array 耗時 {dt:.3f}s（相機可能過忙）")
            frame = cv2.flip(frame, -1)
            with raw_frame_lock:
                raw_frame = frame
            _thread_heartbeat['capture'] = time.time()
            loop_count += 1
            consecutive_errors = 0
            if loop_count % 100 == 0:
                print(f"[capture_thread] 已擷取 {loop_count} 幀")
            time.sleep(0.033)
        except Exception as e:
            consecutive_errors += 1
            print(f"[ERROR] capture_thread (連續失敗 {consecutive_errors} 次): {e}")
            traceback.print_exc()
            # 連續失敗超過 10 次，等久一點避免刷屏
            time.sleep(0.5 if consecutive_errors > 10 else 0.1)

threading.Thread(target=capture_thread, daemon=True, name='capture').start()


# ---- 背景執行緒：推論 + 標注（手動模式專用）----
# AUTO 模式下此執行緒直接休眠，所有推論由 lane_follow_thread 統一執行
# 避免兩個執行緒同時跑 detect_lane/detect_objects 搶 CPU 造成卡死
def inference_thread():
    global current_status, annotated_frame
    print("[inference_thread] 啟動")
    loop_count = 0
    while True:
        t0 = time.time()
        try:
            _thread_heartbeat['inference'] = time.time()

            # AUTO 模式：交給 lane_follow_thread，這裡只休眠保持心跳
            if auto_mode:
                time.sleep(0.1)
                continue

            # 手動模式：車道偵測 + 物件偵測
            with raw_frame_lock:
                frame = raw_frame
            if frame is None:
                time.sleep(0.05)
                continue

            loop_count += 1
            if loop_count % 50 == 0:
                print(f"[inference_thread] MANUAL loop={loop_count}")

            # 耗時運算全在 lock 外
            if LANE_AVAILABLE and show_lane_debug:
                t1 = time.time()
                e, debug_img = detect_lane(frame)
                dt_lane = time.time() - t1
                if dt_lane > 0.5:
                    print(f"[inference_thread] ⚠️  detect_lane 耗時 {dt_lane:.3f}s")
                current_status = f"偵測中 e={e}px"
                overlay_params(debug_img, e)
            else:
                e = None
                debug_img = frame.copy()

            if TL_AVAILABLE and show_tl_detect:
                t2 = time.time()
                # detect_objects 直接在 debug_img 上畫框（不再內部 copy）
                debug_img, detection = tld.detect_objects(debug_img)
                dt_tl = time.time() - t2
                if dt_tl > 0.5:
                    print(f"[inference_thread] ⚠️  detect_objects 耗時 {dt_tl:.3f}s")
                if detection:
                    current_status = f"偵測: {detection['label']} {detection['conf']:.2f}"
            else:
                detection = None
            _update_tl_state(detection)

            with frame_lock:
                annotated_frame = debug_img

            total = time.time() - t0
            if total > 1.0:
                print(f"[inference_thread] ⚠️  整幀處理耗時 {total:.3f}s")

            time.sleep(0.1)

        except Exception as e:
            print(f"[ERROR] inference_thread loop={loop_count}: {e}")
            traceback.print_exc()
            time.sleep(0.1)

threading.Thread(target=inference_thread, daemon=True, name='inference').start()


# ---- 背景執行緒：自動循線 ----
# AUTO 模式下負責所有推論（detect_lane + detect_objects），inference_thread 此時休眠
def lane_follow_thread():
    global auto_mode, current_status, current_speed, annotated_frame
    correct_counter = 0
    loop_count = 0
    print("[lane_follow_thread] 啟動")

    while True:
        t0 = time.time()
        try:
            _thread_heartbeat['lane_follow'] = time.time()

            if not auto_mode or not LANE_AVAILABLE or not MOTOR_AVAILABLE:
                time.sleep(0.05)
                correct_counter = 0
                continue

            loop_interval = 1.0 / max(1, auto_fps)
            loop_count += 1

            # 取一幀（快速，lock 時間極短）
            with raw_frame_lock:
                frame = raw_frame
            if frame is None:
                print(f"[lane_follow_thread] raw_frame 為 None，等待")
                time.sleep(loop_interval)
                continue

            # ---- 所有耗時推論都在 lock 外 ----
            t1 = time.time()
            e, debug_img = detect_lane(frame)
            dt_lane = time.time() - t1
            if dt_lane > 0.5:
                print(f"[lane_follow_thread] ⚠️  detect_lane 耗時 {dt_lane:.3f}s")

            # TL 偵測（AUTO 模式在此執行，inference_thread 休眠中）
            detection = None
            if TL_AVAILABLE:
                t2 = time.time()
                if show_tl_detect:
                    # 直接在 debug_img 上畫框，省去一次 copy
                    debug_img, detection = tld.detect_objects(debug_img)
                else:
                    # 只需推論結果，不改影像；傳入小型 copy 避免污染 debug_img
                    _, detection = tld.detect_objects(frame)
                dt_tl = time.time() - t2
                if dt_tl > 0.5:
                    print(f"[lane_follow_thread] ⚠️  detect_objects 耗時 {dt_tl:.3f}s")
                if detection:
                    print(f"[TL] label={detection['label']} conf={detection['conf']:.2f} area={detection.get('area_ratio',0)*100:.1f}%") if loop_count % 10 == 0 else None
            _update_tl_state(detection)

            if loop_count % 30 == 0:
                print(f"[lane_follow_thread] loop={loop_count} e={e} tl={tl_state} dt={time.time()-t0:.3f}s")

            # ---- 紅燈停車 ----
            with tl_lock:
                _tl = tl_state
            if tl_enforce and _tl == 'red':
                if MOTOR_AVAILABLE:
                    with i2c_lock:
                        motor.stop()
                current_status = "STOPPED - RED LIGHT"
                _draw_tl_stop_overlay(debug_img)
                with frame_lock:
                    annotated_frame = debug_img
                time.sleep(loop_interval)
                continue

            # ---- 無車道 → 停車 ----
            if e is None:
                if MOTOR_AVAILABLE:
                    with i2c_lock:
                        motor.stop()
                current_status = "no lane - stopped"
                overlay_params(debug_img, 0)
                with frame_lock:
                    annotated_frame = debug_img
                time.sleep(loop_interval)
                continue

            # ---- 正常循線：先算出 l, r，再一次性更新畫面 ----
            l, r = auto_base_speed, auto_base_speed  # 預設值，correct_counter 未到時用

            correct_counter += 1
            if correct_counter >= auto_correct_skip:
                correct_counter = 0
                pid.kp = auto_kp
                out    = pid.compute(e)
                l, r   = pid_to_speeds(out, base_speed=auto_base_speed)

                t3 = time.time()
                with i2c_lock:
                    dt_lock = time.time() - t3
                    if dt_lock > 0.3:
                        print(f"[lane_follow_thread] ⚠️  等待 i2c_lock 耗時 {dt_lock:.3f}s")
                    if not auto_mode:
                        # auto_mode 在等鎖期間被關閉
                        motor.stop()
                        time.sleep(loop_interval)
                        continue

                    motor.pwm.setDutycycle(motor.CHANNELS['A_PWM'], l)
                    motor.pwm.setLevel(motor.CHANNELS['A_IN1'], 0)
                    motor.pwm.setLevel(motor.CHANNELS['A_IN2'], 1)

                    motor.pwm.setDutycycle(motor.CHANNELS['C_PWM'], l)
                    motor.pwm.setLevel(motor.CHANNELS['C_IN1'], 1)
                    motor.pwm.setLevel(motor.CHANNELS['C_IN2'], 0)

                    motor.pwm.setDutycycle(motor.CHANNELS['B_PWM'], r)
                    motor.pwm.setLevel(motor.CHANNELS['B_IN1'], 1)
                    motor.pwm.setLevel(motor.CHANNELS['B_IN2'], 0)

                    motor.pwm.setDutycycle(motor.CHANNELS['D_PWM'], r)
                    motor.motorD1.off()
                    motor.motorD2.on()

                    current_speed = (l + r) // 2

                current_status = f"e={e}px Kp={auto_kp} L={l} R={r}"
                print(f"[LANE] e={e:4d} kp={auto_kp} out={out:6.1f} L={l:3d} R={r:3d}")

            # 畫面更新（只寫一次 annotated_frame，避免和其他執行緒競爭）
            overlay_params(debug_img, e)
            with frame_lock:
                annotated_frame = debug_img

            total = time.time() - t0
            if total > 1.0:
                print(f"[lane_follow_thread] ⚠️  整幀耗時 {total:.3f}s")

            time.sleep(max(0, loop_interval - (time.time() - t0)))

        except Exception as ex:
            print(f"[ERROR] lane_follow_thread loop={loop_count}: {ex}")
            traceback.print_exc()
            time.sleep(0.1)

threading.Thread(target=lane_follow_thread, daemon=True, name='lane_follow').start()


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
    d = {
        'speed':          auto_base_speed,
        'fps':            auto_fps,
        'kp':             auto_kp,
        'correct_skip':   auto_correct_skip,
        'show_debug':     show_lane_debug,
        'show_tl_detect': show_tl_detect,
        'tl_enforce':     tl_enforce,
        'roi':            get_roi_ratio(),
        **get_lane_params(),
    }
    if TL_AVAILABLE:
        d.update(tld.get_params())
    return d


@app.route('/settings', methods=['POST'])
def settings():
    """前端動態調整自動循線參數"""
    global auto_base_speed, auto_fps, auto_kp, auto_correct_skip, show_lane_debug, show_tl_detect, tl_enforce
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
    if 'show_tl_detect' in data:
        show_tl_detect = bool(data['show_tl_detect'])
    if 'tl_enforce' in data:
        tl_enforce = bool(data['tl_enforce'])

    # 車道偵測參數
    lane_keys = {'white_s_min', 'white_s_max', 'white_v_min', 'white_v_max', 'hough_threshold', 'hough_min_len', 'hough_max_gap', 'seg_min_len', 'lane_half_width', 'single_target_ratio', 'horizontal_slope', 'single_e_clamp', 'overlay_alpha'}
    lane_data = {k: data[k] for k in lane_keys if k in data}
    if lane_data:
        set_lane_params(lane_data)

    # 紅綠燈偵測參數
    tl_data = {k: data[k] for k in ('tl_conf', 'tl_conf_red', 'tl_conf_yellow', 'tl_conf_green', 'tl_iou', 'tl_smooth_win', 'tl_smooth_hits', 'tl_min_area_ratio') if k in data}
    if tl_data and TL_AVAILABLE:
        tld.set_params(tl_data)

    _save_settings(_collect_settings())
    print(f"[SETTINGS] speed={auto_base_speed} fps={auto_fps} kp={auto_kp} skip={auto_correct_skip} debug={show_lane_debug} tl={show_tl_detect}")
    resp = {
        'speed':          auto_base_speed,
        'fps':            auto_fps,
        'kp':             auto_kp,
        'correct_skip':   auto_correct_skip,
        'show_debug':     show_lane_debug,
        'show_tl_detect': show_tl_detect,
        **get_lane_params(),
    }
    if TL_AVAILABLE:
        resp.update(tld.get_params())
    return jsonify(resp)


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
        'tl_available': TL_AVAILABLE,
        'tl_state':     tl_state,
        'settings': {
            'speed':          auto_base_speed,
            'fps':            auto_fps,
            'kp':             auto_kp,
            'correct_skip':   auto_correct_skip,
            'show_debug':     show_lane_debug,
            'show_tl_detect': show_tl_detect,
            'tl_enforce':     tl_enforce,
            **get_lane_params(),
            **(tld.get_params() if TL_AVAILABLE else {}),
        }
    })


def gen_frames():
    none_count  = 0
    frame_count = 0
    print("[gen_frames] 串流開始")
    while True:
        with frame_lock:
            frame = annotated_frame
        if frame is None:
            none_count += 1
            if none_count % 20 == 1:
                print(f"[gen_frames] annotated_frame 尚未就緒（none_count={none_count}）"
                      f"  raw_frame={'有' if raw_frame is not None else '無'}")
            time.sleep(0.05)
            continue
        none_count = 0
        ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if not ret:
            print("[gen_frames] ⚠️  imencode 失敗")
            time.sleep(0.05)
            continue
        frame_count += 1
        if frame_count % 100 == 0:
            print(f"[gen_frames] 已串流 {frame_count} 幀")
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
        time.sleep(0.033)


@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, threaded=True)
