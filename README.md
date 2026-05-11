# AI 自駕小車專案

部署在 Raspberry Pi 上的 AI 自駕小車，整合 YOLOv8 紅綠燈辨識與 OpenCV 車道偵測，透過網頁介面提供手動遙控與自動巡線雙模式。

---

## 功能特色

- **手動模式**：網頁 WASD 按鈕或鍵盤遙控，支援攝影機雲台 Pan/Tilt 控制
- **自動巡線模式**：PID 控制器依即時車道偏差調整左右馬達，實現循跡行駛
- **紅綠燈辨識**：YOLOv8 模型偵測紅燈並自動停車（可開關）
- **即時串流**：MJPEG 影像串流顯示偵測結果與疊加資訊
- **動態調參**：網頁 UI 即時調整速度、PID 增益、HSV 閾值、ROI 多邊形，設定自動持久化

---

## 硬體需求

| 元件 | 說明 |
|------|------|
| Raspberry Pi | 主控板，需支援 PiCamera2 |
| PiCamera2 | 640×480 RGB888，畫面垂直翻轉 |
| PCA9685 | I2C PWM 控制器，位址 `0x40`，頻率 50Hz |
| DC 馬達 × 4 | A/B/C/D 四輪，透過 PCA9685 + GPIO 控制 |
| 伺服機 × 2 | 攝影機雲台 Pan（ch.9）/ Tilt（ch.10）|

### PCA9685 通道對應

| 通道 | 功能 |
|------|------|
| 0 | 馬達 A PWM |
| 1 | 馬達 A IN2 |
| 2 | 馬達 A IN1 |
| 3 | 馬達 B IN1 |
| 4 | 馬達 B IN2 |
| 5 | 馬達 B PWM |
| 6 | 馬達 C PWM |
| 7 | 馬達 C IN2 |
| 8 | 馬達 C IN1 |
| 9 | 伺服機 Pan |
| 10 | 伺服機 Tilt |
| 11 | 馬達 D PWM |

> 馬達 D 方向由 GPIO 25（motorD1）與 GPIO 24（motorD2）控制，非 PCA9685 通道。

---

## 安裝

### 1. 安裝 Python 依賴

```bash
pip install flask opencv-python ultralytics smbus2 psutil
```

> `picamera2` 與 `gpiozero` 為 Raspberry Pi 系統內建，通常不需另行安裝。

### 2. 啟動應用程式

```bash
python app.py
```

### 3. 開啟控制介面

用任意裝置的瀏覽器連到：

```
http://<Pi-IP>:5000
```

---

## 專案結構

```
ai_car_project/
├── app.py                    # Flask 主程式（路由、執行緒、全域狀態）
├── lane_detection.py         # 車道偵測（OpenCV HSV + HoughLinesP）
├── motor_control.py          # PCA9685 PWM 馬達控制器
├── pid_controller.py         # PID 控制器（計算轉向修正量）
├── traffic_light_detection.py# YOLOv8 紅綠燈偵測
├── best.pt                   # YOLOv8 預訓練模型
├── settings.json             # 持久化參數設定
└── templates/
    └── index.html            # 網頁控制介面
```

---

## 核心模組

### `app.py` — 主程式

多執行緒架構：

| 執行緒 | 功能 |
|--------|------|
| `capture_thread` | 持續從 PiCamera2 擷取畫面 |
| `inference_thread` | 手動模式：每 100ms 執行車道 + 紅綠燈偵測 |
| `lane_follow_thread` | 自動模式：車道偵測 → PID → 馬達指令 |
| `watchdog_thread` | 監控各執行緒心跳，防止死鎖 |

### `lane_detection.py` — 車道偵測

偵測流程：`BGR → HSV 白色遮罩 → ROI 裁切 → 形態學閉合 → HoughLinesP → 斜率分類 → EMA 學習車道寬度 → 輸出偏差值 e`

- `e > 0`：車道偏右；`e < 0`：車道偏左
- 雙線可見時自動學習 `LANE_HALF_WIDTH`
- 單線可見時依學到的車道寬推算中心

### `pid_controller.py` — PID 控制器

```python
PIDController(kp=0.2, ki=0.0, kd=0.0)  # 目前僅用比例控制
```

輸出左右輪速：`left = base_speed + correction`，`right = base_speed - correction`（夾至 [0, 100]）

### `motor_control.py` — 馬達控制

`MotorController` 提供 `forward()`、`backward()`、`left()`、`right()`、`stop()` 方法，含漸速啟動防止電流突波。

### `traffic_light_detection.py` — 紅綠燈偵測

- YOLOv8 模型（`best.pt`）
- 5 幀時間平滑，需連續 2 幀確認才觸發停車
- 紅燈置信度閾值較低（0.20）以確保行車安全

---

## API 端點

| 方法 | 路徑 | 說明 |
|------|------|------|
| GET | `/` | 網頁控制介面 |
| POST | `/control` | 手動馬達指令：`action=forward/backward/left/right/stop` |
| POST | `/auto` | 切換自動巡線：`action=start/stop` |
| POST | `/settings` | 動態調整參數（JSON） |
| POST | `/camera` | 雲台控制：`cam_left/right/up/down/center/release` |
| POST | `/roi` | 調整 ROI 多邊形頂點 |
| GET | `/status` | 系統狀態 JSON（CPU、記憶體、速度、模式等） |
| GET | `/video_feed` | MJPEG 即時影像串流 |

---

## 可調參數

所有參數可在網頁 UI 即時調整，並自動存入 `settings.json`：

| 參數 | 預設值 | 說明 |
|------|--------|------|
| `speed` | 20 | 自動模式基礎速度（%）|
| `fps` | 8 | 自動模式推論頻率（Hz）|
| `kp` | 0.1 | PID 比例增益 |
| `correct_skip` | 1 | 每幾幀執行一次馬達修正 |
| `tl_enforce` | true | 偵測到紅燈時自動停車 |
| `white_v_min` | 195 | 車道白色亮度下限 |
| `hough_threshold` | 17 | HoughLinesP 投票門檻 |
| `lane_half_width` | 67 | 車道半寬（像素）|
| `tl_conf` | 0.4 | 紅綠燈信心閾值 |

---

## 容錯機制

- `MOTOR_AVAILABLE = False`：馬達模組載入失敗時，程式仍可純視覺運行
- `LANE_AVAILABLE = False`：車道偵測模組載入失敗時，自動模式停用
- Watchdog 執行緒：監控各背景執行緒，防止靜默死鎖

---

## 已知限制

- 雙線皆遺失時回傳 `e=None`，馬達停止，無跨幀 tracking
- PID 僅啟用 Kp（Ki/Kd 設為 0），需要 D 控制需修改前端
- I2C 鎖在高負載下可能造成輕微控制延遲
