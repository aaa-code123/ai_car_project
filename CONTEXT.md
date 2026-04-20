# AI Car Project — 專案背景說明（CONTEXT）

> 提供給 AI 助手或新開發者快速理解本專案的完整背景。

---

## 專案簡介

這是一個部署在 **Raspberry Pi** 上的 AI 自駕小車專案。透過網頁介面（Flask）提供遠端控制，
並整合 YOLOv8 物件偵測與 OpenCV 車道偵測，實現雙模式駕駛：

- **手動模式**：透過網頁 WASD 按鈕或鍵盤遠端操控，並可控制攝影機雲台方向。
- **自動巡線模式**：PID 控制器根據即時車道偏差自動調整左右馬達轉速，實現循跡行駛。

**執行方式：**
```bash
python app.py
# 瀏覽器開啟 http://<Pi-IP>:5000
```

---

## 檔案結構

```
ai_car_project/
├── app.py              # Flask 主程式（路由、執行緒、全域狀態）
├── lane_detection.py   # 車道偵測模組（OpenCV Hough + PID 輸入）
├── motor_control.py    # 馬達與 PCA9685 PWM 控制
├── pid_controller.py   # PID 控制器（計算轉向修正量）
├── yolo_model.pt       # YOLOv8 預訓練模型（6.3 MB）
├── lane_test.jpg       # 車道偵測測試用圖片
├── templates/
│   └── index.html      # 前端控制介面（HTML/CSS/JS）
└── static/             # 靜態資源（目前為空）
```

---

## 硬體需求

| 元件 | 說明 |
|------|------|
| Raspberry Pi | 執行主程式，需支援 PiCamera2 |
| PiCamera2 | 640×480 RGB888，畫面垂直翻轉（`cv2.flip(frame, 0)`）|
| PCA9685 | I2C PWM 控制器，位址 `0x40`，頻率 50Hz |
| DC 馬達 × 4 | A/B/C/D 四顆，透過 PCA9685 + GPIO 控制 |
| 伺服機 × 2 | 攝影機雲台 Pan（ch.9）/ Tilt（ch.10）|

---

## 硬體通道對應（PCA9685）

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
| 9 | 伺服機 Pan（左右）|
| 10 | 伺服機 Tilt（上下）|
| 11 | 馬達 D PWM |

> **馬達 D 方向**由 GPIO 25（motorD1）與 GPIO 24（motorD2）的 LED 控制，而非 PCA9685 通道。

---

## 核心模組說明

### `app.py` — 主程式

**全域可調參數：**

| 變數 | 預設值 | 說明 |
|------|--------|------|
| `auto_base_speed` | 35 | 自動模式基礎速度（0~100%）|
| `auto_fps` | 20 | 自動模式推論頻率（Hz）|
| `auto_kp` | 0.2 | PID 比例增益 |
| `auto_correct_skip` | 1 | 每幾幀執行一次馬達修正 |
| `show_lane_debug` | True | 是否將偵測線疊回串流 |

**執行緒架構（3 個 daemon thread）：**

1. `inference_thread`：持續擷取畫面；手動模式下每 2 幀跑一次 YOLO；自動模式跳過 YOLO 節省 CPU。
2. `lane_follow_thread`：自動模式啟動時，每幀執行車道偵測 → PID → 馬達指令。
3. Flask 內建執行緒（`threaded=True`）：處理 HTTP 請求。

**執行緒安全：**
- `frame_lock`：保護共用的 `annotated_frame`
- `i2c_lock`：保護所有 PCA9685 I2C 寫入操作

**容錯旗標：**
- `MOTOR_AVAILABLE`：馬達模組載入失敗時為 False，程式仍可執行（純視覺模式）
- `LANE_AVAILABLE`：車道偵測模組載入失敗時為 False

---

### `lane_detection.py` — 車道偵測

**偵測流程：**
```
BGR frame
  → HSV 白色遮罩（飽和度 + 亮度過濾）
  → 6 頂點多邊形 ROI（比例座標，可由 UI 調整）
  → 形態學閉合（15×5 kernel，填補木紋縫隙）
  → HoughLinesP（threshold=15, minLen=20, maxGap=150）
  → 幾何過濾（太短、頂部線段、垂直線排除）
  → 斜率分類：slope<0 → 左線，slope>0 → 右線（|slope|<0.2 退回用 x 位置）
  → 斜率一致性驗證：各側中位斜率方向不符則整側丟棄
  → 單線雜訊過濾（兩側數量差距超過 NOISE_RATIO 倍時丟棄少數側）
  → 雙線：EMA 自動學習 LANE_HALF_WIDTH，取中點算 e
  → 單線：以 LANE_HALF_WIDTH 推算車道中心算 e（±SINGLE_E_CLAMP 夾值）
  → 回傳 e（偏差像素）+ debug 圖
```

**幀間持久狀態：**
- `LANE_HALF_WIDTH`：雙線可見時以 EMA 自動學習（0.8 舊值 + 0.2 新測量值）

**e 值定義：** `e = 車道中心 x - 畫面中心 x`（正值 = 偏右，負值 = 偏左）

**主要可調參數：**

| 參數 | 預設值 | 說明 |
|------|--------|------|
| `WHITE_V_MIN` | 140 | 白色亮度下限 |
| `HOUGH_THRESHOLD` | 15 | Hough 投票門檻 |
| `LANE_HALF_WIDTH` | 160 | 車道半寬（像素，自動學習）|
| `SINGLE_TARGET_RATIO` | 0.5 | 單線時目標 x 比例 |
| `HORIZONTAL_SLOPE` | 0.2 | 斜率小於此值視為水平，退回用位置分類 |
| `SINGLE_E_CLAMP` | 200 | 單線 e 最大絕對值 |
| `NOISE_RATIO` | 2.0 | 單線雜訊過濾倍率 |

---

### `motor_control.py` — 馬達控制

**`PCA9685` 類別：** 直接操作 I2C 暫存器，支援 setPWMFreq / setPWM / setDutycycle / setLevel。

**`MotorController` 類別：**
- `forward()` — 四輪同方向前進
- `backward()` — 四輪同方向後退
- `left()` — 左側 A/C 後退、右側 B/D 前進（原地左轉）
- `right()` — 左側 A/C 前進、右側 B/D 後退（原地右轉）
- `stop()` — PWM 全部清零
- 預設速度：60% duty cycle

---

### `pid_controller.py` — PID 控制器

```python
PIDController(kp=0.2, ki=0.0, kd=0.0)  # 目前僅用 P 控制
```

- `compute(e)` → 回傳 float 修正量（含積分飽和限制 ±200）
- `pid_to_speeds(pid_output, base_speed=35)` → 回傳 `(left_speed, right_speed)`
  - left  = base_speed + correction
  - right = base_speed - correction
  - 兩者皆夾至 [0, 100]

---

## API 端點

| 方法 | 路徑 | 說明 |
|------|------|------|
| GET | `/` | 回傳 `index.html` 控制介面 |
| POST | `/control` | 手動馬達指令：`action=forward/backward/left/right/stop` |
| POST | `/auto` | 切換自動巡線：`action=start/stop` |
| POST | `/settings` | 動態調參（JSON）：`speed`, `fps`, `kp`, `correct_skip`, `show_debug` |
| POST | `/camera` | 雲台控制：`direction=cam_left/cam_right/cam_up/cam_down/cam_center/cam_release` |
| GET | `/status` | 回傳系統狀態 JSON（含 CPU、記憶體、速度、角度、自動模式狀態） |
| GET | `/video_feed` | MJPEG 即時影像串流（multipart/x-mixed-replace）|

---

## 依賴套件（需自行安裝）

```bash
pip install flask opencv-python ultralytics smbus2 psutil
# Raspberry Pi 內建（通常不需 pip）：
#   picamera2, gpiozero
```

**缺少 `requirements.txt`**，以上為程式碼 import 反推出的清單。

---

## 已知問題 / TODO

- **設定不持久化**：所有參數（speed、kp 等）僅存在 RAM，重啟後回到預設值。
- **無 requirements.txt**：建議補上。
- **車道遺失處理**：雙線都偵測不到時回傳 `e=None`，馬達停止，無跨幀 tracking。
- **I2C 鎖爭搶**：三個執行緒共用 `i2c_lock`，高負載時可能造成控制延遲。
- **硬編碼數值**：HSV 閾值、伺服機通道、I2C 位址均散佈在各檔案中，未集中設定。
- **Ki / Kd 未使用**：PID 三項中僅 Kp 有效，若需要 D 控制需調整前端。
