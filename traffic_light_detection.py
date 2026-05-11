import cv2
import numpy as np
import os
import time
from collections import deque

MODEL_PATH = os.path.join(os.path.dirname(__file__), 'best.pt')

_model = None
_model_available = False

# 偵測參數
CONF_THRESHOLD  = 0.25  # 降低全域門檻，讓小目標也能通過
IOU_THRESHOLD   = 0.45
SMOOTH_WINDOW   = 5     # 時序平滑窗口（幀數）
SMOOTH_MIN_HITS = 2     # 窗口內需出現幾次才顯示（從3降為2，減少漏偵測）

# 各類別的時序緩衝（每幀記錄 {label: conf_val}）
_history: deque = deque(maxlen=SMOOTH_WINDOW)

# 各燈號的獨立信心值門檻（None = 使用 CONF_THRESHOLD）
CONF_RED    = 0.20  # 紅燈用更低門檻，停車安全優先（寧可誤判也不漏判）
CONF_YELLOW = None
CONF_GREEN  = None

# 距離過濾：bbox 面積需佔畫面比例的最小值（太小=距離太遠，忽略）
TL_MIN_AREA_RATIO = 0.003  # 降低至 0.3%（640×480 約 922 px²），允許偵測較遠/小的燈號

# 推論降頻：每隔幾幀才真正跑 YOLO（其餘幀沿用上次結果）
INFER_EVERY_N = 2  # 從3降為2，提高偵測頻率
_infer_counter = 0
_last_best = None          # 上次推論的原始 best（含 bbox，供繪圖用）

# YOLO 輸入縮放（推論用，不影響顯示）
INFER_WIDTH  = 416  # 從320提高至416，保留更多小目標細節
INFER_HEIGHT = 416

# 各類別顏色（BGR）
CLASS_COLORS = {
    'red':    (0,   0,   255),
    'yellow': (0,   200, 255),
    'green':  (0,   200, 0),
}
DEFAULT_COLOR = (200, 200, 0)

# 最後一次穩定偵測結果（供外部查詢）
last_stable: dict | None = None   # {'label': str, 'conf': float, 'bbox': tuple}


def load_model():
    global _model, _model_available
    try:
        from ultralytics import YOLO
        _model = YOLO(MODEL_PATH)
        _model_available = True
        print(f"✅ 紅綠燈模型載入成功: {MODEL_PATH}")
        # warm-up：讓模型預先編譯，避免第一幀推論特別慢
        dummy = np.zeros((INFER_HEIGHT, INFER_WIDTH, 3), dtype=np.uint8)
        _model.predict(source=dummy, conf=0.9, iou=0.45, verbose=False)
        print("[TL] warm-up 完成")
    except Exception as e:
        import traceback
        print(f"[警告] 紅綠燈模型載入失敗: {e}")
        traceback.print_exc()
        _model_available = False
    return _model_available


def is_available():
    return _model_available


def _conf_for(label: str) -> float:
    """取得該類別的信心值門檻（各別設定或 fallback 全域值）"""
    ll = label.lower()
    if 'red'    in ll and CONF_RED    is not None: return CONF_RED
    if 'yellow' in ll and CONF_YELLOW is not None: return CONF_YELLOW
    if 'green'  in ll and CONF_GREEN  is not None: return CONF_GREEN
    return CONF_THRESHOLD


def detect_objects(frame, conf=None, iou=None):
    """
    回傳：
      - annotated : 畫有偵測框的 BGR 影像
      - detection : 唯一穩定燈號 dict {'label','conf','bbox'} 或 None
    規則：同一幀只取信心值最高的一個類別，再經時序平滑後輸出。
    每 INFER_EVERY_N 幀才真正執行推論，其餘幀沿用上次結果以降低 CPU 負擔。
    """
    global last_stable, _infer_counter, _last_best

    if not _model_available or _model is None:
        return frame.copy(), None

    _iou = iou if iou is not None else IOU_THRESHOLD
    _infer_counter += 1

    if _infer_counter >= INFER_EVERY_N:
        _infer_counter = 0

        # 縮小輸入解析度再推論，降低 CPU 負擔
        fh, fw = frame.shape[:2]
        small = cv2.resize(frame, (INFER_WIDTH, INFER_HEIGHT), interpolation=cv2.INTER_LINEAR)

        min_conf = min(filter(None, [CONF_RED, CONF_YELLOW, CONF_GREEN, CONF_THRESHOLD]))
        results = _model.predict(
            source=small,
            conf=min_conf,
            iou=_iou,
            verbose=False,
        )

        # 只保留各別門檻過關的框，取信心值最高的一個
        # bbox 座標從縮小尺寸還原回原始尺寸
        sx = fw / INFER_WIDTH
        sy = fh / INFER_HEIGHT
        frame_area = fw * fh
        best = None
        if results and len(results) > 0:
            result = results[0]
            names  = result.names
            for box in result.boxes:
                conf_val = float(box.conf[0])
                cls_id   = int(box.cls[0])
                label    = names.get(cls_id, str(cls_id))
                if conf_val < _conf_for(label):
                    continue
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                x1, y1, x2, y2 = int(x1*sx), int(y1*sy), int(x2*sx), int(y2*sy)
                bbox_area = (x2 - x1) * (y2 - y1)
                if bbox_area / frame_area < TL_MIN_AREA_RATIO:
                    continue
                if best is None or conf_val > best['conf']:
                    best = {'label': label, 'conf': conf_val, 'bbox': (x1, y1, x2, y2),
                            'area_ratio': bbox_area / frame_area}
        _last_best = best
    else:
        best = _last_best

    # ---- 時序平滑：記錄本幀最佳候選 ----
    _history.append(best['label'] if best else None)

    hits: dict[str, int] = {}
    for entry in _history:
        if entry is not None:
            hits[entry] = hits.get(entry, 0) + 1

    stable = None
    if best and hits.get(best['label'], 0) >= SMOOTH_MIN_HITS:
        stable = best

    last_stable = stable

    # ---- 畫框（直接在傳入的 frame 上操作，呼叫端已確保可寫）----
    annotated = frame
    if stable:
        x1, y1, x2, y2 = stable['bbox']
        _draw_box(annotated, x1, y1, x2, y2, stable['label'], stable['conf'],
                  stable.get('area_ratio'))

    return annotated, stable


def _draw_box(img, x1, y1, x2, y2, label, conf, area_ratio=None):
    color = _get_color(label)
    cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)

    area_str = f" {area_ratio*100:.1f}%" if area_ratio is not None else ""
    text = f"{label} {conf:.2f}{area_str}"
    (tw, th), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
    ty = max(y1 - 5, th + 5)
    cv2.rectangle(img, (x1, ty - th - baseline - 2), (x1 + tw + 4, ty + 2), color, -1)
    cv2.putText(img, text, (x1 + 2, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1, cv2.LINE_AA)


def _get_color(label):
    ll = label.lower()
    for key, color in CLASS_COLORS.items():
        if key in ll:
            return color
    return DEFAULT_COLOR


def get_params():
    return {
        'tl_conf':           CONF_THRESHOLD,
        'tl_conf_red':       CONF_RED    if CONF_RED    is not None else CONF_THRESHOLD,
        'tl_conf_yellow':    CONF_YELLOW if CONF_YELLOW is not None else CONF_THRESHOLD,
        'tl_conf_green':     CONF_GREEN  if CONF_GREEN  is not None else CONF_THRESHOLD,
        'tl_iou':            IOU_THRESHOLD,
        'tl_smooth_win':     SMOOTH_WINDOW,
        'tl_smooth_hits':    SMOOTH_MIN_HITS,
        'tl_min_area_ratio': TL_MIN_AREA_RATIO,
    }


def set_params(params):
    global CONF_THRESHOLD, IOU_THRESHOLD
    global CONF_RED, CONF_YELLOW, CONF_GREEN
    global SMOOTH_WINDOW, SMOOTH_MIN_HITS, _history
    global TL_MIN_AREA_RATIO, INFER_EVERY_N, INFER_WIDTH, INFER_HEIGHT

    if 'tl_conf' in params:
        CONF_THRESHOLD = float(max(0.01, min(1.0, params['tl_conf'])))
    if 'tl_conf_red' in params:
        CONF_RED    = float(max(0.01, min(1.0, params['tl_conf_red'])))
    if 'tl_conf_yellow' in params:
        CONF_YELLOW = float(max(0.01, min(1.0, params['tl_conf_yellow'])))
    if 'tl_conf_green' in params:
        CONF_GREEN  = float(max(0.01, min(1.0, params['tl_conf_green'])))
    if 'tl_iou' in params:
        IOU_THRESHOLD = float(max(0.1, min(1.0, params['tl_iou'])))
    if 'tl_smooth_win' in params:
        SMOOTH_WINDOW = int(max(1, min(30, params['tl_smooth_win'])))
        _history = deque(_history, maxlen=SMOOTH_WINDOW)
    if 'tl_smooth_hits' in params:
        SMOOTH_MIN_HITS = int(max(1, min(SMOOTH_WINDOW, params['tl_smooth_hits'])))
    if 'tl_min_area_ratio' in params:
        TL_MIN_AREA_RATIO = float(max(0.001, min(1.0, params['tl_min_area_ratio'])))
    if 'tl_infer_every_n' in params:
        INFER_EVERY_N = int(max(1, min(10, params['tl_infer_every_n'])))
