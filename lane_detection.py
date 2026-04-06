import cv2
import numpy as np

# HSV 白色過濾閾值（可依相機微調）
WHITE_S_MIN = 0     # 飽和度下限
WHITE_S_MAX = 60    # 飽和度上限：越低越接近純白
WHITE_V_MIN = 140   # 亮度下限：越高越只取亮白色
WHITE_V_MAX = 255   # 亮度上限：可排除過曝區域

# Hough 與幾何過濾參數
HOUGH_THRESHOLD  = 15   # Hough 累加器投票門檻
HOUGH_MIN_LEN    = 20   # 最小線段長度（像素）
HOUGH_MAX_GAP    = 150  # 最大線段間隙（像素）
SEG_MIN_LEN      = 25   # 幾何過濾：線段長度下限（像素）

# 車道半寬（雙線時自動學習，單線 fallback 用）
LANE_HALF_WIDTH  = 160  # 預設值（像素），可由 UI 調整

# 單線行為參數
SINGLE_TARGET_RATIO = 0.5   # 單線目標位置（0.0=最左, 1.0=最右, 0.5=中央）
HORIZONTAL_SLOPE    = 0.2   # 斜率絕對值小於此值視為水平線（大轉彎）
SINGLE_E_CLAMP      = 200   # 單線 e 的最大絕對值，避免過度修正
NOISE_RATIO         = 2.0   # 單線雜訊過濾倍率：另一側線段數少於此倍時丟棄
OVERLAY_ALPHA       = 0.4   # 綠色遮罩重疊倍率（0=不顯示, 1=完全覆蓋）
LANE_MERGE_RATIO    = 0.5   # 雙線合併比例（0=全用左線, 1=全用右線, 0.5=各半）


def get_lane_params():
    return {
        'white_s_min':         WHITE_S_MIN,
        'white_s_max':         WHITE_S_MAX,
        'white_v_min':         WHITE_V_MIN,
        'white_v_max':         WHITE_V_MAX,
        'hough_threshold':     HOUGH_THRESHOLD,
        'hough_min_len':       HOUGH_MIN_LEN,
        'hough_max_gap':       HOUGH_MAX_GAP,
        'seg_min_len':         SEG_MIN_LEN,
        'lane_half_width':     LANE_HALF_WIDTH,
        'single_target_ratio': SINGLE_TARGET_RATIO,
        'horizontal_slope':    HORIZONTAL_SLOPE,
        'single_e_clamp':      SINGLE_E_CLAMP,
        'noise_ratio':         NOISE_RATIO,
        'overlay_alpha':       OVERLAY_ALPHA,
        'lane_merge_ratio':    LANE_MERGE_RATIO,
    }


def set_lane_params(params):
    global WHITE_S_MIN, WHITE_S_MAX, WHITE_V_MIN, WHITE_V_MAX
    global HOUGH_THRESHOLD, HOUGH_MIN_LEN, HOUGH_MAX_GAP, SEG_MIN_LEN
    global LANE_HALF_WIDTH, SINGLE_TARGET_RATIO, HORIZONTAL_SLOPE, SINGLE_E_CLAMP, NOISE_RATIO, OVERLAY_ALPHA, LANE_MERGE_RATIO
    if 'white_s_min'         in params: WHITE_S_MIN           = int(max(0,    min(255,  params['white_s_min'])))
    if 'white_s_max'         in params: WHITE_S_MAX           = int(max(0,    min(255,  params['white_s_max'])))
    if 'white_v_min'         in params: WHITE_V_MIN           = int(max(0,    min(255,  params['white_v_min'])))
    if 'white_v_max'         in params: WHITE_V_MAX           = int(max(0,    min(255,  params['white_v_max'])))
    if 'hough_threshold'     in params: HOUGH_THRESHOLD       = int(max(1,    min(100,  params['hough_threshold'])))
    if 'hough_min_len'       in params: HOUGH_MIN_LEN         = int(max(5,    min(200,  params['hough_min_len'])))
    if 'hough_max_gap'       in params: HOUGH_MAX_GAP         = int(max(0,    min(300,  params['hough_max_gap'])))
    if 'seg_min_len'         in params: SEG_MIN_LEN           = int(max(5,    min(200,  params['seg_min_len'])))
    if 'lane_half_width'     in params: LANE_HALF_WIDTH       = int(max(50,   min(400,  params['lane_half_width'])))
    if 'single_target_ratio' in params: SINGLE_TARGET_RATIO   = float(max(0.0, min(1.0, params['single_target_ratio'])))
    if 'horizontal_slope'    in params: HORIZONTAL_SLOPE      = float(max(0.0, min(1.0, params['horizontal_slope'])))
    if 'single_e_clamp'      in params: SINGLE_E_CLAMP        = int(max(50,   min(500,  params['single_e_clamp'])))
    if 'noise_ratio'         in params: NOISE_RATIO           = float(max(1.0, min(10.0, params['noise_ratio'])))
    if 'overlay_alpha'       in params: OVERLAY_ALPHA         = float(max(0.0, min(1.0, params['overlay_alpha'])))
    if 'lane_merge_ratio'    in params: LANE_MERGE_RATIO      = float(max(0.0, min(1.0, params['lane_merge_ratio'])))

# 全域 ROI 頂點（6 頂點，比例值 0.0~1.0，順序：左下、左中、左上、右上、右中、右下）
_roi_pts_ratio = [
    (0.0,  1.0),
    (0.15, 0.55),
    (0.3,  0.1),
    (0.7,  0.1),
    (0.85, 0.55),
    (1.0,  1.0),
]


def get_roi_ratio():
    return list(_roi_pts_ratio)


def set_roi_ratio(pts):
    """設定 ROI 頂點（比例值列表，每個元素為 [x_ratio, y_ratio]）"""
    global _roi_pts_ratio
    _roi_pts_ratio = [(float(p[0]), float(p[1])) for p in pts]


def detect_lane(frame):
    h, w = frame.shape[:2]

    # 1. HSV 白色遮罩（飽和度 + 亮度過濾）
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    white_mask = cv2.inRange(hsv,
                             np.array([0,   WHITE_S_MIN, WHITE_V_MIN]),
                             np.array([180, WHITE_S_MAX, WHITE_V_MAX]))

    # 2. ROI 6 頂點遮罩
    roi_mask = np.zeros_like(white_mask)
    roi = np.array([[
        (int(rx * w), int(ry * h)) for rx, ry in _roi_pts_ratio
    ]], dtype=np.int32)
    cv2.fillPoly(roi_mask, roi, 255)
    masked = cv2.bitwise_and(white_mask, roi_mask)

    # 3. 形態學閉合：填補白線上的木紋縫隙，讓碎塊連成完整線段
    close_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (15, 5))
    closed = cv2.morphologyEx(masked, cv2.MORPH_CLOSE, close_kernel)

    # 4. HoughLinesP 抓線段
    lines = cv2.HoughLinesP(closed, 1, np.pi/180,
                             threshold=HOUGH_THRESHOLD,
                             minLineLength=HOUGH_MIN_LEN,
                             maxLineGap=HOUGH_MAX_GAP)

    # 4. 幾何過濾 + 位置分類左右
    left_x,  right_x  = [], []
    left_lines, right_lines = [], []

    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            # 幾何過濾：太短的線段是雜訊
            if np.hypot(x2 - x1, y2 - y1) < SEG_MIN_LEN:
                continue
            # 幾何過濾：完全在畫面頂部的線段排除
            if min(y1, y2) < h * 0.1:
                continue
            # 防止除以零（垂直線）
            if x2 == x1:
                continue

            # 左右分類：斜率為主，位置為輔
            # 影像座標 y 向下，所以斜率 (dy/dx) 正值 = 往右上傾 = 右線
            #                                      負值 = 往左上傾 = 左線
            slope = (y2 - y1) / (x2 - x1)
            bx = x1 if y1 > y2 else x2
            if abs(slope) > HORIZONTAL_SLOPE:
                # 斜率夠明顯，直接用斜率判斷（轉彎時線段跑到對側也不會誤判）
                is_left = slope < 0
            else:
                # 斜率太平（近水平），退回用底部 x 位置判斷
                is_left = bx < w // 2
            if is_left:
                left_x.append(bx)
                left_lines.append((x1, y1, x2, y2))
            else:
                right_x.append(bx)
                right_lines.append((x1, y1, x2, y2))

    # 4b. 斜率一致性驗證：若某側的斜率中位數方向不對，整側丟棄
    def _median_slope(segs):
        slopes = [(y2 - y1) / (x2 - x1) for x1, y1, x2, y2 in segs if x2 != x1]
        return float(np.median(slopes)) if slopes else 0.0

    if left_lines:
        ms = _median_slope(left_lines)
        if ms > 0:  # 左線群的中位斜率應該 < 0，否則是誤判
            left_lines, left_x = [], []
    if right_lines:
        ms = _median_slope(right_lines)
        if ms < 0:  # 右線群的中位斜率應該 > 0，否則是誤判
            right_lines, right_x = [], []

    # 5. 視覺化：綠色半透明遮罩顯示閉合後的範圍
    result = frame.copy()
    overlay = result.copy()
    overlay[closed > 0] = (0, 200, 0)
    cv2.addWeighted(overlay, OVERLAY_ALPHA, result, 1.0 - OVERLAY_ALPHA, 0, result)

    x_center = w // 2
    cv2.line(result, (x_center, h), (x_center, h//2), (255, 255, 255), 1)  # 車身中心白線

    # 取各線段底部（y 最大）端點的 x，比 cx 平均更穩定
    def _bottom_x(segs):
        """回傳所有線段中 y 最大端點的 x 平均"""
        xs = []
        for x1, y1, x2, y2 in segs:
            xs.append(x1 if y1 > y2 else x2)
        return int(np.mean(xs))

    # 6. 計算偏差量 e
    global LANE_HALF_WIDTH

    # 單線雜訊過濾：兩側都有線段時，若數量差距超過 NOISE_RATIO 倍，少的那側視為雜訊丟棄
    if left_lines and right_lines:
        if len(left_lines) > len(right_lines) * NOISE_RATIO:
            right_lines, right_x = [], []
        elif len(right_lines) > len(left_lines) * NOISE_RATIO:
            left_lines, left_x = [], []

    # 繪製偵測線段（過濾後才畫，避免顯示已被丟棄的雜訊線段）
    for (x1, y1, x2, y2) in left_lines:
        cv2.line(result, (x1, y1), (x2, y2), (0, 255, 0), 4)     # 綠 = 左
    for (x1, y1, x2, y2) in right_lines:
        cv2.line(result, (x1, y1), (x2, y2), (0, 128, 255), 4)   # 橘 = 右

    if left_lines and right_lines:
        lx = _bottom_x(left_lines)
        rx = _bottom_x(right_lines)
        # 自動學習車道半寬（EMA）
        measured = (rx - lx) / 2
        if measured > 30:
            LANE_HALF_WIDTH = int(LANE_HALF_WIDTH * 0.8 + measured * 0.2)
        x_lane = (lx + rx) // 2
        e = x_lane - x_center
        cv2.line(result, (x_lane, h), (x_lane, h//2), (0, 255, 255), 2)
        status = f"L+R e={e}px"
        status_color = (0, 255, 255)
    elif left_lines:
        # 只有左線：線的位置決定力度
        lx = _bottom_x(left_lines)
        target_x = int(w * SINGLE_TARGET_RATIO) - LANE_HALF_WIDTH
        e = int(np.clip(lx - target_x, -SINGLE_E_CLAMP, SINGLE_E_CLAMP))
        status = f"L only e={e}"
        status_color = (0, 200, 255)
    elif right_lines:
        # 只有右線：線的位置決定力度
        rx = _bottom_x(right_lines)
        target_x = int(w * SINGLE_TARGET_RATIO) + LANE_HALF_WIDTH
        e = int(np.clip(rx - target_x, -SINGLE_E_CLAMP, SINGLE_E_CLAMP))
        status = f"R only e={e}"
        status_color = (0, 200, 255)
    else:
        e = None
        status = "no lane"
        status_color = (0, 0, 255)

    cv2.polylines(result, roi, True, (255, 200, 0), 1)
    _put_text(result, status, (10, 30), 0.8, status_color)
    _put_text(result, f"L:{len(left_lines)} R:{len(right_lines)}", (10, 60), 0.7, (200, 200, 200))

    return e, result


def _put_text(img, text, pos, scale, color):
    """畫黑底描邊文字，提高辨識度"""
    thickness = max(1, int(scale * 2))
    outline = thickness + max(1, int(scale * 1.5))
    cv2.putText(img, text, pos, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), outline, cv2.LINE_AA)
    cv2.putText(img, text, pos, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thickness, cv2.LINE_AA)


if __name__ == "__main__":
    from picamera2 import Picamera2
    import time

    cam = Picamera2()
    cam.configure(cam.create_video_configuration(
        main={"size": (640, 480), "format": "RGB888"}
    ))
    cam.start()
    time.sleep(2)

    frame = cam.capture_array()
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    frame = cv2.flip(frame, 0)

    e, result = detect_lane(frame)
    cv2.imwrite("lane_test.jpg", result)
    print(f"偏差量 e = {e} px")
    cam.stop()
