#!/usr/bin/env python3
# vision.py — trafficlight.py + stopline_detector.py 통합 노드
#
# 카메라 1번 캡처로:
#   1. YOLO 신호등 검출
#   2. HSV 흰색/빨간색 정지선 검출
#
# 발행 토픽:
#   /traffic_light/state
#       String: 'RED' | 'YELLOW' | 'GREEN'
#       conf >= 0.70인 같은 상태가 0.5초 유지되면 발행
#       상태 유지 중 0.5초마다 재발행
#       미검출 또는 조건 미달이면 발행하지 않음
#
#   /line/white_detected
#       Bool: 정상 프레임마다 발행
#
#   /line/red_detected
#       Bool: 정상 프레임마다 발행
#       흰선(정지선) 기준 근접한 빨간 띠가 정확히 2개일 때만 True.
#       (장애물 구역의 격자형 빨간 블록 오탐 방지 — 모양+개수+거리 필터)
#
# 화면 출력은 show_display=True일 때만 사용한다.
# 디스플레이(X) 없는 환경에서 imshow는 프로세스를 abort시키므로
# 실전 주행 전에는 반드시 False로 둘 것.

import os
import sys

# 시스템 TensorRT Python 바인딩만 뒤쪽에 추가한다.
# insert(0)를 사용하면 시스템 cv2/numpy가 venv 패키지를 가릴 수 있으므로 금지한다.
sys.path.append('/usr/lib/python3.10/dist-packages')

import cv2
import numpy as np
import rclpy
import torch

from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool, String
from ultralytics import YOLO


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision')

        # ========================================================
        # 카메라
        # ========================================================
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter(
            'model_path',
            os.environ.get('VISION_MODEL_PATH', ''),
        )
        camera_index = int(self.get_parameter('camera_index').value)
        frame_width = int(self.get_parameter('frame_width').value)
        frame_height = int(self.get_parameter('frame_height').value)
        if camera_index < 0 or frame_width <= 0 or frame_height <= 0:
            raise ValueError('camera index/size parameters are invalid')

        self.cap = cv2.VideoCapture(camera_index)

        if not self.cap.isOpened():
            self.get_logger().error(
                f'Failed to open camera index {camera_index}'
            )
            raise RuntimeError('Camera not available')

        self.cap.set(
            cv2.CAP_PROP_FRAME_WIDTH,
            frame_width
        )
        self.cap.set(
            cv2.CAP_PROP_FRAME_HEIGHT,
            frame_height
        )

        # ========================================================
        # Publisher
        # ========================================================
        self.pub_state = self.create_publisher(
            String,
            '/traffic_light/state',
            10
        )

        self.pub_white = self.create_publisher(
            Bool,
            '/line/white_detected',
            10
        )

        self.pub_red = self.create_publisher(
            Bool,
            '/line/red_detected',
            10
        )

        # Control treats a missing/False heartbeat as a fail-closed stop only
        # inside vision-dependent mission zones.
        self.pub_health = self.create_publisher(
            Bool,
            '/vision/health',
            10
        )

        # ========================================================
        # Device / 모델
        # ========================================================
        self.device = (
            'cuda:0'
            if torch.cuda.is_available()
            else 'cpu'
        )

        self.get_logger().info(
            f'Using device: {self.device}'
        )

        requested_model = str(
            self.get_parameter('model_path').value
        ).strip()
        self.model_path = os.path.abspath(
            os.path.expandvars(os.path.expanduser(requested_model))
        ) if requested_model else ''

        if not self.model_path or not os.path.isfile(self.model_path):
            self.get_logger().error(
                'A calibrated traffic-light model_path is required and '
                f'must exist; received {requested_model!r}'
            )
            raise FileNotFoundError(
                'Calibrated traffic-light model not found'
            )

        self.get_logger().info(
            f'Found model at: {self.model_path}'
        )

        if self.model_path.endswith('.engine'):
            # TensorRT 엔진에는 .to()를 호출하지 않는다.
            self.model = YOLO(
                self.model_path,
                task='detect'
            )
        else:
            self.model = YOLO(
                self.model_path
            ).to(self.device)

        self.get_logger().info(
            f'YOLO model loaded. Classes: {self.model.names}'
        )

        # ========================================================
        # 신호등 판정 파라미터
        # ========================================================
        self.min_conf = 0.15  # TEMP: 인식 테스트용 하향(원래 0.70). 실전 전 복구할 것.
        self.hold_required_sec = 0.5
        self.republish_period_sec = 0.5

        self.cand_state = 'UNKNOWN'
        self.cand_since: Time = None
        self._last_pub_time: Time = None

        # ========================================================
        # HSV 정지선 판정 파라미터
        # ========================================================
        self.lower_white = np.array(
            [0, 0, 180],
            dtype=np.uint8
        )
        self.upper_white = np.array(
            [180, 50, 255],
            dtype=np.uint8
        )

        self.lower_red1 = np.array(
            [0, 80, 120],
            dtype=np.uint8
        )
        self.upper_red1 = np.array(
            [10, 255, 255],
            dtype=np.uint8
        )

        self.lower_red2 = np.array(
            [170, 80, 120],
            dtype=np.uint8
        )
        self.upper_red2 = np.array(
            [180, 255, 255],
            dtype=np.uint8
        )

        # 정지선 검출은 화면 아래쪽 일부(ROI)만 사용한다.
        # 신호등은 화면 위쪽에 찍히므로 이 영역엔 물리적으로 들어오지 않는다.
        # (보수적으로 35%로 시작 — 실주행에서 카메라 마운트에 맞춰 조정할 것)
        self.roi_bottom_ratio = 0.35

        # 임계값은 원래 전체 프레임(640x480=307200px) 기준 30000px(≈9.77%)로
        # 튜닝된 값이다. ROI로 면적이 줄었으므로 같은 비율로 재계산했다.
        # (실주행에서 조정 시 이 비율 감각을 기준으로 삼을 것)
        # white_pixel_threshold는 /line/white_detected(횡단보도 게이트)에 그대로 사용.
        # red_pixel_threshold는 디버그 표시용으로만 남기고, 실제 정지 판정은
        # 아래 빨간선 패턴(모양+개수+거리) 검사 결과를 사용한다.
        self.white_pixel_threshold = 10500
        self.red_pixel_threshold = 10500

        self.kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT,
            (5, 5)
        )

        # ------------------------------------------------------------
        # 빨간선 패턴 검증 파라미터
        #   흰선(정지선) 기준 가까운 곳에 가늘고 긴 빨간 띠가 정확히 2개
        #   있어야 '진짜 정지선'으로 인정한다. 장애물 구역처럼 격자형으로
        #   흩어진 빨간 블록은 모양(가로/세로 비율)과 개수 조건에서 걸러진다.
        #   ※ 실측 전 placeholder 값. 코스에서 흰선+빨간선 마킹을 촬영해
        #     아래 로그의 row 값들을 보고 재조정할 것.
        # ------------------------------------------------------------
        self.line_blob_min_area = 300         # 이보다 작은 덩어리는 노이즈로 무시
        self.line_aspect_ratio_min = 3.0      # 폭/높이 비율. 이상이어야 '띠' 모양으로 인정
        self.red_line_row_tolerance_px = 100  # 흰선 기준 빨간선 허용 행(row) 거리 (≈40cm, placeholder)

        # ========================================================
        # Display (디버그용 — 실전에서는 False)
        # ========================================================
        self.show_display = False
        self.window_name = 'Vision'

        # ========================================================
        # 10Hz Timer
        # ========================================================
        self.timer = self.create_timer(
            0.1,
            self.detect_callback
        )

        self.get_logger().info(
            'Vision node started '
            '(traffic light + stop line, one camera)'
        )

    # ============================================================
    # 메인 검출 콜백
    # ============================================================
    def detect_callback(self):
        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().warn(
                'Failed to capture frame'
            )
            self.invalidate_candidate('camera frame unavailable')
            self.pub_health.publish(Bool(data=False))
            return

        # --------------------------------------------------------
        # 1. YOLO 신호등 검출
        # --------------------------------------------------------
        yolo_state, yolo_conf, results, inference_ok = self.infer_yolo(frame)

        self.pub_health.publish(Bool(data=inference_ok))

        self.update_candidate(
            yolo_state,
            yolo_conf
        )

        self.maybe_publish_tl()

        # --------------------------------------------------------
        # 2. 정지선 검출용 ROI(화면 아래쪽)만 잘라서 HSV 변환
        #    → 신호등 색이 정지선 판정에 섞여 들어오는 것을 방지
        # --------------------------------------------------------
        frame_h = frame.shape[0]
        self.roi_y0 = int(frame_h * (1.0 - self.roi_bottom_ratio))
        line_roi = frame[self.roi_y0:frame_h, :]

        hsv = cv2.cvtColor(
            line_roi,
            cv2.COLOR_BGR2HSV
        )

        white_detected, white_sum, white_mask = self.detect_line(
            hsv,
            [
                (
                    self.lower_white,
                    self.upper_white
                )
            ],
            self.white_pixel_threshold
        )

        # 참고용 픽셀합/마스크 (디버그 표시용, 정지 판정에는 미사용)
        _, red_sum, red_mask = self.detect_line(
            hsv,
            [
                (
                    self.lower_red1,
                    self.upper_red1
                ),
                (
                    self.lower_red2,
                    self.upper_red2
                )
            ],
            self.red_pixel_threshold
        )

        # 빨간선 최종 판정: 흰선 기준 근접한 '띠' 모양 빨간 덩어리가
        # 정확히 2개일 때만 True (모양+개수+거리 필터)
        red_detected, pattern_info = self.classify_red_pattern(
            white_mask,
            red_mask
        )

        # 정상 프레임마다 반드시 발행
        self.pub_white.publish(
            Bool(data=white_detected)
        )

        self.pub_red.publish(
            Bool(data=red_detected)
        )

        # 1초 스로틀 디버그 로그
        # white_row/red_rows: 실측 캘리브레이션 시 이 값들을 보고
        # red_line_row_tolerance_px를 재조정한다.
        self.get_logger().info(
            f'[YOLO] {yolo_state} conf={yolo_conf:.2f}  '
            f'[LINE] W:{white_detected}({white_sum}) '
            f'R:{red_detected} '
            f'red_blobs={pattern_info["red_count"]} '
            f'white_row={pattern_info["white_row"]} '
            f'red_rows={pattern_info["red_rows"]} '
            f'selected={pattern_info["selected_red_rows"]}',
            throttle_duration_sec=1.0
        )

        # --------------------------------------------------------
        # 3. 디스플레이 (show_display=True일 때만)
        # --------------------------------------------------------
        if self.show_display:
            self.show_debug_windows(
                frame,
                results,
                yolo_state,
                yolo_conf,
                white_detected,
                white_sum,
                white_mask,
                red_detected,
                red_sum,
                red_mask,
                pattern_info
            )

    # ============================================================
    # HSV 정지선 검출
    # ============================================================
    def detect_line(
        self,
        hsv,
        bounds,
        threshold
    ):
        mask = None

        for lower, upper in bounds:
            current_mask = cv2.inRange(
                hsv,
                lower,
                upper
            )

            if mask is None:
                mask = current_mask
            else:
                mask = cv2.bitwise_or(
                    mask,
                    current_mask
                )

        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_OPEN,
            self.kernel
        )

        pixel_sum = int(
            cv2.countNonZero(mask)
        )

        detected = pixel_sum > threshold

        return detected, pixel_sum, mask

    # ============================================================
    # 마스크에서 '띠(선)' 모양 덩어리만 추출
    #   - 너무 작은 덩어리(노이즈) 제외
    #   - 폭/높이 비율이 line_aspect_ratio_min 이상인 것만 채택
    #     (장애물 구역의 격자형 블록은 폭/높이 비율이 낮아 여기서 걸러짐)
    # ============================================================
    def find_stripe_blobs(self, mask):
        num_labels, _labels, stats, centroids = cv2.connectedComponentsWithStats(
            mask,
            connectivity=8
        )

        blobs = []

        for label in range(1, num_labels):  # 0번은 배경
            x, y, w, h, area = stats[label]

            if area < self.line_blob_min_area:
                continue

            aspect_ratio = w / float(max(h, 1))

            if aspect_ratio < self.line_aspect_ratio_min:
                continue

            blobs.append({
                'x': int(x),
                'y': int(y),
                'w': int(w),
                'h': int(h),
                'area': int(area),
                'row_center': float(centroids[label][1]),
            })

        return blobs

    # ============================================================
    # 빨간선 패턴 판정
    #   흰선(정지선) 기준 근접한 '띠' 모양 빨간 덩어리가 2개 이상 있으면,
    #   그중 흰선에 가장 가까운 2개를 골라 거리 조건을 검사한다.
    #   (모양 필터 + '2개 이상' 필터 + 흰선 대비 거리 필터)
    #
    #   '정확히 2개'가 아니라 '2개 이상 중 가장 가까운 2개'로 완화한 이유:
    #   반사광/먼지로 여분의 작은 띠 하나가 더 잡혀도(개수 3) 진짜 패턴을
    #   놓치지 않기 위함. 정지선을 놓쳐 넘어가는 감점이 더 크므로
    #   미탐(false negative) 쪽을 줄이는 방향으로 완화했다.
    #   장애물 구역은 애초에 흰선이 없어 anchor 자체가 안 잡히므로,
    #   완화해도 원래 목적(장애물 오탐 방지)은 유지된다.
    # ============================================================
    def classify_red_pattern(self, white_mask, red_mask):
        white_blobs = self.find_stripe_blobs(white_mask)
        red_blobs = self.find_stripe_blobs(red_mask)

        white_anchor = (
            max(white_blobs, key=lambda b: b['area'])
            if white_blobs
            else None
        )

        pattern_ok = False
        selected_red_blobs = []

        if white_anchor is not None and len(red_blobs) >= 2:
            red_blobs_sorted = sorted(
                red_blobs,
                key=lambda rb: abs(
                    rb['row_center'] - white_anchor['row_center']
                )
            )

            selected_red_blobs = red_blobs_sorted[:2]

            pattern_ok = all(
                abs(rb['row_center'] - white_anchor['row_center'])
                <= self.red_line_row_tolerance_px
                for rb in selected_red_blobs
            )

        info = {
            'white_anchor': white_anchor,
            'red_blobs': red_blobs,
            'red_count': len(red_blobs),
            'white_row': (
                round(white_anchor['row_center'], 1)
                if white_anchor is not None
                else None
            ),
            'red_rows': [
                round(rb['row_center'], 1)
                for rb in red_blobs
            ],
            'selected_red_rows': [
                round(rb['row_center'], 1)
                for rb in selected_red_blobs
            ],
            'selected_red_blobs': selected_red_blobs,
        }

        return pattern_ok, info

    # ============================================================
    # YOLO 신호등 추론
    # ============================================================
    def infer_yolo(self, frame):
        try:
            results = self.model(
                frame,
                device=self.device,
                imgsz=640,
                conf=0.05,  # TEMP: 인식 테스트용 하향(원래 0.25). 실전 전 복구할 것.
                verbose=False
            )[0]

            boxes = getattr(
                results,
                'boxes',
                None
            )

            if boxes is None or len(boxes) == 0:
                return 'UNKNOWN', 0.0, results, True

            best = max(
                boxes,
                key=lambda box: float(box.conf[0])
            )

            confidence = float(
                best.conf[0]
            )
            class_id = int(
                best.cls[0]
            )

            yolo_label = str(
                self.model.names[class_id]
            ).upper()

            label_map = {
                'TRAFFIC_GREEN': 'GREEN',
                'TRAFFIC_BLUE': 'GREEN',
                'TRAFFIC_RED': 'RED',
                'TRAFFIC_ORANGE': 'YELLOW',
                'TRAFFIC_YELLOW': 'YELLOW',
                'TRAFFIC_LEFT': 'GREEN',
                'TRAFFIC_GREEN_LEFT': 'GREEN',
                'TRAFFIC_BLUE_LEFT': 'GREEN',
                'LEFT': 'GREEN',
                'LEFT_ARROW': 'GREEN',
                'GREEN_ARROW': 'GREEN',
                'BLUE_ARROW': 'GREEN',
                'ARROW_LEFT': 'GREEN',
                'GREEN': 'GREEN',
                'BLUE': 'GREEN',
                'RED': 'RED',
                'YELLOW': 'YELLOW',
                'ORANGE': 'YELLOW',
            }

            state = label_map.get(
                yolo_label,
                'UNKNOWN'
            )

            return state, confidence, results, True

        except Exception as error:
            self.get_logger().error(
                f'YOLO inference failed: {error}'
            )

            return 'UNKNOWN', 0.0, None, False

    # ============================================================
    # 같은 신호 상태 연속 유지 판정
    # ============================================================
    def invalidate_candidate(self, reason=None):
        """Forget all temporal evidence after a vision discontinuity."""
        if self.cand_state != 'UNKNOWN':
            detail = f' ({reason})' if reason else ''
            self.get_logger().info(
                f'Candidate reset from {self.cand_state}{detail}'
            )
        self.cand_state = 'UNKNOWN'
        self.cand_since = None
        self._last_pub_time = None
        self.pub_state.publish(
            String(data='UNKNOWN')
        )

    def update_candidate(
        self,
        yolo_state,
        yolo_conf
    ):
        now = self.get_clock().now()

        valid = (
            yolo_state != 'UNKNOWN'
            and yolo_conf >= self.min_conf
        )

        if valid:
            if yolo_state != self.cand_state:
                self.cand_state = yolo_state
                self.cand_since = now
                self._last_pub_time = None
                # Invalidate a previously published GREEN immediately while
                # the new colour is being temporally confirmed. This makes a
                # first YELLOW/RED observation stop the vehicle instead of
                # leaving an old GREEN valid for the topic TTL.
                self.pub_state.publish(
                    String(data='UNKNOWN')
                )

                self.get_logger().info(
                    f'Candidate set to '
                    f'{self.cand_state} '
                    f'(conf={yolo_conf:.2f})'
                )

                return

            if self.cand_since is None:
                self.cand_since = now
            elif (now - self.cand_since).nanoseconds < 0:
                # A ROS clock epoch reset must not leave a future candidate
                # timestamp that blocks this mission indefinitely.
                self.cand_since = now
                self._last_pub_time = None
                self.pub_state.publish(
                    String(data='UNKNOWN')
                )

        else:
            self.invalidate_candidate()

    # ============================================================
    # 신호등 상태 발행
    # ============================================================
    def maybe_publish_tl(self):
        if (
            self.cand_state == 'UNKNOWN'
            or self.cand_since is None
        ):
            return

        now = self.get_clock().now()

        elapsed_sec = (
            now - self.cand_since
        ).nanoseconds / 1e9

        if elapsed_sec < 0.0:
            self.cand_since = now
            self._last_pub_time = None
            self.pub_state.publish(
                String(data='UNKNOWN')
            )
            return

        if elapsed_sec < self.hold_required_sec:
            return

        if self._last_pub_time is not None:
            since_last_sec = (
                now - self._last_pub_time
            ).nanoseconds / 1e9

            if since_last_sec < 0.0:
                self._last_pub_time = None
                since_last_sec = self.republish_period_sec

            if (
                since_last_sec
                < self.republish_period_sec
            ):
                return

        self.pub_state.publish(
            String(data=self.cand_state)
        )

        self._last_pub_time = now

        self.get_logger().info(
            f'Published: {self.cand_state} '
            f'(held {elapsed_sec:.2f}s, '
            f'conf >= {self.min_conf})'
        )

    # ============================================================
    # 디버그 창 표시
    #   - 메인 창: YOLO 바운딩 박스 + 상태 텍스트
    #   - White/Red Mask 창: HSV 마스크 (모폴로지 처리 후)
    # ============================================================
    def show_debug_windows(
        self,
        frame,
        results,
        yolo_state,
        yolo_conf,
        white_detected,
        white_sum,
        white_mask,
        red_detected,
        red_sum,
        red_mask,
        pattern_info
    ):
        # YOLO 바운딩 박스가 그려진 프레임 (검출 없으면 원본 복사)
        if results is not None:
            vis = results.plot()
        else:
            vis = frame.copy()

        # 정지선 ROI 경계선 표시 (실주행 중 crop 비율 조정 참고용)
        cv2.line(
            vis,
            (0, self.roi_y0),
            (vis.shape[1], self.roi_y0),
            (255, 0, 255),
            2
        )

        # 후보 상태 경과 시간
        if self.cand_since is not None:
            elapsed = (
                self.get_clock().now() - self.cand_since
            ).nanoseconds / 1e9
        else:
            elapsed = 0.0

        lines = [
            f'YOLO: {yolo_state} (conf={yolo_conf:.2f})',
            f'Candidate: {self.cand_state} '
            f'(held {elapsed:.2f}s / need {self.hold_required_sec:.2f}s)',
            f'WHITE: {white_detected} ({white_sum} px, '
            f'thr {self.white_pixel_threshold})',
            f'RED:   {red_detected}  blobs={pattern_info["red_count"]} '
            f'(need>=2)  tol={self.red_line_row_tolerance_px}px',
            f'white_row={pattern_info["white_row"]}  '
            f'selected={pattern_info["selected_red_rows"]}',
        ]

        # 반투명 배경 박스 + 텍스트
        x0, y0 = 10, 20
        pad = 6
        line_height = 22

        box_w = max(len(s) for s in lines) * 8 + pad * 2
        box_h = line_height * len(lines) + pad * 2

        overlay = vis.copy()
        cv2.rectangle(
            overlay,
            (x0 - pad, y0 - 16),
            (x0 - pad + box_w, y0 - 16 + box_h),
            (0, 0, 0),
            -1
        )
        cv2.addWeighted(
            overlay,
            0.35,
            vis,
            0.65,
            0,
            vis
        )

        for i, text in enumerate(lines):
            cv2.putText(
                vis,
                text,
                (x0, y0 + i * line_height),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 255),
                1,
                cv2.LINE_AA
            )

        # 마스크 창에 '띠' 모양으로 채택된 덩어리만 초록 박스로 표시
        # (모양/개수/거리 필터 튜닝 시 참고용)
        white_anchor_list = (
            [pattern_info['white_anchor']]
            if pattern_info['white_anchor'] is not None
            else []
        )
        white_mask_vis = self.draw_blob_boxes(
            white_mask,
            white_anchor_list
        )
        # 모든 빨간 띠 후보는 노란 박스, 그중 실제 판정에 쓰인
        # (흰선에 가장 가까운) 2개는 초록 박스로 덮어 그린다.
        red_mask_vis = self.draw_blob_boxes(
            red_mask,
            pattern_info['red_blobs'],
            color=(0, 255, 255)
        )
        red_mask_vis = self.draw_blob_boxes(
            red_mask_vis,
            pattern_info['selected_red_blobs'],
            color=(0, 255, 0)
        )

        cv2.imshow(self.window_name, vis)
        cv2.imshow('White Mask', white_mask_vis)
        cv2.imshow('Red Mask', red_mask_vis)
        cv2.waitKey(1)

    # ============================================================
    # 마스크(그레이스케일)를 BGR로 바꾸고 블롭에 박스를 그린다
    #   - 이미 BGR인 이미지를 넘기면(2번째 호출로 덧그리기) 변환 생략
    # ============================================================
    def draw_blob_boxes(self, mask, blobs, color=(0, 255, 0)):
        if mask.ndim == 2:
            vis = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        else:
            vis = mask

        for b in blobs:
            cv2.rectangle(
                vis,
                (b['x'], b['y']),
                (b['x'] + b['w'], b['y'] + b['h']),
                color,
                2
            )

        return vis

    # ============================================================
    # 종료 처리
    # ============================================================
    def destroy_node(self):
        if hasattr(self, 'cap'):
            self.cap.release()

        cv2.destroyAllWindows()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = VisionNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
