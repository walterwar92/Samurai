"""
YoloBackend — единый бэкенд для всех YOLO-вариантов (PyTorch + ONNX, CPU/GPU).

Заменяет три копии загрузки/инференса YOLO:
  - yolo_detector_node.py (ROS2)
  - yolo_detector_mqtt.py (GPU laptop)
  - object_detector_node.py (CPU MQTT)

Возможности:
  - Auto-lookup модели в текущей директории / ~/models/ / корне проекта
  - Auto-export .pt → .onnx (imgsz=416, optimize, simplify) при первом запуске
  - ONNX Runtime с CUDA или CPU providers + tuning потоков
  - Frame-drop: если предыдущий inference ещё работает — пропустить кадр
  - Graceful fallback PyTorch если ONNX недоступен
"""
from __future__ import annotations

import logging
import os
from typing import Optional

import numpy as np

from .base import Detection, DetectorBackend, FrameContext

log = logging.getLogger(__name__)


# Маппинг имён классов модели на (Detection.cls, Detection.colour).
# Срабатывает для моделей обученных через tools/train_balls.py
# (классы ball_red/green/blue). Для COCO-моделей (yolo11n.pt с
# 80 классами) ни одно имя не совпадёт — пройдёт по else-ветке,
# colour=='unknown' дозаполнится HSV-классификатором в DetectionPipeline.
_BALL_CLASS_MAP: dict[str, tuple[str, str]] = {
    'ball_red':    ('ball', 'red'),
    'ball_green':  ('ball', 'green'),
    'ball_blue':   ('ball', 'blue'),
    'ball_yellow': ('ball', 'yellow'),
    'ball_orange': ('ball', 'orange'),
}


def _map_cls_to_cls_colour(cls_name: str) -> tuple[str, str]:
    """Маппинг class name из YOLO на (Detection.cls, Detection.colour).

    FSM на Pi ждёт `colour=red/green/blue` в `ball_detection` MQTT-топике
    и `class='ball'`. Если модель выдаёт `ball_<colour>` напрямую — мы
    раскладываем это здесь, и FSM работает без правок.
    """
    if cls_name in _BALL_CLASS_MAP:
        return _BALL_CLASS_MAP[cls_name]
    return cls_name, 'unknown'


class YoloBackend(DetectorBackend):
    """YOLO детектор (PyTorch или ONNX). Auto-выбор по доступности .onnx файла."""

    DEFAULT_LOOKUP_DIRS = ['.', '~/models']

    def __init__(self,
                 model_path: str = 'yolo11n.pt',
                 device: str = 'cuda',
                 confidence: float = 0.40,
                 imgsz: int = 416,
                 prefer_onnx: bool = True,
                 frame_drop: bool = False):
        """
        Args:
            model_path: имя или полный путь к .pt/.onnx модели
            device: 'cuda' | 'cpu'
            confidence: порог уверенности
            imgsz: размер входа модели (квадрат)
            prefer_onnx: пытаться экспортировать .pt → .onnx (быстрее на CPU/GPU)
            frame_drop: если True — пропускать кадры если предыдущий inference активен
        """
        try:
            from ultralytics import YOLO  # type: ignore  # noqa: F401
        except ImportError as e:
            raise RuntimeError(
                'YoloBackend требует ultralytics. Установи: pip install ultralytics'
            ) from e

        self._device = device
        self._conf = confidence
        self._imgsz = imgsz
        self._frame_drop = frame_drop
        self._processing = False  # для frame-drop

        resolved_path = self._resolve_model_path(model_path)
        if resolved_path is None:
            raise FileNotFoundError(
                f"YOLO модель '{model_path}' не найдена. "
                f"Проверь: {model_path}, ~/models/{model_path}, "
                f"{os.path.dirname(__file__)}/../../{model_path}"
            )

        self._model = None
        self._ort = None
        self._input_name = ''
        self._use_onnx = False
        self._names: dict[int, str] = {}

        if prefer_onnx:
            self._try_load_onnx(resolved_path)

        if not self._use_onnx:
            self._load_pytorch(resolved_path)

    # ── Model resolution ───────────────────────────────────────
    @classmethod
    def _resolve_model_path(cls, model_path: str) -> Optional[str]:
        """Find model file by name, checking multiple lookup dirs."""
        if os.path.isabs(model_path) and os.path.exists(model_path):
            return model_path

        # Path as-is in cwd
        if os.path.exists(model_path):
            return os.path.abspath(model_path)

        # Lookup in standard dirs
        candidates = []
        for base in cls.DEFAULT_LOOKUP_DIRS:
            candidates.append(os.path.expanduser(os.path.join(base, model_path)))

        # Project root (compute_node/.. — where yolo11n.pt is committed)
        project_root = os.path.join(os.path.dirname(__file__), '..', '..')
        candidates.append(os.path.normpath(os.path.join(project_root, model_path)))

        for candidate in candidates:
            if os.path.exists(candidate):
                return candidate
        return None

    # ── ONNX path ──────────────────────────────────────────────
    def _try_load_onnx(self, model_path: str):
        """Export .pt → .onnx if needed, then load via onnxruntime."""
        onnx_path = model_path.replace('.pt', '.onnx') if model_path.endswith('.pt') else model_path

        # Export if .pt но .onnx нет.
        # opset=17 — самый новый, который гарантированно поддерживается
        # текущим onnxruntime (ORT 1.x). Без него ultralytics экспортирует
        # с opset 22 (под-development), ORT валится с "Opset 22 not supported".
        if model_path.endswith('.pt') and not os.path.exists(onnx_path):
            try:
                from ultralytics import YOLO
                log.info('Exporting %s → ONNX (imgsz=%d, opset=17) ...',
                         os.path.basename(model_path), self._imgsz)
                tmp = YOLO(model_path)
                tmp.export(format='onnx', imgsz=self._imgsz, opset=17,
                           optimize=True, simplify=True)
                log.info('ONNX export complete')
            except Exception as e:
                log.warning('ONNX export failed (%s) — fallback to PyTorch', e)
                return

        if not os.path.exists(onnx_path):
            return

        try:
            import onnxruntime as ort  # type: ignore
        except ImportError:
            log.warning('onnxruntime не установлен — fallback to PyTorch')
            return

        try:
            providers = (['CUDAExecutionProvider', 'CPUExecutionProvider']
                         if self._device == 'cuda' else ['CPUExecutionProvider'])

            n_threads = max(1, (os.cpu_count() or 4) // 2)
            sess_opts = ort.SessionOptions()
            sess_opts.intra_op_num_threads = n_threads
            sess_opts.inter_op_num_threads = max(1, n_threads // 2)
            sess_opts.execution_mode = ort.ExecutionMode.ORT_PARALLEL

            self._ort = ort.InferenceSession(
                onnx_path, sess_options=sess_opts, providers=providers)
            self._input_name = self._ort.get_inputs()[0].name
            self._use_onnx = True

            # Class names — пытаемся достать из .pt (если он рядом)
            from ultralytics import YOLO
            if os.path.exists(model_path):
                _tmp = YOLO(model_path)
                self._names = _tmp.names
            else:
                self._names = {0: 'object'}

            active_provider = self._ort.get_providers()[0]
            log.info('ONNX Runtime loaded: %s | provider=%s',
                     os.path.basename(onnx_path), active_provider)
            if self._device == 'cuda' and 'CUDA' not in active_provider:
                log.warning('CUDA requested but not available — running on CPU')
        except Exception as e:
            log.warning('ONNX loading failed (%s) — fallback to PyTorch', e)
            self._use_onnx = False

    # ── PyTorch path ───────────────────────────────────────────
    def _load_pytorch(self, model_path: str):
        from ultralytics import YOLO
        self._model = YOLO(model_path)
        self._model.to(self._device)
        self._names = self._model.names
        log.info('PyTorch YOLO loaded: %s on %s',
                 os.path.basename(model_path), self._device)

    # ── Backend interface ──────────────────────────────────────
    def infer(self, ctx: FrameContext) -> list[Detection]:
        if self._frame_drop and self._processing:
            return []
        self._processing = True
        try:
            return self._do_infer(ctx)
        finally:
            self._processing = False

    def _do_infer(self, ctx: FrameContext) -> list[Detection]:
        if self._use_onnx:
            raw_boxes = self._infer_onnx(ctx.bgr)
        else:
            raw_boxes = self._infer_pytorch(ctx.bgr)

        detections = []
        for (x1, y1, x2, y2, conf, cls_id) in raw_boxes:
            x1 = max(0, x1); y1 = max(0, y1)
            x2 = min(ctx.width, x2); y2 = min(ctx.height, y2)
            w = x2 - x1; h = y2 - y1
            if w < 5 or h < 5:
                continue
            cls_name = self._names.get(cls_id, 'object')
            mapped_cls, mapped_colour = _map_cls_to_cls_colour(cls_name)
            detections.append(Detection(
                cls=mapped_cls,
                colour=mapped_colour,
                x=int(x1), y=int(y1), w=int(w), h=int(h),
                conf=round(float(conf), 3),
            ))
        return detections

    def _infer_pytorch(self, frame: np.ndarray):
        results = self._model(frame, conf=self._conf, verbose=False)
        boxes = []
        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                boxes.append((x1, y1, x2, y2,
                              float(box.conf[0]), int(box.cls[0])))
        return boxes

    def _infer_onnx(self, frame: np.ndarray):
        import cv2
        h, w = frame.shape[:2]
        img = cv2.resize(frame, (self._imgsz, self._imgsz))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        img = np.transpose(img, (2, 0, 1))[np.newaxis]
        outputs = self._ort.run(None, {self._input_name: img})[0]
        boxes = []
        preds = outputs[0].T
        for pred in preds:
            cx, cy, bw, bh = pred[:4]
            scores = pred[4:]
            cls_id = int(np.argmax(scores))
            conf = float(scores[cls_id])
            if conf < self._conf:
                continue
            x1 = int((cx - bw / 2) / self._imgsz * w)
            y1 = int((cy - bh / 2) / self._imgsz * h)
            x2 = int((cx + bw / 2) / self._imgsz * w)
            y2 = int((cy + bh / 2) / self._imgsz * h)
            boxes.append((x1, y1, x2, y2, conf, cls_id))
        return boxes

    @property
    def class_names(self) -> dict[int, str]:
        return dict(self._names)

    @property
    def using_onnx(self) -> bool:
        return self._use_onnx
