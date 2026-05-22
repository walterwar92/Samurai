#!/usr/bin/env python3
"""
tools/train_balls.py — fine-tune yolo11n.pt на 3 класса мячей (red/green/blue).

Использование:
    python tools/train_balls.py
    python tools/train_balls.py --epochs 100 --imgsz 640
    python tools/train_balls.py --device cuda  # если есть NVIDIA GPU

Ожидаемая структура датасета (после collect_frames.py + разметки):

    dataset/raw/red/*.jpg     ← фото красных мячей
    dataset/raw/red/*.txt     ← разметка YOLO (parallel basename)
    dataset/raw/green/*.jpg
    dataset/raw/green/*.txt
    dataset/raw/blue/*.jpg
    dataset/raw/blue/*.txt

Формат .txt (YOLO): одна строка на объект
    <class_id> <cx> <cy> <w> <h>
где class_id из dataset/samurai_balls.yaml (0=red, 1=green, 2=blue),
координаты нормализованы в [0,1] относительно размера изображения.

Что делает скрипт:
1. Валидирует датасет (число пар jpg+txt, баланс классов)
2. Делит 80/20 train/val с фиксированным seed (повторяемость)
3. Копирует в dataset/images/{train,val} + dataset/labels/{train,val}
4. Запускает yolo train (fine-tune от yolo11n.pt)
5. Копирует best.pt в корень проекта как samurai_balls.pt

После завершения:
    ./samurai.sh detector --pi 192.168.4.1 --model samurai_balls.pt
"""
from __future__ import annotations

import argparse
import os
import random
import shutil
import sys
from pathlib import Path

# Bootstrap project root
_THIS = Path(__file__).resolve()
_ROOT = _THIS.parent.parent

# Workarounds (см. compute_node/detector.py) — должны быть ДО любого
# импорта ultralytics/torch.
if os.name == 'nt':
    if 'YOLO_CONFIG_DIR' not in os.environ:
        try:
            os.path.expanduser('~').encode('ascii')
        except (UnicodeEncodeError, UnicodeDecodeError):
            _cache = 'C:/yolo_cache'
            os.makedirs(_cache, exist_ok=True)
            os.environ['YOLO_CONFIG_DIR'] = _cache
    if not any(os.environ.get(v) for v in ('USERNAME', 'USER', 'LOGNAME', 'LNAME')):
        os.environ['USERNAME'] = 'samurai'

DATASET = _ROOT / 'dataset'
RAW = DATASET / 'raw'
IMAGES_TRAIN = DATASET / 'images' / 'train'
IMAGES_VAL = DATASET / 'images' / 'val'
LABELS_TRAIN = DATASET / 'labels' / 'train'
LABELS_VAL = DATASET / 'labels' / 'val'

CLASSES = ('red', 'green', 'blue')
SPLIT_SEED = 42


def collect_pairs() -> list[tuple[Path, Path]]:
    """Собирает [(jpg, txt), ...] для всех размеченных кадров.

    Фото без парного .txt (того же basename) — пропускаются с warning'ом.
    """
    pairs: list[tuple[Path, Path]] = []
    skipped = 0
    for cls in CLASSES:
        d = RAW / cls
        if not d.exists():
            print(f'WARN: папка {d} не существует — пропуск класса {cls}')
            continue
        for jpg in sorted(d.glob('*.jpg')):
            txt = jpg.with_suffix('.txt')
            if txt.exists():
                pairs.append((jpg, txt))
            else:
                skipped += 1
    if skipped:
        print(f'WARN: пропущено {skipped} .jpg без парного .txt — '
              f'разметь их в Roboflow/LabelImg или удали')
    return pairs


def split_and_copy(pairs: list[tuple[Path, Path]], val_frac: float = 0.2) -> tuple[int, int]:
    """Очищает и наполняет dataset/{images,labels}/{train,val}. Возвращает (n_train, n_val)."""
    for d in (IMAGES_TRAIN, IMAGES_VAL, LABELS_TRAIN, LABELS_VAL):
        if d.exists():
            shutil.rmtree(d)
        d.mkdir(parents=True, exist_ok=True)

    random.seed(SPLIT_SEED)
    shuffled = pairs[:]
    random.shuffle(shuffled)

    n_val = max(1, int(len(shuffled) * val_frac))
    val_pairs = shuffled[:n_val]
    train_pairs = shuffled[n_val:]

    for jpg, txt in train_pairs:
        shutil.copy2(jpg, IMAGES_TRAIN / jpg.name)
        shutil.copy2(txt, LABELS_TRAIN / txt.name)
    for jpg, txt in val_pairs:
        shutil.copy2(jpg, IMAGES_VAL / jpg.name)
        shutil.copy2(txt, LABELS_VAL / txt.name)

    return len(train_pairs), len(val_pairs)


def main() -> int:
    parser = argparse.ArgumentParser(
        description='Fine-tune yolo11n.pt на 3 цвета мячей',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument('--epochs', type=int, default=50)
    parser.add_argument('--imgsz', type=int, default=416,
                        help='Размер входа модели (default: 416)')
    parser.add_argument('--batch', type=int, default=8,
                        help='Batch size (8 для CPU, 16+ для GPU)')
    parser.add_argument('--device', default='cpu',
                        help='cpu или cuda (default: cpu)')
    parser.add_argument('--model', default='yolo11n.pt',
                        help='Базовая модель для fine-tune (default: yolo11n.pt)')
    parser.add_argument('--patience', type=int, default=10,
                        help='Early stopping patience epochs (default: 10)')
    parser.add_argument('--val-frac', type=float, default=0.2,
                        help='Доля validation set (default: 0.2)')
    parser.add_argument('--skip-split', action='store_true',
                        help='Не пересобирать train/val split (использовать существующий)')
    parser.add_argument('--name', default='balls_train',
                        help='Имя run-папки в runs/ (default: balls_train)')
    args = parser.parse_args()

    # Шаг 1: валидация датасета
    print('Step 1: validating dataset ...')
    pairs = collect_pairs()
    if len(pairs) < 30:
        print(f'ERROR: только {len(pairs)} размеченных кадров. Минимум 30, '
              f'рекомендую 300+ (по 100 на класс).')
        print('Сначала запусти tools/collect_frames.py и размечь в Roboflow или LabelImg.')
        return 1

    counts = {cls: 0 for cls in CLASSES}
    for jpg, _ in pairs:
        cls = jpg.parent.name
        if cls in counts:
            counts[cls] += 1
    print(f'  Total pairs: {len(pairs)}')
    print(f'  By class: red={counts["red"]} green={counts["green"]} blue={counts["blue"]}')

    min_cls = min(counts.values())
    if min_cls < 10:
        print(f'WARN: минимум {min_cls} фото в одном из классов. Желательно ≥50 каждого.')
    max_min_ratio = max(counts.values()) / max(1, min_cls)
    if max_min_ratio > 3.0:
        print(f'WARN: дисбаланс классов {max_min_ratio:.1f}× — модель будет смещена. '
              f'Доберите фото слабого класса.')

    # Шаг 2: train/val split
    if args.skip_split:
        print('Step 2: skipping split (--skip-split)')
        n_train = len(list(IMAGES_TRAIN.glob('*.jpg'))) if IMAGES_TRAIN.exists() else 0
        n_val = len(list(IMAGES_VAL.glob('*.jpg'))) if IMAGES_VAL.exists() else 0
        if n_train == 0 or n_val == 0:
            print(f'ERROR: --skip-split, но {IMAGES_TRAIN}/{IMAGES_VAL} пусты. '
                  f'Запусти без --skip-split.')
            return 1
    else:
        print('Step 2: splitting train/val ...')
        n_train, n_val = split_and_copy(pairs, val_frac=args.val_frac)
    print(f'  Split: {n_train} train / {n_val} val')

    # Шаг 3: тренировка. CWD = project root чтобы 'path: dataset' в yaml совпал.
    os.chdir(_ROOT)
    print(f'\nStep 3: training (CWD={_ROOT})')
    print(f'  Model: {args.model} | Epochs: {args.epochs} | Imgsz: {args.imgsz} '
          f'| Batch: {args.batch} | Device: {args.device}')
    print('  Ctrl+C прерывает — последний чекпойнт сохранится в runs/.\n')

    from ultralytics import YOLO  # type: ignore
    model = YOLO(args.model)
    model.train(
        data=str(DATASET / 'samurai_balls.yaml'),
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        optimizer='AdamW',
        patience=args.patience,
        project='runs',
        name=args.name,
        exist_ok=True,
        verbose=True,
    )

    # Шаг 4: копируем best.pt в корень
    save_dir = Path(model.trainer.save_dir)
    best = save_dir / 'weights' / 'best.pt'
    if not best.exists():
        print(f'\nWARN: best.pt не найден в {save_dir / "weights"}. Смотри метрики выше.')
        return 1

    out = _ROOT / 'samurai_balls.pt'
    shutil.copy2(best, out)
    print(f'\n✓ Модель готова: {out}')
    print(f'  Запусти детектор: ./samurai.sh detector --pi 192.168.4.1 --model samurai_balls.pt')

    return 0


if __name__ == '__main__':
    sys.exit(main())
