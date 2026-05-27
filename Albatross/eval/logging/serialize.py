from __future__ import annotations

from dataclasses import asdict, is_dataclass
from pathlib import Path
from typing import Any
import json
import time

import numpy as np
import cv2


def jsonable(obj: Any) -> Any:
    if is_dataclass(obj):
        return {k: jsonable(v) for k, v in asdict(obj).items()}

    if isinstance(obj, np.ndarray):
        return obj.tolist()

    if isinstance(obj, (np.floating,)):
        return float(obj)

    if isinstance(obj, (np.integer,)):
        return int(obj)

    if isinstance(obj, dict):
        return {str(k): jsonable(v) for k, v in obj.items()}

    if isinstance(obj, (list, tuple)):
        return [jsonable(x) for x in obj]

    return obj


def save_frame_image(run_dir: Path, seq: int, image_bgr: np.ndarray) -> str:
    frames_dir = run_dir / "frames"
    frames_dir.mkdir(parents=True, exist_ok=True)
    rel_path = f"frames/frame_{seq:06d}.jpg"
    abs_path = run_dir / rel_path
    cv2.imwrite(str(abs_path), image_bgr)
    return rel_path


def save_npy_array(run_dir: Path, subdir: str, seq: int, arr: np.ndarray) -> str:
    out_dir = run_dir / subdir
    out_dir.mkdir(parents=True, exist_ok=True)
    rel_path = f"{subdir}/{subdir}_{seq:06d}.npy"
    abs_path = run_dir / rel_path
    np.save(abs_path, arr)
    return rel_path