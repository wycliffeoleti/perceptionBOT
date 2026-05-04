#!/usr/bin/env python3
"""Download ONNX model files used by perception_bot_perception.

Targets:
  - YOLOv11n  (Ultralytics export)        → yolov11n.onnx
  - Depth Anything V2 ViT-S (HF export)   → depth_anything_v2_vits.onnx

Files land in ``src/perception_bot_perception/models/`` so they're picked up
by ``install(DIRECTORY models/ …)`` and end up in the package share.

Already-present files with matching size are skipped, so the script is
idempotent.

Usage:
  python3 scripts/download_models.py
"""
from __future__ import annotations

import argparse
import os
import sys
import urllib.request
from pathlib import Path
from typing import NamedTuple

ROOT = Path(__file__).resolve().parents[1]
MODELS_DIR = ROOT / "src" / "perception_bot_perception" / "models"


class Model(NamedTuple):
    filename: str
    url: str
    min_bytes: int  # sanity-check size


MODELS = [
    Model(
        filename="yolov11n.onnx",
        url=("https://github.com/ultralytics/assets/releases/download/v8.3.0/"
             "yolo11n.onnx"),
        min_bytes=8_000_000,
    ),
    Model(
        filename="depth_anything_v2_vits.onnx",
        # Public HF mirror of Depth Anything V2 ViT-S ONNX export.
        url=("https://huggingface.co/onnx-community/depth-anything-v2-small/"
             "resolve/main/onnx/model.onnx"),
        min_bytes=80_000_000,
    ),
]


def download(model: Model, dest: Path, *, force: bool) -> None:
    target = dest / model.filename
    if target.exists() and not force:
        size = target.stat().st_size
        if size >= model.min_bytes:
            print(f"[skip]  {model.filename} ({size/1e6:.1f} MB) already present")
            return
        print(f"[redo]  {model.filename} present but only {size} B "
              f"(< {model.min_bytes}); redownloading")

    print(f"[fetch] {model.filename}")
    print(f"        ← {model.url}")
    target.parent.mkdir(parents=True, exist_ok=True)
    tmp = target.with_suffix(target.suffix + ".part")
    with urllib.request.urlopen(model.url) as r, tmp.open("wb") as f:
        while chunk := r.read(1 << 20):
            f.write(chunk)
    tmp.rename(target)
    size = target.stat().st_size
    if size < model.min_bytes:
        target.unlink(missing_ok=True)
        raise SystemExit(f"Downloaded {model.filename} but size {size} "
                         f"< expected {model.min_bytes}")
    print(f"[ok]    {model.filename} ({size/1e6:.1f} MB)")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--force", action="store_true",
                        help="re-download even if file exists")
    parser.add_argument("--only", choices=[m.filename for m in MODELS],
                        help="download only one model")
    args = parser.parse_args()

    targets = MODELS
    if args.only:
        targets = [m for m in MODELS if m.filename == args.only]

    MODELS_DIR.mkdir(parents=True, exist_ok=True)
    for m in targets:
        download(m, MODELS_DIR, force=args.force)
    print(f"\nModels saved to {MODELS_DIR}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
