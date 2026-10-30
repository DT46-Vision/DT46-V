"""Read the simulator's selected crop manifest without exposing targets as inputs."""

import hashlib
import json
from pathlib import Path

import numpy as np
import torch
from PIL import Image
from torch.utils.data import Dataset


IMAGE_SIZE = 128
TEAMS = ["red", "blue"]
FRONTAL_AXES = [1.0, 0.0, 0.0, 0.0, -1.0, 0.0]


def read_manifest(path: Path) -> list[dict]:
    rows = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
    groups: dict[str, str] = {}
    images: set[str] = set()
    for row in rows:
        split, group = row["split"], row["scene_group"]
        if split not in {"train", "validation", "test"}:
            raise ValueError(f"Unknown split: {split}")
        if groups.setdefault(group, split) != split:
            raise ValueError(f"Scene group crosses splits: {group}")
        image = str((path.parent / row["image"]).resolve())
        if image in images:
            raise ValueError(f"Duplicate image: {image}")
        images.add(image)
        if not Path(image).is_file():
            raise ValueError(f"Missing image: {image}")
        masks = row["supervision"]
        if not masks["is_armor"]:
            raise ValueError("This trainer requires an armor-presence label")
        if not row["is_armor"] and (row["target"] is not None or any(masks[k] for k in ("pose", "sticker", "team"))):
            raise ValueError("Negative samples may only supervise armor presence")
        if row["is_armor"] and row["target"] is None:
            raise ValueError("Positive sample is missing its target")
    if not rows:
        raise ValueError("Empty manifest")
    return rows


def class_catalog(rows: list[dict]) -> dict:
    catalog: dict[str, dict] = {}
    for row in rows:
        if row["split"] != "train" or not row["is_armor"]:
            continue
        target = row["target"]
        name = target["sticker_type"]
        details = {k: target[k] for k in ("target_family", "panel_size", "geometry")}
        if name in catalog and catalog[name] != details:
            raise ValueError(f"Sticker {name} spans different geometry; this baseline needs separate size supervision")
        catalog[name] = details
    if not catalog:
        raise ValueError("No positive training samples")
    return dict(sorted(catalog.items()))


def camera_features(k: list, distortion: list) -> np.ndarray:
    matrix = np.asarray(k, dtype=np.float32)
    d = np.asarray(distortion, dtype=np.float32)
    if matrix.shape != (3, 3) or d.shape != (5,):
        raise ValueError("Expected a 3x3 crop K and five OpenCV distortion coefficients")
    if not np.isfinite(matrix).all() or not np.isfinite(d).all() or min(matrix[0, 0], matrix[1, 1]) <= 0:
        raise ValueError("Invalid camera calibration")
    if not np.allclose(matrix[2], [0, 0, 1]) or abs(matrix[0, 1]) > 1e-6 or abs(matrix[1, 0]) > 1e-6:
        raise ValueError("Only the current zero-skew camera contract is supported")
    # Cropping scale is essential for metric depth; a resized image alone loses it.
    return np.concatenate((
        np.log([matrix[0, 0] / IMAGE_SIZE, matrix[1, 1] / IMAGE_SIZE]),
        [matrix[0, 2] / IMAGE_SIZE, matrix[1, 2] / IMAGE_SIZE], d,
    )).astype(np.float32)


def image_tensor(path: Path) -> torch.Tensor:
    with Image.open(path) as image:
        if image.size != (IMAGE_SIZE, IMAGE_SIZE):
            raise ValueError(f"Expected a 128x128 saved crop, got {image.size}: {path}")
        pixels = np.array(image.convert("RGB"), dtype=np.float32) / 255.0
    return torch.from_numpy(pixels.transpose(2, 0, 1).copy())


class ArmorCrops(Dataset):
    def __init__(self, manifest: Path, rows: list[dict], split: str, classes: list[str]):
        self.root = manifest.parent
        self.rows = [r for r in rows if r["split"] == split]
        self.classes = classes
        if not self.rows:
            raise ValueError(f"No samples in split {split}")
        for row in self.rows:
            if row["is_armor"] and row["target"]["sticker_type"] not in classes:
                raise ValueError("Validation/test contains a class absent from training")

    def __len__(self) -> int:
        return len(self.rows)

    def __getitem__(self, index: int) -> dict[str, torch.Tensor]:
        row = self.rows[index]
        masks = row["supervision"]
        target = row["target"]
        position, rotation = [0.0, 0.0, 1.0], FRONTAL_AXES
        sticker, team = 0, 0
        if target is not None:
            position = target["pose"]["position_camera_m"]
            rotation = target["pose"]["rotation_6d"]
            sticker = self.classes.index(target["sticker_type"])
            team = TEAMS.index(target["team"])
            if not np.isfinite(position).all() or position[2] <= 0 or not np.isfinite(rotation).all():
                raise ValueError(f"Invalid target: {row['sample_id']}")
        return {
            "image": image_tensor(self.root / row["image"]),
            "camera": torch.from_numpy(camera_features(row["K"], row["D"])),
            "is_armor": torch.tensor(float(row["is_armor"])),
            "position": torch.tensor(position, dtype=torch.float32),
            "rotation": torch.tensor(rotation, dtype=torch.float32),
            "sticker": torch.tensor(sticker), "team": torch.tensor(team),
            **{f"{name}_mask": torch.tensor(bool(masks[name])) for name in ("pose", "sticker", "team")},
        }


def manifest_sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()
