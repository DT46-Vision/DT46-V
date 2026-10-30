import json
import tempfile
import unittest
from pathlib import Path

import numpy as np
import torch
from PIL import Image

from .data import ArmorCrops, FRONTAL_AXES, camera_features, read_manifest
from .model import rotation_matrix, training_loss


def sample(image: str = "crop.png", split: str = "train", group: str = "scene") -> dict:
    return {
        "image": image, "split": split, "scene_group": group, "sample_id": image,
        "K": [[800, 0, 64], [0, 800, 64], [0, 0, 1]], "D": [0] * 5,
        "is_armor": True, "supervision": {k: True for k in ("is_armor", "pose", "sticker", "team")},
        "target": {"sticker_type": "infantry_3", "team": "red", "pose":
                   {"position_camera_m": [0, 0, 2], "rotation_6d": FRONTAL_AXES}},
    }


class TrainingContractTests(unittest.TestCase):
    def test_scene_leakage_and_duplicate_images_are_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for name in ("a.png", "b.png"):
                Image.new("RGB", (128, 128)).save(root / name)
            path = root / "training.jsonl"
            rows = [sample("a.png"), sample("b.png", "validation")]
            path.write_text("\n".join(map(json.dumps, rows)), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "crosses splits"):
                read_manifest(path)
            rows[1] = sample("a.png", "train", "other")
            path.write_text("\n".join(map(json.dumps, rows)), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "Duplicate image"):
                read_manifest(path)

    def test_targets_do_not_change_network_inputs(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            Image.new("RGB", (128, 128), (20, 60, 90)).save(root / "crop.png")
            row = sample()
            data = ArmorCrops(root / "training.jsonl", [row], "train", ["infantry_3"])
            first = data[0]
            row["target"]["pose"]["position_camera_m"] = [3, -2, 7]
            second = data[0]
            self.assertTrue(torch.equal(first["image"], second["image"]))
            self.assertTrue(torch.equal(first["camera"], second["camera"]))
            self.assertFalse(torch.equal(first["position"], second["position"]))

    def test_negative_samples_only_supervise_presence(self) -> None:
        outputs = tuple(torch.tensor(value, dtype=torch.float32, requires_grad=True) for value in (
            [0.3], [[0.1, 0.2]], [[0.4, 0.1]], [[0.3, 0.2, 3.0]], [[0.8, 0.1, 0, 0, -0.7, 0.1]],
        ))
        batch = {"is_armor": torch.zeros(1), "sticker": torch.zeros(1, dtype=torch.long),
                 "team": torch.zeros(1, dtype=torch.long), "position": torch.tensor([[0., 0., 1.]]),
                 "rotation": torch.tensor([FRONTAL_AXES]),
                 **{f"{name}_mask": torch.zeros(1, dtype=torch.bool) for name in ("pose", "sticker", "team")}}
        loss, _ = training_loss(outputs, batch)
        loss.backward()
        self.assertGreater(float(outputs[0].grad.abs().sum()), 0)
        for tensor in outputs[1:]:
            self.assertEqual(float(tensor.grad.abs().sum()), 0)

    def test_calibration_retains_scale_and_rotation_columns_match_contract(self) -> None:
        a = camera_features([[800, 0, 64], [0, 800, 64], [0, 0, 1]], [0] * 5)
        b = camera_features([[400, 0, 64], [0, 400, 64], [0, 0, 1]], [0] * 5)
        np.testing.assert_allclose(a[:2] - b[:2], np.log(2), atol=1e-6)
        rotation, valid = rotation_matrix(torch.tensor([FRONTAL_AXES, [1., 0, 0, 2, 0, 0]]))
        torch.testing.assert_close(rotation[0], torch.diag(torch.tensor([1., -1., -1.])))
        self.assertEqual(valid.tolist(), [True, False])


if __name__ == "__main__":
    unittest.main()
