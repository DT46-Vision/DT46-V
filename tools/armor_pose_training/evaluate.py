import argparse
import json
from pathlib import Path

import torch
from torch.utils.data import DataLoader

from .data import ArmorCrops, manifest_sha256, read_manifest
from .model import ArmorPose, rotation_matrix, training_loss


def pose_metrics(position: torch.Tensor, axes: torch.Tensor, target: dict, mask: torch.Tensor) -> dict:
    count = int(mask.sum())
    if not count:
        return {"count": 0}
    error = position[mask] - target["position"][mask]
    depth = error[:, 2]
    predicted_r, valid = rotation_matrix(axes[mask])
    true_r, _ = rotation_matrix(target["rotation"][mask])
    cosine = ((predicted_r * true_r).sum((1, 2)) - 1) / 2
    angles = torch.rad2deg(torch.acos(cosine[valid].clamp(-1, 1)))
    return {
        "count": count, "invalid_rotation_count": int((~valid).sum()),
        "depth_bias_m": float(depth.mean()), "depth_std_m": float(depth.std(unbiased=False)),
        "depth_absolute_p50_m": float(depth.abs().quantile(0.5)),
        "depth_absolute_p95_m": float(depth.abs().quantile(0.95)),
        "translation_rmse_m": float(error.square().sum(1).mean().sqrt()),
        "lateral_rmse_m": float(error[:, :2].square().sum(1).mean().sqrt()),
        "rotation_mean_deg": float(angles.mean()) if len(angles) else None,
        "rotation_p95_deg": float(angles.quantile(0.95)) if len(angles) else None,
    }


def metrics(outputs: tuple, target: dict) -> dict:
    armor, sticker, team, position, axes = outputs
    predicted = armor.sigmoid() >= 0.5
    positive = target["is_armor"].bool()
    tp = int((predicted & positive).sum())
    fp = int((predicted & ~positive).sum())
    fn = int((~predicted & positive).sum())
    total, terms = training_loss(outputs, target)
    result = {
        "count": len(armor), "loss": float(total),
        "loss_terms": {k: float(v) for k, v in terms.items()},
        "presence": {"threshold": 0.5, "tp": tp, "fp": fp, "fn": fn,
                     "tn": int((~predicted & ~positive).sum()),
                     "precision": tp / max(tp + fp, 1), "recall": tp / max(tp + fn, 1)},
        "pose_all_supervised": pose_metrics(position, axes, target, target["pose_mask"]),
    }
    for name, logits in (("sticker", sticker), ("team", team)):
        mask = target[f"{name}_mask"]
        result[name] = {"count": int(mask.sum()), "accuracy":
                        float((logits.argmax(1)[mask] == target[name][mask]).float().mean()) if mask.any() else None}
    r, _ = rotation_matrix(target["rotation"])
    ray = -torch.nn.functional.normalize(target["position"], dim=1)
    incidence = torch.rad2deg(torch.acos((r[:, :, 2] * ray).sum(1).clamp(-1, 1)))
    for name, value, edges in (("depth_bins_m", target["position"][:, 2], [0, 2, 4, 8, float("inf")]),
                               ("incidence_bins_deg", incidence, [0, 20, 40, 60, 181])):
        result[name] = {f"[{low},{high})": pose_metrics(position, axes, target,
                       target["pose_mask"] & (value >= low) & (value < high))
                       for low, high in zip(edges, edges[1:])}
    return result


@torch.inference_mode()
def collect_predictions(model: ArmorPose, loader: DataLoader, device: torch.device) -> tuple[tuple, dict]:
    model.eval()
    predictions: list[list[torch.Tensor]] = [[] for _ in range(5)]
    targets: dict[str, list[torch.Tensor]] = {}
    for batch in loader:
        output = model(batch["image"].to(device), batch["camera"].to(device))
        if not all(torch.isfinite(x).all() for x in output):
            raise RuntimeError("Non-finite model output")
        for values, tensor in zip(predictions, output):
            values.append(tensor.float().cpu())
        for name, tensor in batch.items():
            if name not in {"image", "camera"}:
                targets.setdefault(name, []).append(tensor)
    return tuple(torch.cat(x) for x in predictions), {k: torch.cat(v) for k, v in targets.items()}


def load_model(checkpoint: Path, device: torch.device) -> tuple[ArmorPose, dict]:
    state = torch.load(checkpoint, map_location="cpu", weights_only=True)
    if state["format_version"] != 1:
        raise ValueError("Unsupported checkpoint format")
    model = ArmorPose(len(state["classes"]))
    model.load_state_dict(state["model"])
    return model.to(device).eval(), state


def main() -> None:
    parser = argparse.ArgumentParser(description="Score saved crop predictions, including rejected positives")
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--split", choices=["validation", "test"], default="test")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--device", choices=["cuda", "cpu"], default="cuda")
    parser.add_argument("--batch-size", type=int, default=128)
    args = parser.parse_args()
    torch.set_num_threads(4)
    device = torch.device(args.device)
    model, state = load_model(args.checkpoint, device)
    rows = read_manifest(args.manifest)
    data = ArmorCrops(args.manifest, rows, args.split, state["classes"])
    result = metrics(*collect_predictions(model, DataLoader(data, batch_size=args.batch_size), device))
    result.update(split=args.split, checkpoint_epoch=state["epoch"],
                  manifest_sha256=manifest_sha256(args.manifest),
                  training_manifest_sha256=state["manifest_sha256"],
                  scope="Saved crops; pose metrics include all supervised positives, independent of presence threshold")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(json.dumps(result, allow_nan=False), flush=True)


if __name__ == "__main__":
    main()
