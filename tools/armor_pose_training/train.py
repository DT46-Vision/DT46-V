import argparse
import json
import platform
import random
import sys
import time
from pathlib import Path

import torch
import torchvision
from torch.utils.data import DataLoader

from .data import ArmorCrops, class_catalog, manifest_sha256, read_manifest
from .evaluate import collect_predictions, metrics
from .model import ArmorPose, training_loss


def train_epoch(model: ArmorPose, loader: DataLoader, optimizer: torch.optim.Optimizer,
                scaler: torch.amp.GradScaler, device: torch.device, amp: bool) -> dict:
    model.train()
    loss_sum, samples, skipped = 0.0, 0, 0
    for batch in loader:
        batch = {k: v.to(device, non_blocking=True) for k, v in batch.items()}
        optimizer.zero_grad(set_to_none=True)
        with torch.autocast(device_type=device.type, dtype=torch.float16, enabled=amp):
            output = model(batch["image"], batch["camera"])
        # Metric geometry and loss reductions stay FP32 even during AMP training.
        loss, _ = training_loss(output, batch)
        if not torch.isfinite(loss):
            raise RuntimeError("Non-finite training loss")
        scaler.scale(loss).backward()
        scaler.unscale_(optimizer)
        torch.nn.utils.clip_grad_norm_(model.parameters(), 5.0)
        old_scale = scaler.get_scale()
        scaler.step(optimizer)
        scaler.update()
        skipped += scaler.get_scale() < old_scale
        count = len(batch["image"])
        loss_sum += float(loss.detach()) * count
        samples += count
    if skipped == len(loader):
        raise RuntimeError("All optimizer steps were skipped")
    return {"loss": loss_sum / samples, "samples": samples, "skipped_amp_steps": skipped}


def save_checkpoint(path: Path, state: dict) -> None:
    temporary = path.with_suffix(".tmp")
    torch.save(state, temporary)
    temporary.replace(path)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Train a small direct armor pose model on local GPU")
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--epochs", type=int, default=20, help="Total epoch count, including resumed epochs")
    parser.add_argument("--batch-size", type=int, default=128)
    parser.add_argument("--workers", type=int, default=0)
    parser.add_argument("--learning-rate", type=float, default=0.0003)
    parser.add_argument("--weight-decay", type=float, default=0.0001)
    parser.add_argument("--seed", type=int, default=20261030)
    parser.add_argument("--device", choices=["cuda", "cpu"], default="cuda")
    parser.add_argument("--amp", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--resume", type=Path)
    args = parser.parse_args()
    if args.epochs < 1 or args.batch_size < 2 or args.workers < 0 or args.learning_rate <= 0 or args.weight_decay < 0:
        parser.error("Invalid training settings")
    return args


def initialize_training(args: argparse.Namespace) -> tuple[torch.device, bool]:
    if args.device == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("CUDA was requested but is unavailable; no silent CPU fallback")
    device = torch.device(args.device)
    torch.set_num_threads(4)
    torch.manual_seed(args.seed)
    random.seed(args.seed)
    return device, args.amp and device.type == "cuda"


def main() -> None:
    args = parse_args()
    device, amp = initialize_training(args)
    args.manifest = args.manifest.resolve()
    args.output = args.output.resolve()
    rows = read_manifest(args.manifest)
    catalog = class_catalog(rows)
    classes = list(catalog)
    digest = manifest_sha256(args.manifest)
    train_data = ArmorCrops(args.manifest, rows, "train", classes)
    validation_data = ArmorCrops(args.manifest, rows, "validation", classes)
    generator = torch.Generator().manual_seed(args.seed)
    model = ArmorPose(len(classes)).to(device)
    optimizer = torch.optim.AdamW(model.parameters(), lr=args.learning_rate, weight_decay=args.weight_decay)
    scaler = torch.amp.GradScaler("cuda", enabled=amp)
    start_epoch, best_loss = 1, float("inf")
    if args.resume:
        state = torch.load(args.resume, map_location="cpu", weights_only=True)
        if args.resume.resolve() != args.output / "last.pt" or not (args.output / "best.pt").exists():
            raise ValueError("Resume last.pt in the original run directory, preserving best.pt")
        if state["manifest_sha256"] != digest or state["classes"] != classes:
            raise ValueError("Resume dataset or class vocabulary differs")
        if state["amp"] != amp or state["batch_size"] != args.batch_size:
            raise ValueError("Resume with the original AMP and batch settings")
        model.load_state_dict(state["model"])
        optimizer.load_state_dict(state["optimizer"])
        scaler.load_state_dict(state["scaler"])
        generator.set_state(state["loader_rng"])
        torch.set_rng_state(state["torch_rng"])
        if device.type == "cuda":
            torch.cuda.set_rng_state_all(state["cuda_rng"])
        start_epoch, best_loss = state["epoch"] + 1, state["best_loss"]
        if args.epochs < start_epoch:
            raise ValueError("Requested epoch count has already been completed")
    else:
        args.output.mkdir(parents=True, exist_ok=False)
    train_loader = DataLoader(train_data, batch_size=args.batch_size, shuffle=True, generator=generator,
                              num_workers=args.workers, pin_memory=device.type == "cuda")
    validation_loader = DataLoader(validation_data, batch_size=args.batch_size, num_workers=args.workers)
    info = {
        "command": sys.argv, "python": platform.python_version(), "torch": str(torch.__version__),
        "torchvision": str(torchvision.__version__), "cuda_runtime": torch.version.cuda,
        "device": torch.cuda.get_device_name() if device.type == "cuda" else "cpu", "amp": amp,
        "parameter_count": sum(p.numel() for p in model.parameters()), "classes": classes, "catalog": catalog,
        "manifest": str(args.manifest), "manifest_sha256": digest,
        "train_samples": len(train_data), "validation_samples": len(validation_data),
        "options": {k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()},
        "optimizer": {k: v for k, v in optimizer.param_groups[0].items() if k != "params"},
    }
    info_name = "environment.json" if not args.resume else f"resume_epoch_{start_epoch}.json"
    (args.output / info_name).write_text(json.dumps(info, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(info), flush=True)
    for epoch in range(start_epoch, args.epochs + 1):
        if device.type == "cuda":
            torch.cuda.reset_peak_memory_stats()
        started = time.perf_counter()
        training = train_epoch(model, train_loader, optimizer, scaler, device, amp)
        validation = metrics(*collect_predictions(model, validation_loader, device))
        elapsed = time.perf_counter() - started
        improved = validation["loss"] < best_loss
        best_loss = min(best_loss, validation["loss"])
        state = {
            "format_version": 1, "model": model.state_dict(), "optimizer": optimizer.state_dict(),
            "scaler": scaler.state_dict(), "classes": classes, "catalog": catalog,
            "manifest_sha256": digest, "epoch": epoch, "best_loss": best_loss,
            "amp": amp, "batch_size": args.batch_size, "loader_rng": generator.get_state(),
            "torch_rng": torch.get_rng_state(),
            "cuda_rng": torch.cuda.get_rng_state_all() if device.type == "cuda" else [],
        }
        save_checkpoint(args.output / "last.pt", state)
        if improved:
            save_checkpoint(args.output / "best.pt", state)
        record = {"epoch": epoch, "seconds": elapsed, "train": training, "validation": validation,
                  "best": improved, "cuda_peak_allocated_mib":
                  torch.cuda.max_memory_allocated() / 1024**2 if device.type == "cuda" else 0}
        with (args.output / "history.jsonl").open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(record, allow_nan=False) + "\n")
        print(json.dumps({"epoch": epoch, "seconds": round(elapsed, 2), "train_loss": training["loss"],
                          "validation_loss": validation["loss"], "best": improved,
                          "depth_p95_m": validation["pose_all_supervised"]["depth_absolute_p95_m"]}), flush=True)


if __name__ == "__main__":
    main()
