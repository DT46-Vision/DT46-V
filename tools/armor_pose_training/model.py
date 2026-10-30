import torch
from torch import nn
from torch.nn import functional as F
from torchvision.models import mobilenet_v3_small

from .data import FRONTAL_AXES


OUTPUT_NAMES = ["armor_logit", "sticker_logits", "team_logits", "position_camera_m", "rotation_6d"]


class ArmorPose(nn.Module):
    def __init__(self, sticker_count: int):
        super().__init__()
        backbone = mobilenet_v3_small(weights=None)
        self.features = backbone.features
        self.pool = nn.AdaptiveAvgPool2d(1)
        self.shared = nn.Sequential(nn.Linear(576 + 9, 128), nn.Hardswish())
        self.armor = nn.Linear(128, 1)
        self.sticker = nn.Linear(128, sticker_count)
        self.team = nn.Linear(128, 2)
        self.position = nn.Linear(128, 3)
        self.rotation = nn.Linear(128, 6)
        self.register_buffer("mean", torch.tensor([0.485, 0.456, 0.406]).reshape(1, 3, 1, 1))
        self.register_buffer("std", torch.tensor([0.229, 0.224, 0.225]).reshape(1, 3, 1, 1))
        nn.init.normal_(self.position.weight, std=0.001)
        nn.init.zeros_(self.position.bias)
        nn.init.normal_(self.rotation.weight, std=0.001)
        with torch.no_grad():
            self.position.bias[2] = 0.69314718056
            self.rotation.bias.copy_(torch.tensor(FRONTAL_AXES))

    def forward(self, image: torch.Tensor, camera: torch.Tensor) -> tuple[torch.Tensor, ...]:
        visual = self.pool(self.features((image - self.mean) / self.std)).flatten(1)
        hidden = self.shared(torch.cat((visual, camera), dim=1))
        encoded = self.position(hidden).float()
        # Positive depth and one shared translation; no redundant angle/range heads.
        depth = torch.exp(encoded[:, 2:3].clamp(-6.0, 6.0))
        position = torch.cat((encoded[:, :2] * depth, depth), dim=1)
        return (self.armor(hidden).squeeze(1), self.sticker(hidden), self.team(hidden),
                position, self.rotation(hidden).float())


def rotation_matrix(axes: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
    a, b = axes.float()[:, :3], axes.float()[:, 3:]
    x = F.normalize(a, dim=1, eps=1e-8)
    orthogonal = b - (x * b).sum(1, keepdim=True) * x
    y = F.normalize(orthogonal, dim=1, eps=1e-8)
    valid = torch.isfinite(axes).all(1) & (a.norm(dim=1) >= 1e-8) & (orthogonal.norm(dim=1) >= 1e-8)
    return torch.stack((x, y, torch.linalg.cross(x, y, dim=1)), dim=2), valid


def masked_mean(values: torch.Tensor, mask: torch.Tensor) -> torch.Tensor:
    weights = mask.to(values.dtype)
    return (values * weights).sum() / weights.sum().clamp_min(1)


def training_loss(outputs: tuple, batch: dict) -> tuple[torch.Tensor, dict[str, torch.Tensor]]:
    armor, sticker, team, position, axes = (x.float() for x in outputs)
    target = batch["position"].float()
    encoded = torch.cat((position[:, :2] / position[:, 2:3], position[:, 2:3].log()), dim=1)
    truth = torch.cat((target[:, :2] / target[:, 2:3], target[:, 2:3].log()), dim=1)
    scales = position.new_tensor([5.0, 5.0, 1.0])
    terms = {
        "armor": F.binary_cross_entropy_with_logits(armor, batch["is_armor"]),
        "sticker": masked_mean(F.cross_entropy(sticker, batch["sticker"], reduction="none"), batch["sticker_mask"]),
        "team": masked_mean(F.cross_entropy(team, batch["team"], reduction="none"), batch["team_mask"]),
        "position": masked_mean(F.smooth_l1_loss(encoded * scales, truth * scales, reduction="none").mean(1), batch["pose_mask"]),
        "rotation": masked_mean(F.mse_loss(axes, batch["rotation"], reduction="none").mean(1), batch["pose_mask"]),
    }
    total = terms["armor"] + terms["sticker"] + terms["team"] + 5 * terms["position"] + 2 * terms["rotation"]
    return total, terms
