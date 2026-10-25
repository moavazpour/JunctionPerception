"""Ground-aware monocular 3D detector (CenterNet-style, stride 4).

All outputs are image-space and calibration-free; 3D lifting happens outside the network using the
camera intrinsics and ground plane, so the same engine works for any camera mounting.
"""
from __future__ import annotations

import math

import timm
import torch
import torch.nn as nn
import torch.nn.functional as F

from .dataset import NUM_BINS, MEAN_DIMS
from .geometry import CLASSES

HEADS = {"hm": len(CLASSES), "off": 2, "ltrb": 4, "dz": 1, "dim": 3, "rot": 2 * NUM_BINS}


def conv_bn(cin, cout, k=3):
    return nn.Sequential(nn.Conv2d(cin, cout, k, padding=k // 2, bias=False), nn.BatchNorm2d(cout), nn.ReLU(inplace=True))


class Neck(nn.Module):
    """Top-down FPN fused down to stride 4."""

    def __init__(self, in_chs: list[int], ch: int = 128):
        super().__init__()
        self.lat = nn.ModuleList([nn.Conv2d(c, ch, 1) for c in in_chs])
        self.smooth = nn.ModuleList([conv_bn(ch, ch) for _ in in_chs])

    def forward(self, feats):
        x = self.lat[-1](feats[-1])
        x = self.smooth[-1](x)
        for i in range(len(feats) - 2, -1, -1):
            x = F.interpolate(x, size=feats[i].shape[-2:], mode="bilinear", align_corners=False)
            x = self.smooth[i](x + self.lat[i](feats[i]))
        return x


class GroundCenter3D(nn.Module):
    def __init__(self, backbone: str = "dla34", pretrained: bool = True, ch: int = 128, head_ch: int = 128):
        super().__init__()
        self.backbone = timm.create_model(backbone, pretrained=pretrained, features_only=True)
        info = self.backbone.feature_info
        idx = [i for i, r in enumerate(info.reduction()) if r >= 4]
        self.out_idx = idx
        self.neck = Neck([info.channels()[i] for i in idx], ch)
        self.heads = nn.ModuleDict()
        for name, n in HEADS.items():
            head = nn.Sequential(nn.Conv2d(ch, head_ch, 3, padding=1), nn.ReLU(inplace=True), nn.Conv2d(head_ch, n, 1))
            if name == "hm":
                head[-1].bias.data.fill_(-math.log((1 - 0.01) / 0.01))
            self.heads[name] = head

    def forward(self, x):
        feats = self.backbone(x)
        y = self.neck([feats[i] for i in self.out_idx])
        return {k: h(y) for k, h in self.heads.items()}


class DeployWrapper(nn.Module):
    """Adds sigmoid, max-pool NMS, top-K and decoding so the engine emits a flat detection table.

    Output ``dets`` [B, K, 13]:
        score, class, kp_u, kp_v (input pixels), box_l, box_t, box_r, box_b (input pixels, relative to kp),
        dz (m), l, w, h (m), alpha (rad)
    """

    def __init__(self, model: GroundCenter3D, topk: int = 256, stride: int = 4):
        super().__init__()
        self.model, self.topk, self.stride = model, topk, stride
        self.register_buffer("mean_dims", torch.tensor(MEAN_DIMS))

    def forward(self, x):
        out = self.model(x)
        hm = torch.sigmoid(out["hm"])
        keep = (F.max_pool2d(hm, 3, stride=1, padding=1) == hm).float()
        hm = hm * keep
        B, C, H, W = hm.shape
        scores, inds = torch.topk(hm.reshape(B, -1), self.topk)
        cls = torch.div(inds, H * W, rounding_mode="floor")
        pix = inds - cls * (H * W)
        ys = torch.div(pix, W, rounding_mode="floor").float()
        xs = (pix - torch.div(pix, W, rounding_mode="floor") * W).float()

        def gather(t):
            t = t.reshape(B, t.shape[1], H * W)
            return torch.gather(t, 2, pix.unsqueeze(1).expand(-1, t.shape[1], -1)).permute(0, 2, 1)

        off = gather(out["off"])
        ltrb = gather(out["ltrb"]) * self.stride
        dz = gather(out["dz"])
        dim = torch.exp(gather(out["dim"])) * self.mean_dims[cls]
        rot = gather(out["rot"])
        bins = torch.argmax(rot[..., :NUM_BINS], dim=-1, keepdim=True)
        res = torch.gather(rot[..., NUM_BINS:], 2, bins)
        bw = 2 * math.pi / NUM_BINS
        alpha = (bins.float() + 0.5) * bw + res * (bw / 2) - math.pi
        u = (xs.unsqueeze(-1) + off[..., 0:1]) * self.stride
        v = (ys.unsqueeze(-1) + off[..., 1:2]) * self.stride
        return torch.cat([scores.unsqueeze(-1), cls.float().unsqueeze(-1), u, v, ltrb, dz, dim, alpha], dim=-1)
