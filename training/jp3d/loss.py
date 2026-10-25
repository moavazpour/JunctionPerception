from __future__ import annotations

import torch
import torch.nn.functional as F

from .dataset import NUM_BINS


def focal_loss(logits, gt, ignore):
    """CenterNet penalty-reduced focal loss; ``ignore`` [B,H,W] zeroes negatives at tiny objects."""
    pred = torch.sigmoid(logits.float()).clamp(1e-4, 1 - 1e-4)
    pos = gt.eq(1).float()
    neg = (1 - pos) * (1 - ignore.unsqueeze(1))
    pos_loss = torch.log(pred) * (1 - pred) ** 2 * pos
    neg_loss = torch.log(1 - pred) * pred ** 2 * (1 - gt) ** 4 * neg
    num_pos = pos.sum()
    return -(pos_loss.sum() + neg_loss.sum()) / num_pos.clamp(min=1)


def gather_feat(t, ind):
    B, C = t.shape[:2]
    t = t.reshape(B, C, -1)
    return torch.gather(t, 2, ind.unsqueeze(1).expand(-1, C, -1)).permute(0, 2, 1)


class JP3DLoss:
    def __init__(self, w=None):
        self.w = w or {"hm": 1.0, "off": 1.0, "ltrb": 0.1, "dz": 1.0, "dim": 1.0, "rbin": 0.5, "rres": 1.0}

    def __call__(self, out, t):
        m = t["mask"]
        nm = m.sum().clamp(min=1)
        losses = {"hm": focal_loss(out["hm"], t["hm"], t["ignore"])}
        off = gather_feat(out["off"].float(), t["ind"])
        losses["off"] = (F.l1_loss(off, t["off"], reduction="none").sum(-1) * m).sum() / nm
        ltrb = gather_feat(out["ltrb"].float(), t["ind"])
        losses["ltrb"] = (F.l1_loss(ltrb, t["ltrb"], reduction="none").sum(-1) * m).sum() / nm
        dz = gather_feat(out["dz"].float(), t["ind"])[..., 0]
        losses["dz"] = (F.smooth_l1_loss(dz, t["dz"], reduction="none", beta=0.1) * m).sum() / nm
        dim = gather_feat(out["dim"].float(), t["ind"])
        losses["dim"] = (F.l1_loss(dim, t["dim"], reduction="none").sum(-1) * m).sum() / nm
        rot = gather_feat(out["rot"].float(), t["ind"])
        logits, res = rot[..., :NUM_BINS], rot[..., NUM_BINS:]
        ce = F.cross_entropy(logits.reshape(-1, NUM_BINS), t["rbin"].reshape(-1), reduction="none").reshape(m.shape)
        losses["rbin"] = (ce * m).sum() / nm
        r = torch.gather(res, 2, t["rbin"].unsqueeze(-1))[..., 0]
        losses["rres"] = (F.l1_loss(r, t["rres"], reduction="none") * m).sum() / nm
        total = sum(self.w[k] * v for k, v in losses.items())
        return total, {k: float(v.detach()) for k, v in losses.items()}
