"""Training dataset: geometric camera augmentation + CenterNet-style ground-aware targets."""
from __future__ import annotations

import json
import math
from pathlib import Path

import cv2
import numpy as np
import torch
from torch.utils.data import Dataset

from . import geometry as G
from .weather import WeatherAug

NUM_BINS = 12
STRIDE = 4
MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32) * 255
STD = np.array([0.229, 0.224, 0.225], dtype=np.float32) * 255
MEAN_DIMS = np.array([G.CLASS_MEAN_DIMS[c] for c in G.CLASSES], dtype=np.float32)


def rot_xyz(rx: float, ry: float, rz: float) -> np.ndarray:
    cx, sx, cy, sy, cz, sz = math.cos(rx), math.sin(rx), math.cos(ry), math.sin(ry), math.cos(rz), math.sin(rz)
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def gaussian_radius(h: float, w: float, min_overlap: float = 0.7) -> float:
    # CenterNet radius heuristic.
    a1, b1, c1 = 1, h + w, w * h * (1 - min_overlap) / (1 + min_overlap)
    r1 = (b1 + math.sqrt(b1 ** 2 - 4 * a1 * c1)) / 2
    a2, b2, c2 = 4, 2 * (h + w), (1 - min_overlap) * w * h
    r2 = (b2 + math.sqrt(b2 ** 2 - 4 * a2 * c2)) / 2
    a3, b3, c3 = 4 * min_overlap, -2 * min_overlap * (h + w), (min_overlap - 1) * w * h
    r3 = (b3 + math.sqrt(b3 ** 2 - 4 * a3 * c3)) / 2
    return min(r1, r2, r3)


def draw_gaussian(hm: np.ndarray, cx: int, cy: int, radius: int) -> None:
    d = 2 * radius + 1
    sigma = d / 6
    y, x = np.ogrid[-radius:radius + 1, -radius:radius + 1]
    g = np.exp(-(x * x + y * y) / (2 * sigma * sigma))
    H, W = hm.shape
    l, r = min(cx, radius), min(W - cx, radius + 1)
    t, b = min(cy, radius), min(H - cy, radius + 1)
    if l + r <= 0 or t + b <= 0:
        return
    region = hm[cy - t:cy + b, cx - l:cx + r]
    np.maximum(region, g[radius - t:radius + b, radius - l:radius + r], out=region)


class CameraAug:
    """Samples a virtual camera: new focal/principal point, small rotation, optional mirror.

    The image is warped with the induced homography and every 3D quantity is transformed with the
    same rotation, so labels stay exact. This teaches the network many mountings from few cameras.
    """

    def __init__(self, out_w: int, out_h: int, train: bool, max_rot_deg=(4.0, 6.0, 3.0),
                 focal_scale=(0.45, 1.0)):
        self.out_w, self.out_h, self.train = out_w, out_h, train
        self.max_rot = np.deg2rad(max_rot_deg)
        self.focal_scale = focal_scale

    def sample(self, K: np.ndarray, w: int, h: int, rng: np.random.Generator):
        fit = min(self.out_w / w, self.out_h / h)
        if not self.train:
            s = fit
            Kn = K.copy()
            Kn[:2] *= s
            pad_x = (self.out_w - w * s) / 2
            pad_y = (self.out_h - h * s) / 2
            Kn[0, 2] += pad_x
            Kn[1, 2] += pad_y
            return Kn, np.eye(3), False
        s = rng.uniform(*self.focal_scale)
        s = max(s, fit * 0.7)
        Kn = K.copy()
        Kn[:2] *= s
        # Random crop window: choose where the scaled principal point lands in the output.
        sw, sh = w * s, h * s
        ox = rng.uniform(min(0, self.out_w - sw), max(0, self.out_w - sw))
        oy = rng.uniform(min(0, self.out_h - sh), max(0, self.out_h - sh))
        Kn[0, 2] = K[0, 2] * s + ox
        Kn[1, 2] = K[1, 2] * s + oy
        Kn[1, 1] = Kn[0, 0] * (K[1, 1] / K[0, 0]) * rng.uniform(0.97, 1.03)
        dR = rot_xyz(*(rng.uniform(-1, 1, 3) * self.max_rot))
        flip = rng.random() < 0.5
        return Kn, dR, flip


class JP3DDataset(Dataset):
    def __init__(self, jsonl: str | Path, image_root: str | Path, out_w=1280, out_h=736, train=True,
                 weather=True, max_objs=256, seed=0, extra: list[tuple[Path, Path]] | None = None):
        self.samples = []
        for jl, root in [(Path(jsonl), Path(image_root))] + list(extra or []):
            if not Path(jl).exists():
                continue
            for l in Path(jl).read_text().splitlines():
                if l.strip():
                    s = json.loads(l)
                    s["_root"] = str(root)
                    self.samples.append(s)
        self.root = Path(image_root)
        self.out_w, self.out_h, self.train = out_w, out_h, train
        self.cam_aug = CameraAug(out_w, out_h, train)
        self.weather = WeatherAug() if (train and weather) else None
        self.max_objs = max_objs
        self.seed = seed
        self.cls_index = {c: i for i, c in enumerate(G.CLASSES)}

    def __len__(self):
        return len(self.samples)

    def load(self, idx: int, rng: np.random.Generator):
        s = self.samples[idx]
        img = cv2.imread(str(Path(s.get("_root", self.root)) / s["image"]), cv2.IMREAD_COLOR)
        K = np.array(s["K"], dtype=np.float64)
        plane = np.array(s["plane"], dtype=np.float64)
        Kn, dR, flip = self.cam_aug.sample(K, img.shape[1], img.shape[0], rng)
        # Homography from source pixels to output pixels for a pure camera rotation dR (X' = dR X).
        Hm = Kn @ dR @ np.linalg.inv(K)
        out = cv2.warpPerspective(img, Hm, (self.out_w, self.out_h), flags=cv2.INTER_LINEAR,
                                  borderMode=cv2.BORDER_CONSTANT, borderValue=(114, 114, 114))
        n = dR @ plane[:3]
        plane_n = np.concatenate([n, [plane[3]]])
        objs = []
        for o in s["objects"]:
            objs.append({
                "cls": o["cls"],
                "bottom": dR @ np.array(o["bottom"]),
                "dims": np.array(o["dims"]),
                "heading": dR @ np.array(o["heading"]),
            })
        if flip:
            out = out[:, ::-1].copy()
            M = np.diag([-1.0, 1.0, 1.0])
            Kn = Kn.copy()
            Kn[0, 2] = self.out_w - 1 - Kn[0, 2]
            plane_n = np.concatenate([M @ plane_n[:3], [plane_n[3]]])
            for o in objs:
                o["bottom"] = M @ o["bottom"]
                o["heading"] = M @ o["heading"]
        return out, Kn, plane_n, objs, s

    def __getitem__(self, idx: int):
        rng = np.random.default_rng((self.seed, idx, int(torch.randint(1 << 30, (1,)))) if self.train else (self.seed, idx))
        img, K, plane, objs, s = self.load(idx, rng)
        if self.weather is not None:
            img = self.weather(img, rng)
        Ho, Wo = self.out_h // STRIDE, self.out_w // STRIDE
        C = len(G.CLASSES)
        hm = np.zeros((C, Ho, Wo), np.float32)
        ind = np.zeros(self.max_objs, np.int64)
        mask = np.zeros(self.max_objs, np.float32)
        cls_t = np.zeros(self.max_objs, np.int64)
        off = np.zeros((self.max_objs, 2), np.float32)
        ltrb = np.zeros((self.max_objs, 4), np.float32)
        dz = np.zeros(self.max_objs, np.float32)
        dim = np.zeros((self.max_objs, 3), np.float32)
        rbin = np.zeros(self.max_objs, np.int64)
        rres = np.zeros(self.max_objs, np.float32)
        ignore = np.zeros((Ho, Wo), np.float32)
        n = plane[:3]
        k = 0
        for o in objs:
            b, dims, head = o["bottom"], o["dims"], o["heading"]
            if b[2] < 1.0:
                continue
            corners = G.box_corners(b[None], dims[None], head[None], n)[0]
            if (corners[:, 2] < 0.3).any():
                continue
            uv = G.project(K, corners)
            x1, y1 = uv.min(0)
            x2, y2 = uv.max(0)
            kp = G.project(K, b[None])[0]
            if not (0 <= kp[0] < self.out_w and 0 <= kp[1] < self.out_h):
                continue
            cx1, cy1 = max(x1, 0), max(y1, 0)
            cx2, cy2 = min(x2, self.out_w - 1), min(y2, self.out_h - 1)
            bw, bh = (cx2 - cx1) / STRIDE, (cy2 - cy1) / STRIDE
            kx, ky = kp / STRIDE
            ix, iy = int(kx), int(ky)
            if bw < 1.0 or bh < 1.0 or k >= self.max_objs:
                # Too small to learn: do not penalise detections there.
                ignore[max(iy - 1, 0):iy + 2, max(ix - 1, 0):ix + 2] = 1
                continue
            ci = self.cls_index[o["cls"]]
            r = max(1, int(gaussian_radius(math.ceil(bh), math.ceil(bw))))
            draw_gaussian(hm[ci], ix, iy, r)
            ray = G.pixel_rays(K, kp[None])
            alpha = float(G.heading_to_alpha(head[None], ray, n)[0])
            a = (alpha + math.pi) % (2 * math.pi)
            bin_w = 2 * math.pi / NUM_BINS
            bi = int(a // bin_w) % NUM_BINS
            ind[k] = iy * Wo + ix
            mask[k] = 1
            cls_t[k] = ci
            off[k] = (kx - ix, ky - iy)
            ltrb[k] = ((kp[0] - cx1) / STRIDE, (kp[1] - cy1) / STRIDE, (cx2 - kp[0]) / STRIDE, (cy2 - kp[1]) / STRIDE)
            dz[k] = float(G.plane_height(plane, b[None])[0])
            dim[k] = np.log(dims / MEAN_DIMS[ci])
            rbin[k] = bi
            rres[k] = (a - (bi + 0.5) * bin_w) / (bin_w / 2)
            k += 1
        x = (img[:, :, ::-1].astype(np.float32) - MEAN) / STD
        x = torch.from_numpy(x.transpose(2, 0, 1).copy())
        target = {
            "hm": torch.from_numpy(hm), "ind": torch.from_numpy(ind), "mask": torch.from_numpy(mask),
            "cls": torch.from_numpy(cls_t), "off": torch.from_numpy(off), "ltrb": torch.from_numpy(ltrb),
            "dz": torch.from_numpy(dz), "dim": torch.from_numpy(dim), "rbin": torch.from_numpy(rbin),
            "rres": torch.from_numpy(rres), "ignore": torch.from_numpy(ignore),
        }
        meta = {"K": torch.from_numpy(K.astype(np.float32)), "plane": torch.from_numpy(plane.astype(np.float32)),
                "idx": idx}
        return x, target, meta
