"""Train the ground-aware mono-3D detector.

    python train.py --data ~/data/junction_perception/jp3d --images ~/data/junction_perception/datasets/TUMTraf \
                    --out runs/gc3d_dla34 --epochs 60
"""
from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import numpy as np
import torch
from torch.utils.data import DataLoader

from jp3d.dataset import JP3DDataset
from jp3d.evaluate import evaluate, format_results
from jp3d.lift import lift
from jp3d.loss import JP3DLoss
from jp3d.model import DeployWrapper, GroundCenter3D


def collate(batch):
    xs, ts, ms = zip(*batch)
    x = torch.stack(xs)
    t = {k: torch.stack([b[k] for b in ts]) for k in ts[0]}
    return x, t, list(ms)


@torch.no_grad()
def run_eval(model, ds, device, max_frames=None, score_thr=0.1, bs=8):
    model.eval()
    wrapper = DeployWrapper(model).to(device).eval()
    frames = []
    idxs = list(range(len(ds)))[:max_frames] if max_frames else list(range(len(ds)))
    for i in range(0, len(idxs), bs):
        items = [ds[j] for j in idxs[i:i + bs]]
        x = torch.stack([it[0] for it in items]).to(device)
        with torch.autocast("cuda", dtype=torch.float16):
            dets = wrapper(x).float().cpu().numpy()
        for (xx, tt, meta), d in zip(items, dets):
            K, plane = meta["K"].numpy().astype(float), meta["plane"].numpy().astype(float)
            s = ds.samples[meta["idx"]]
            # GT in the evaluation camera: eval mode only rescales/pads, so 3D labels are unchanged.
            scale = K[0, 0] / s["K"][0][0]
            gts = [dict(o, box2d=[v * scale for v in o["box2d"]]) for o in s["objects"]]
            frames.append((gts, lift(d, K, plane, score_thr), plane))
    model.train()
    return evaluate(frames, min_box_h=8.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", type=Path, required=True)
    ap.add_argument("--images", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--backbone", default="dla34")
    ap.add_argument("--epochs", type=int, default=60)
    ap.add_argument("--bs", type=int, default=8)
    ap.add_argument("--lr", type=float, default=3e-4)
    ap.add_argument("--workers", type=int, default=10)
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=736)
    ap.add_argument("--eval_every", type=int, default=5)
    ap.add_argument("--eval_frames", type=int, default=600)
    ap.add_argument("--resume", type=Path, default=None)
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / "args.json").write_text(json.dumps({k: str(v) for k, v in vars(args).items()}, indent=1))

    device = torch.device("cuda")
    train_ds = JP3DDataset(args.data.expanduser() / "train.jsonl", args.images.expanduser(), args.width, args.height, train=True)
    val_ds = JP3DDataset(args.data.expanduser() / "val.jsonl", args.images.expanduser(), args.width, args.height, train=False)
    dl = DataLoader(train_ds, batch_size=args.bs, shuffle=True, num_workers=args.workers, collate_fn=collate,
                    drop_last=True, pin_memory=True, persistent_workers=True)
    model = GroundCenter3D(args.backbone).to(device)
    bb = [p for n, p in model.named_parameters() if n.startswith("backbone.")]
    rest = [p for n, p in model.named_parameters() if not n.startswith("backbone.")]
    opt = torch.optim.AdamW([{"params": bb, "lr": args.lr * 0.5}, {"params": rest, "lr": args.lr}], weight_decay=1e-4)
    total = args.epochs * len(dl)
    warm = min(1000, total // 20)
    sched = torch.optim.lr_scheduler.LambdaLR(
        opt, lambda s: (s + 1) / warm if s < warm else 0.5 * (1 + math.cos(math.pi * (s - warm) / max(total - warm, 1))) * 0.99 + 0.01)
    scaler = torch.amp.GradScaler()
    crit = JP3DLoss()
    start_ep, best = 0, -1.0
    if args.resume:
        ck = torch.load(args.resume, map_location="cpu")
        model.load_state_dict(ck["model"])
        opt.load_state_dict(ck["opt"])
        sched.load_state_dict(ck["sched"])
        start_ep, best = ck["epoch"] + 1, ck.get("best", -1)
    log = open(args.out / "log.jsonl", "a")
    step = start_ep * len(dl)
    for ep in range(start_ep, args.epochs):
        model.train()
        t0 = time.time()
        agg = {}
        for x, t, _ in dl:
            x = x.to(device, non_blocking=True)
            t = {k: v.to(device, non_blocking=True) for k, v in t.items()}
            with torch.autocast("cuda", dtype=torch.float16):
                out = model(x)
            loss, parts = crit(out, t)
            opt.zero_grad(set_to_none=True)
            scaler.scale(loss).backward()
            scaler.unscale_(opt)
            torch.nn.utils.clip_grad_norm_(model.parameters(), 10.0)
            scaler.step(opt)
            scaler.update()
            sched.step()
            step += 1
            for k, v in parts.items():
                agg[k] = agg.get(k, 0) + v
            if step % 50 == 0:
                print(f"ep {ep} step {step}/{total} loss {float(loss):.3f} " +
                      " ".join(f"{k}={v:.3f}" for k, v in parts.items()), flush=True)
        n = len(dl)
        rec = {"epoch": ep, "time": time.time() - t0, **{k: v / n for k, v in agg.items()}}
        ck = {"model": model.state_dict(), "opt": opt.state_dict(), "sched": sched.state_dict(), "epoch": ep,
              "best": best, "backbone": args.backbone}
        if (ep + 1) % args.eval_every == 0 or ep == args.epochs - 1:
            res = run_eval(model, val_ds, device, args.eval_frames)
            print(format_results(res), flush=True)
            rec.update({"mAP_dist": res["mAP_dist"], "mAP_iou0.1": res["mAP_iou0.1"], "mATE": res["mATE"]})
            if res["mAP_dist"] > best:
                best = res["mAP_dist"]
                ck["best"] = best
                torch.save(ck, args.out / "best.pt")
        torch.save(ck, args.out / "last.pt")
        log.write(json.dumps(rec) + "\n")
        log.flush()
        print(json.dumps(rec), flush=True)


if __name__ == "__main__":
    main()
