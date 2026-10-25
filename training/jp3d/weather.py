"""Photometric and adverse-weather augmentation (fog, rain, low light, glare, lens droplets, blur, JPEG)."""
from __future__ import annotations

import cv2
import numpy as np


def _fog(img, rng):
    h, w = img.shape[:2]
    strength = rng.uniform(0.25, 0.7)
    # Haze grows towards the top of the image, where the scene is farther away for a roadside camera.
    ramp = np.linspace(1.0, 0.25, h, dtype=np.float32)[:, None, None] ** rng.uniform(0.5, 2.0)
    t = 1.0 - strength * ramp
    air = rng.uniform(170, 235)
    return img * t + air * (1 - t)


def _rain(img, rng):
    h, w = img.shape[:2]
    noise = (rng.random((h, w)) < rng.uniform(0.001, 0.004)).astype(np.float32)
    length = int(rng.integers(10, 30))
    angle = rng.uniform(-20, 20)
    k = np.zeros((length, length), np.float32)
    k[:, length // 2] = 1
    M = cv2.getRotationMatrix2D((length / 2, length / 2), angle, 1)
    k = cv2.warpAffine(k, M, (length, length))
    k /= max(k.sum(), 1e-6)
    streaks = cv2.filter2D(noise, -1, k) * rng.uniform(80, 200)
    out = img * rng.uniform(0.75, 0.95) + streaks[:, :, None]
    return cv2.GaussianBlur(out, (3, 3), 0)


def _night(img, rng):
    gamma = rng.uniform(1.8, 3.0)
    out = 255.0 * (np.clip(img, 0, 255) / 255.0) ** gamma * rng.uniform(0.5, 0.9)
    out += rng.normal(0, rng.uniform(3, 10), img.shape)
    return out


def _glare(img, rng):
    h, w = img.shape[:2]
    cx, cy = rng.uniform(0, w), rng.uniform(0, h * 0.6)
    r = rng.uniform(0.1, 0.4) * w
    yy, xx = np.mgrid[0:h, 0:w].astype(np.float32)
    g = np.exp(-((xx - cx) ** 2 + (yy - cy) ** 2) / (2 * r * r))
    return img + g[:, :, None] * rng.uniform(80, 200)


def _droplets(img, rng):
    out = img.copy()
    h, w = img.shape[:2]
    blurred = cv2.GaussianBlur(img, (0, 0), rng.uniform(4, 9))
    mask = np.zeros((h, w), np.float32)
    for _ in range(int(rng.integers(3, 15))):
        c = (int(rng.uniform(0, w)), int(rng.uniform(0, h)))
        ax = (int(rng.uniform(10, 60)), int(rng.uniform(10, 60)))
        cv2.ellipse(mask, c, ax, rng.uniform(0, 180), 0, 360, 1.0, -1)
    mask = cv2.GaussianBlur(mask, (0, 0), 5)[:, :, None]
    return out * (1 - mask) + blurred * mask


def _color(img, rng):
    out = img * rng.uniform(0.7, 1.3) + rng.uniform(-25, 25)
    mean = out.mean(axis=(0, 1), keepdims=True)
    out = (out - mean) * rng.uniform(0.7, 1.3) + mean
    gray = out.mean(axis=2, keepdims=True)
    out = (out - gray) * rng.uniform(0.6, 1.4) + gray
    return out


def _blur(img, rng):
    if rng.random() < 0.5:
        return cv2.GaussianBlur(img, (0, 0), rng.uniform(0.5, 1.8))
    length = int(rng.integers(3, 9))
    k = np.zeros((length, length), np.float32)
    k[length // 2, :] = 1.0 / length
    M = cv2.getRotationMatrix2D((length / 2, length / 2), rng.uniform(0, 180), 1)
    return cv2.filter2D(img, -1, cv2.warpAffine(k, M, (length, length)))


class WeatherAug:
    def __init__(self, p_color=0.8, p_weather=0.35, p_blur=0.2, p_jpeg=0.3):
        self.p_color, self.p_weather, self.p_blur, self.p_jpeg = p_color, p_weather, p_blur, p_jpeg
        self.effects = [(_fog, 1.0), (_rain, 1.0), (_night, 1.0), (_glare, 0.5), (_droplets, 0.5)]

    def __call__(self, img: np.ndarray, rng: np.random.Generator) -> np.ndarray:
        out = img.astype(np.float32)
        if rng.random() < self.p_color:
            out = _color(out, rng)
        if rng.random() < self.p_weather:
            fns, ws = zip(*self.effects)
            ws = np.array(ws) / sum(ws)
            out = fns[rng.choice(len(fns), p=ws)](out, rng)
        if rng.random() < self.p_blur:
            out = _blur(out, rng)
        out = np.clip(out, 0, 255).astype(np.uint8)
        if rng.random() < self.p_jpeg:
            ok, buf = cv2.imencode(".jpg", out, [cv2.IMWRITE_JPEG_QUALITY, int(rng.integers(25, 80))])
            out = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        return out
