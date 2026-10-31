"""Figures and a summary table from jp_eval results of several pipeline variants.

Reads <eval dir>/<variant>/metrics.json for every variant folder and writes
<eval dir>/figures/*.png and <eval dir>/summary.md.

Usage: python tools/plot_evaluation.py outputs/<run>/eval [--variants box_fit ray_offset none]
"""

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

# Categorical slots 1-3 (fixed order), ink and surface of the light theme.
SERIES = ["#2a78d6", "#eb6834", "#1baf7a"]
SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK_SECONDARY = "#52514e"
GRID = "#e4e3df"
LABELS = {"box_fit": "box fit", "ray_offset": "ray offset", "none": "no correction"}


def style(ax, title, ylabel):
    ax.set_facecolor(SURFACE)
    ax.set_title(title, loc="left", color=INK, fontsize=11)
    ax.set_ylabel(ylabel, color=INK_SECONDARY, fontsize=9)
    ax.tick_params(colors=INK_SECONDARY, labelsize=8, length=0)
    ax.grid(axis="y", color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)
    for side in ("top", "right", "left"):
        ax.spines[side].set_visible(False)
    ax.spines["bottom"].set_color(GRID)


def figure():
    fig, ax = plt.subplots(figsize=(6.4, 3.6), dpi=150)
    fig.patch.set_facecolor(SURFACE)
    return fig, ax


def range_labels(metrics):
    return [f"{r['min_m']:.0f}–{r['max_m']:.0f}" for r in metrics["overall"]["by_range"]]


def lines_by_range(results, key, title, ylabel, path):
    fig, ax = figure()
    labels = range_labels(next(iter(results.values())))
    for color, (variant, metrics) in zip(SERIES, results.items()):
        ranges = metrics["overall"]["by_range"]
        values = [key(r) for r in ranges]
        points = [(i, v) for i, (r, v) in enumerate(zip(ranges, values)) if r["truth"] > 0]
        xs, ys = zip(*points)
        ax.plot(xs, ys, color=color, linewidth=2, marker="o", markersize=5, label=LABELS.get(variant, variant))
    ax.set_xticks(range(len(labels)), labels)
    ax.set_xlabel("distance from the camera (m)", color=INK_SECONDARY, fontsize=9)
    style(ax, title, ylabel)
    ax.set_ylim(bottom=0)
    ax.legend(frameon=False, fontsize=8, labelcolor=INK_SECONDARY, loc="best")
    fig.tight_layout()
    fig.savefig(path, facecolor=SURFACE)
    plt.close(fig)


def bars_by_condition(results, path):
    fig, ax = figure()
    conditions = sorted(next(iter(results.values()))["by_condition"])
    width = 0.8 / len(results)
    for i, (color, (variant, metrics)) in enumerate(zip(SERIES, results.items())):
        values = [metrics["by_condition"][c]["position_m"]["median"] for c in conditions]
        xs = [c + (i - (len(results) - 1) / 2) * width for c in range(len(conditions))]
        bars = ax.bar(xs, values, width=width - 0.03, color=color, label=LABELS.get(variant, variant))
        ax.bar_label(bars, fmt="%.2f", fontsize=7, color=INK_SECONDARY, padding=2)
    ax.set_xticks(range(len(conditions)), conditions)
    style(ax, "Median position error by condition", "m")
    ax.legend(frameon=False, fontsize=8, labelcolor=INK_SECONDARY)
    fig.tight_layout()
    fig.savefig(path, facecolor=SURFACE)
    plt.close(fig)


def class_recall(metrics, variant, path):
    classes = {k: v for k, v in metrics["overall"]["by_class"].items() if v["truth"] > 0}
    names = sorted(classes, key=lambda k: -classes[k]["truth"])
    fig, ax = figure()
    bars = ax.bar(names, [classes[n]["recall"] for n in names], width=0.6, color=SERIES[0])
    ax.bar_label(bars, labels=[f"{classes[n]['recall']:.2f}\n({classes[n]['truth']})" for n in names],
                 fontsize=7, color=INK_SECONDARY, padding=2)
    ax.tick_params(axis="x", labelrotation=25)
    for label in ax.get_xticklabels():
        label.set_horizontalalignment("right")
    style(ax, f"Recall by labelled class ({LABELS.get(variant, variant)}; labels in brackets)", "recall")
    ax.set_ylim(0, 1.15)
    fig.tight_layout()
    fig.savefig(path, facecolor=SURFACE)
    plt.close(fig)


def row(name, m):
    t = m["tracking"]
    return (f"| {name} | {m['precision']:.3f} | {m['recall']:.3f} | "
            f"{m['position_m']['mean']:.2f} / {m['position_m']['median']:.2f} / {m['position_m']['p90']:.2f} | "
            f"{m['heading_deg']['median']:.1f} / {m['heading_deg']['p90']:.1f} | "
            f"{m['speed_mps']['median']:.2f} / {m['speed_mps']['p90']:.2f} | {t['mota']:.3f} | {t['idf1']:.3f} | "
            f"{m['displaced']} ({m['displaced_m']['median']:.2f}) | {m['recall_within_displaced_gate']:.3f} |")


def summary(results):
    head = ("| {} | Precision | Recall | Position error (m) | Heading error (deg) | Speed error (m/s) "
            "| MOTA | IDF1 | Displaced (median m) | Recall within 5 m |\n"
            "|---|---|---|---|---|---|---|---|---|---|")
    lines = ["# Benchmark summary", "", "Errors: mean / median / P90 (heading, speed: median / P90).", "",
             "## Bias correction", "", head.format("Variant")]
    for variant, m in results.items():
        lines.append(row(LABELS.get(variant, variant), m["overall"]))
    for variant, m in results.items():
        lines += ["", f"## Conditions ({LABELS.get(variant, variant)})", "", head.format("Condition")]
        for condition, cm in sorted(m["by_condition"].items()):
            lines.append(row(condition, cm))
    return "\n".join(lines) + "\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("eval_dir", type=Path)
    parser.add_argument("--variants", nargs="+", default=["box_fit", "ray_offset", "none"])
    args = parser.parse_args()
    results = {}
    for variant in args.variants:
        path = args.eval_dir / variant / "metrics.json"
        if path.exists():
            results[variant] = json.loads(path.read_text())
    if not results:
        raise SystemExit(f"no metrics.json under {args.eval_dir}")
    figures = args.eval_dir / "figures"
    figures.mkdir(parents=True, exist_ok=True)
    lines_by_range(results, lambda r: r["position_m"]["median"], "Median position error by distance",
                   "m", figures / "position_error_by_distance.png")
    lines_by_range(results, lambda r: r["recall"], "Recall by distance", "recall",
                   figures / "recall_by_distance.png")
    bars_by_condition(results, figures / "position_error_by_condition.png")
    first = next(iter(results))
    class_recall(results[first], first, figures / "recall_by_class.png")
    (args.eval_dir / "summary.md").write_text(summary(results))
    print(f"wrote {figures} and {args.eval_dir / 'summary.md'}")


if __name__ == "__main__":
    main()
