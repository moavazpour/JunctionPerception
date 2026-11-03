#!/usr/bin/env bash
# Runs the live pipeline on every TUMTraf Intersection R02 sequence (S1-S4) and both south
# cameras for each bias-correction variant, evaluates the world states against the labels and
# plots the results.
# Output: outputs/<run>/<variant>/<sequence>_<camera>/world.jsonl,
#         outputs/<run>/eval/<variant>/{metrics.json,report.md},
#         outputs/<run>/eval/{summary.md,figures/}.
# Usage: JP_DATA_ROOT=<dataset root> scripts/run_benchmark.sh <run> [variant ...]
#        (variants: box_fit ray_offset none; default all three). JP_PIPELINE_CONFIG selects the
#        pipeline config (default configs/pipeline.yaml), e.g. to compare detectors. JP_PYTHON
#        selects the Python interpreter for the figures (default: python3, needs matplotlib).
#        JP_EVAL_ARGS adds jp_eval options, e.g. "--labels merged --ignore-unlabelled".
set -euo pipefail

run="${1:?usage: scripts/run_benchmark.sh <run> [variant ...]}"
shift
variants=("$@")
if [ ${#variants[@]} -eq 0 ]; then
    variants=(box_fit ray_offset none)
fi
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_root}"
in_docker="${repo_root}/scripts/in_docker.sh"

for variant in "${variants[@]}"; do
    eval_runs=()
    for seq in s01 s02 s03 s04; do
        # S1-S3 were recorded by day, S4 at night in rain.
        condition=day
        if [ "${seq}" = s04 ]; then
            condition=night
        fi
        for camera in south1 south2; do
            out="outputs/${run}/${variant}/${seq}_${camera}"
            mkdir -p "${out}"
            echo "${variant}: ${seq} ${camera}"
            "${in_docker}" build/release/src/apps/jp_run \
                --source "/data/a9_dataset_r02_${seq}_s110_camera_basler_${camera}_8mm.mp4" \
                --scene "scenes/tumtraf_s110_${camera}" --bias-correction "${variant}" \
                --config "${JP_PIPELINE_CONFIG:-configs/pipeline.yaml}" \
                --world-jsonl "${out}/world.jsonl" >"${out}/log.txt" 2>&1
            eval_runs+=(--run "${seq}_${camera},${condition},${out}/world.jsonl,/data/a9_dataset_r02_${seq},scenes/tumtraf_s110_${camera}")
        done
    done
    # shellcheck disable=SC2086 # JP_EVAL_ARGS is a list of options
    "${in_docker}" build/release/src/apps/jp_eval "${eval_runs[@]}" ${JP_EVAL_ARGS:-} \
        --out "outputs/${run}/eval/${variant}" --title "${variant}"
done
"${JP_PYTHON:-python3}" tools/plot_evaluation.py "outputs/${run}/eval" --variants "${variants[@]}"
