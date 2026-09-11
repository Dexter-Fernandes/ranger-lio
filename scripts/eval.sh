#!/usr/bin/env bash
# Frozen evo invocation used for every sequence and every ablation.
# Usage: scripts/eval.sh <results dir> <estimate.tum> [tag]
#   e.g. scripts/eval.sh results/exp14 results/exp14/identity.tum identity
set -euo pipefail
export MPLBACKEND=Agg

dir="${1:?results dir}"; est="${2:?estimate tum}"; tag="${3:-est}"
gt="$dir/gt.tum"
[ -f "$gt" ] || { echo "missing $gt (run scripts/gt_to_tum.py)" >&2; exit 1; }

# ALIGN=0 disables the SE(3) Umeyama alignment. Needed only for the week-1 identity check:
# a trajectory with every pose at the origin has rank-0 covariance and cannot be aligned.
# Every real estimate is evaluated with alignment on.
align=(-a); [ "${ALIGN:-1}" = "0" ] && align=()

# ATE: SE(3) Umeyama alignment, translation part, plot saved to PNG.
evo_ape tum "$gt" "$est" "${align[@]}" --plot_mode xyz \
  --save_plot "$dir/ape_${tag}.png" --save_results "$dir/ape_${tag}.zip" --no_warnings
# RPE over 1 m segments, same alignment. Skipped for the unaligned identity check, whose
# path length is zero and yields no 1 m pairs.
if [ "${ALIGN:-1}" != "0" ]; then
  evo_rpe tum "$gt" "$est" "${align[@]}" --delta 1 --delta_unit m \
    --save_results "$dir/rpe_${tag}.zip" --no_warnings
else
  echo "RPE skipped (ALIGN=0: zero path length)"
fi
evo_traj tum "$est" --ref "$gt" "${align[@]}" --plot_mode xyz \
  --save_plot "$dir/traj_${tag}.png" --no_warnings
echo "wrote $dir/{ape,rpe,traj}_${tag}.*"
