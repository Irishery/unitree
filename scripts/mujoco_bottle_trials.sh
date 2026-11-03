#!/usr/bin/env bash
# Run stationary bottle-pick trials sequentially in the MuJoCo container.
# Each trial gets its own folder with summary.json, trajectory.csv and frames/.
set -euo pipefail

workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
image="${MUJOCO_IMAGE:-unitree-g1-mujoco:jazzy}"
root="${BOTTLE_TRIAL_ROOT:-${workspace_dir}/src/g1_mujoco/artifacts/bottle_trials}"
stamp="$(date +%Y%m%d_%H%M%S)"
out="${root}/${stamp}"
log="${out}/runner.log"
mkdir -p "${out}"

if [[ "${1:-}" == "--detach" ]]; then
  nohup "$0" "${@:2}" >"${log}" 2>&1 &
  echo "bottle trials running in background: ${log}"
  exit 0
fi

trials=(
  "center 0.420 0.000"
  "left 0.420 0.030"
  "right 0.420 -0.030"
  "near 0.400 0.000"
  "far 0.440 0.000"
)

cleanup() {
  if [[ -n "${container:-}" ]]; then
    docker rm -f "${container}" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT

index=0
for trial in "${trials[@]}"; do
  read -r name bottle_x bottle_y <<<"${trial}"
  index=$((index + 1))
  trial_name="$(printf 'trial_%02d_%s' "${index}" "${name}")"
  trial_dir="${out}/${trial_name}"
  mkdir -p "${trial_dir}"
  container="g1-bottle-trial-$$-${index}" 

  echo "=== ${trial_name}: bottle_x=${bottle_x} bottle_y=${bottle_y} ===" | tee -a "${log}"
  docker run -d --rm --name "${container}" --network host \
    -e ROS_DOMAIN_ID=42 -e ROS_LOCALHOST_ONLY=1 -e MUJOCO_GL=egl \
    -v "${root}:/ws/trials" \
    "${image}" \
    ros2 launch g1_mujoco pick.launch.py \
      viewer:=false bottle_x:="${bottle_x}" bottle_y:="${bottle_y}" \
    >>"${trial_dir}/launch.log" 2>&1

  # Wait until the detector graph is visible, then let the RGB-D frame settle.
  for _ in $(seq 1 30); do
    if docker exec "${container}" bash -lc \
      'source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && ros2 topic list' \
      2>/dev/null | grep -q '/g1/perception/bottle_detected'; then
      break
    fi
    sleep 1
  done

  docker exec "${container}" bash -lc \
    "source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && \
     python3 /ws/src/g1_mujoco/scripts/bottle_trial.py \
       --output /ws/trials/${stamp}/${trial_name}" \
    >"${trial_dir}/recorder.log" 2>&1 || true

  docker logs "${container}" >"${trial_dir}/launch.log" 2>&1 || true
  docker rm -f "${container}" >/dev/null 2>&1 || true
  container=""
done

python3 - "${out}" <<'PY'
import json
import pathlib
import sys

from PIL import Image, ImageDraw

out = pathlib.Path(sys.argv[1])
trials = []
for path in sorted(out.glob("trial_*/summary.json")):
    data = json.loads(path.read_text(encoding="utf-8"))
    data["trial"] = path.parent.name
    trials.append(data)
summary = {
    "trials": len(trials),
    "successes": sum(1 for trial in trials if trial.get("outcome") == "success"),
    "results": trials,
}
(out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
frames = []
for trial in trials:
    image = out / trial["trial"] / "frames" / "07_hold.png"
    if not image.is_file():
        continue
    thumb = Image.open(image).convert("RGB").resize((320, 240))
    panel = Image.new("RGB", (320, 270), "white")
    panel.paste(thumb, (0, 30))
    ImageDraw.Draw(panel).text((8, 8), trial["trial"], fill="black")
    frames.append(panel)
if frames:
    sheet = Image.new("RGB", (320 * len(frames), 270), "white")
    for index, panel in enumerate(frames):
        sheet.paste(panel, (index * 320, 0))
    sheet.save(out / "contact_sheet_hold.png")
print(f"SUMMARY {summary['successes']}/{summary['trials']} -> {out / 'summary.json'}")
PY

echo "Trials finished: ${out}" | tee -a "${log}"
