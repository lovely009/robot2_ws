#!/usr/bin/env bash
# Run on the board inside ~/robot2-main: git pull --ff-only, rebuild changed ROS packages, restart service.
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS_DIR="$REPO_DIR/ros2_ws"
MANUAL_PACKAGES=""
REBUILD_ALL=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --packages) MANUAL_PACKAGES="${2:-}"; shift 2 ;;
    --all) REBUILD_ALL=true; shift ;;
    --help|-h)
      echo "Usage: $0 [--all] [--packages \"pkg1 pkg2\"]"
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      exit 1
      ;;
  esac
done

cd "$REPO_DIR"
OLD_HEAD="$(git rev-parse HEAD)"
echo "=== Pulling latest main ($OLD_HEAD) ==="
git pull --ff-only
NEW_HEAD="$(git rev-parse HEAD)"

if [[ -n "$MANUAL_PACKAGES" ]]; then
  PKGS="$MANUAL_PACKAGES"
elif $REBUILD_ALL; then
  PKGS="can_bridge_msgs motor_hardware_interface robot_dynamics robot_transmission bvh_broadcaster robot_description robot_bringup web_host"
elif [[ "$OLD_HEAD" == "$NEW_HEAD" ]]; then
  echo "Already up to date ($NEW_HEAD)."
  PKGS=""
else
  PKGS="$(git diff --name-only "$OLD_HEAD" "$NEW_HEAD" -- ros2_ws/src/ | awk -F/ '{print $3}' | grep -E '^(can_bridge_msgs|motor_hardware_interface|robot_dynamics|robot_transmission|bvh_broadcaster|robot_description|robot_bringup|web_host)$' | sort -u | tr '\n' ' ' | xargs || true)"
fi

if [[ -n "$PKGS" ]]; then
  echo "=== Rebuilding packages: $PKGS ==="
  cd "$WS_DIR"
  for p in $PKGS; do
    rm -rf "build/$p" "install/$p"
  done
  set +u
  source /opt/ros/humble/setup.bash
  set -u
  colcon build --packages-select $PKGS
fi

echo "=== Restarting robot2-workbench.service ==="
systemctl --user restart robot2-workbench.service
sleep 2
systemctl --user is-active robot2-workbench.service

python3 - <<'PY'
import json, time, urllib.request
base = "http://127.0.0.1:8080"
for attempt in range(8):
    try:
        models = json.loads(urllib.request.urlopen(f"{base}/api/models", timeout=3).read())
        trans = json.loads(urllib.request.urlopen(f"{base}/api/model-transmissions?model=full_body_robot", timeout=3).read())
        assert "full_body_robot" in models.get("models", [])
        assert trans.get("hardware_ready", True) is True
        print(f"OK {base}: models={models['models']}, hardware_ready=True, groups={len(trans['groups'])}")
        break
    except Exception:
        if attempt == 7:
            raise
        time.sleep(1)
PY
