#!/usr/bin/env bash
# Sync shared non-simulation packages, model assets, and frontend modules from dev -> main.
set -euo pipefail

DEV_REPO="${DEV_REPO:-/home/ly/robot2-dev}"
MAIN_REPO="${MAIN_REPO:-/home/ly/robot2-main}"
DRY_RUN=false
NO_BUILD=false

for arg in "$@"; do
  case "$arg" in
    --dry-run|-n) DRY_RUN=true ;;
    --no-build) NO_BUILD=true ;;
    --help|-h)
      echo "Usage: $0 [--dry-run|-n] [--no-build]"
      exit 0
      ;;
    *)
      echo "Unknown option: $arg" >&2
      exit 1
      ;;
  esac
done

RSYNC_OPTS=(-az --delete --itemize-changes --exclude='.git/' --exclude='__pycache__/' --exclude='.pytest_cache/' --exclude='*.pyc' --exclude='node_modules/')
if $DRY_RUN; then
  RSYNC_OPTS+=(-n)
fi

DEV_SRC="$DEV_REPO/ros2_ws/src"
MAIN_SRC="$MAIN_REPO/ros2_ws/src"

echo "=== Syncing shared packages: $DEV_SRC -> $MAIN_SRC ==="
rsync "${RSYNC_OPTS[@]}" "$DEV_SRC/can_bridge_msgs/" "$MAIN_SRC/can_bridge_msgs/"
rsync "${RSYNC_OPTS[@]}" "$DEV_SRC/robot_dynamics/" "$MAIN_SRC/robot_dynamics/"
rsync "${RSYNC_OPTS[@]}" \
  --exclude='robot_transmission/mujoco_positions.py' \
  --exclude='test/test_mujoco_joint_aliases.py' \
  "$DEV_SRC/robot_transmission/" "$MAIN_SRC/robot_transmission/"
rsync "${RSYNC_OPTS[@]}" "$DEV_SRC/bvh_broadcaster/" "$MAIN_SRC/bvh_broadcaster/"
rsync "${RSYNC_OPTS[@]}" "$DEV_SRC/robot_description/config/" "$MAIN_SRC/robot_description/config/"
rsync "${RSYNC_OPTS[@]}" \
  --exclude='mujoco/' \
  --exclude='cad/' \
  --exclude='*.hil.*' \
  --exclude='*.simulation.*' \
  --exclude='tools/build_weighed_mujoco.py' \
  --exclude='tools/build_hil_config.py' \
  "$DEV_SRC/robot_description/models/full_body_robot/" "$MAIN_SRC/robot_description/models/full_body_robot/"

echo "=== Syncing shared web_host backend & frontend modules ==="
rsync "${RSYNC_OPTS[@]}" "$DEV_SRC/web_host/web_host/" "$MAIN_SRC/web_host/web_host/"
for sub in api stores styles core; do
  rsync "${RSYNC_OPTS[@]}" "$DEV_SRC/web_host/frontend/src/$sub/" "$MAIN_SRC/web_host/frontend/src/$sub/"
done
rsync "${RSYNC_OPTS[@]}" "$DEV_SRC/web_host/frontend/src/style.css" "$MAIN_SRC/web_host/frontend/src/style.css"
rsync "${RSYNC_OPTS[@]}" \
  --exclude='arm.ts' \
  --exclude='motion_preview.ts' \
  "$DEV_SRC/web_host/frontend/src/panels/" "$MAIN_SRC/web_host/frontend/src/panels/"
rsync "${RSYNC_OPTS[@]}" \
  --exclude='SimWorkbench.vue' \
  --exclude='MotionWorkbench.vue' \
  "$DEV_SRC/web_host/frontend/src/components/" "$MAIN_SRC/web_host/frontend/src/components/"

echo "=== Checking intentionally board-divergent files ==="
DIVERGENT=(
  "web_host/frontend/src/App.vue"
  "web_host/frontend/src/panels/arm.ts"
  "web_host/frontend/src/panels/motion_preview.ts"
  "web_host/frontend/src/components/MotionWorkbench.vue"
  "robot_bringup/bench_manager/node.py"
  "robot_bringup/launch/bench.launch.py"
)
for rel in "${DIVERGENT[@]}"; do
  if [[ -f "$DEV_SRC/$rel" && -f "$MAIN_SRC/$rel" && "$DEV_SRC/$rel" -nt "$MAIN_SRC/$rel" ]]; then
    echo "  [NOTICE] dev is newer than main: $rel (review manually if non-sim changes were made)"
  fi
done

if ! $DRY_RUN && ! $NO_BUILD; then
  echo "=== Building main frontend ==="
  (cd "$MAIN_SRC/web_host/frontend" && npm run build)
fi
echo "=== main git status ==="
git -C "$MAIN_REPO" status --short | head -n 40
echo "Done. Next: review diff in $MAIN_REPO, then git commit && git push."
