#!/usr/bin/env bash
# Fetch MuJoCo Menagerie UR5e (official visual + dynamics model).
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEST="${REPO_ROOT}/third_party/mujoco_menagerie"
URL="https://github.com/google-deepmind/mujoco_menagerie.git"

if [ -d "${DEST}/universal_robots_ur5e/ur5e.xml" ]; then
  echo "MuJoCo Menagerie UR5e already present at ${DEST}"
  exit 0
fi

mkdir -p "${REPO_ROOT}/third_party"
if [ ! -d "${DEST}/.git" ]; then
  git clone --filter=blob:none --sparse --depth 1 "${URL}" "${DEST}"
fi
git -C "${DEST}" sparse-checkout set universal_robots_ur5e
git -C "${DEST}" checkout

echo "Menagerie UR5e ready: ${DEST}/universal_robots_ur5e/ur5e.xml"
