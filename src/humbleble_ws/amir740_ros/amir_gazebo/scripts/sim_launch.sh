#!/bin/bash
# Ignition Gazebo を NVIDIA GPU で起動するラッパー。
# Intel 統合 GPU (0x7d67) が Mesa に未登録のため EGL 初期化失敗が起きる問題を回避する。
# EGL ベンダーを NVIDIA に固定することで Mesa の Intel GPU 探索をスキップする。

export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/../../../../.." && pwd)"

source "${WS_ROOT}/install/setup.bash"

exec ros2 launch amir_gazebo gazebo_bringup.launch.py "$@"
