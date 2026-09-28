#!/usr/bin/env bash

set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CFG_PATH="$PROJECT_ROOT/stm32h723.cfg"
ELF_NAME="Engineer_2026_H7_Template_new.elf"
BUILD_DIR="$PROJECT_ROOT/cmake-build-debug"

# 若未传参，先尝试编译再解析路径
ELF_PATH="${1:-}"
if [[ -n "$ELF_PATH" && ! -f "$ELF_PATH" ]]; then
  echo "指定路径下没有 ELF: $ELF_PATH" >&2
  exit 1
fi

if [[ -z "$ELF_PATH" ]]; then
  if [[ ! -f "$BUILD_DIR/CMakeCache.txt" ]]; then
    echo "未找到已配置的构建目录: $BUILD_DIR" >&2
    exit 1
  fi

  echo "正在编译: $BUILD_DIR"
  /usr/bin/cmake --build "$BUILD_DIR"
  ELF_PATH="$BUILD_DIR/$ELF_NAME"
  if [[ ! -f "$ELF_PATH" ]]; then
    echo "本次构建未生成 ELF: $ELF_PATH" >&2
    exit 1
  fi
fi

if [[ ! -f "$CFG_PATH" ]]; then
  echo "OpenOCD 配置文件不存在: $CFG_PATH" >&2
  exit 1
fi

echo "使用 ELF: $ELF_PATH"
echo "使用 OpenOCD 配置: $CFG_PATH"

/usr/bin/openocd \
  -s /usr/share/openocd/scripts \
  -f "$CFG_PATH" \
  -c "tcl_port disabled" \
  -c "gdb_port disabled" \
  -c "program {$ELF_PATH} verify reset exit"
