#!/usr/bin/env bash

set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CFG_PATH="$PROJECT_ROOT/stm32h723.cfg"
ELF_NAME="Engineer_2026_H7_Template_new.elf"

# 在常见构建目录中查找 ELF（Ninja 常在 build/ 根目录，多配置时可能在 Debug/Release/）
find_built_elf() {
  local d
  for d in \
    "$PROJECT_ROOT/build" \
    "$PROJECT_ROOT/build/Debug" \
    "$PROJECT_ROOT/build/Release" \
    "$PROJECT_ROOT/cmake-build-debug" \
    "$PROJECT_ROOT/cmake-build-release"
  do
    if [[ -f "$d/$ELF_NAME" ]]; then
      echo "$d/$ELF_NAME"
      return 0
    fi
  done
  return 1
}

# 若未传参，先尝试编译再解析路径
ELF_PATH="${1:-}"
if [[ -n "$ELF_PATH" && ! -f "$ELF_PATH" ]]; then
  echo "指定路径下没有 ELF: $ELF_PATH" >&2
  exit 1
fi

if [[ -z "$ELF_PATH" ]]; then
  if [[ -d "$PROJECT_ROOT/build" ]]; then
    echo "正在编译: $PROJECT_ROOT/build"
    /usr/bin/cmake --build "$PROJECT_ROOT/build"
  fi
  if ELF_PATH=$(find_built_elf); then
    :
  else
    echo "未找到 $ELF_NAME。请先在本机执行一次完整配置与编译，例如:" >&2
    echo "  cd $PROJECT_ROOT && cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug && cmake --build build" >&2
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
  -c "program \"$ELF_PATH\"" \
  -c reset \
  -c shutdown
