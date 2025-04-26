#!/usr/bin/env bash
# vim: set fileformat=unix

#=================================================================
# Скрипт для запуска узла map_visualizer из пакета path_controller
#=================================================================

set -e

### 0) Вычисляем корень workspace (две папки выше от этого скрипта)
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &>/dev/null && pwd )"
# SCRIPT_DIR == .../ros2_ws/src/path_controller
WS_ROOT="$( realpath "${SCRIPT_DIR}/../.." )"
# WS_ROOT == .../ros2_ws

echo "[run] Workspace root: $WS_ROOT"

### 1) Определяем ROS2-дистрибутив (как у тебя было)
DISTROS=(humble iron galactic foxy eloquent dashing rolling)
if [ -n "$ROS_DISTRO" ]; then
  DISTRO="$ROS_DISTRO"
else
  DISTRO=""
  for d in "${DISTROS[@]}"; do
    if [ -f "/opt/ros/$d/setup.bash" ]; then
      DISTRO="$d"
      break
    fi
  done
fi

if [ -z "$DISTRO" ]; then
  echo "Ошибка: не удалось найти установленный ROS 2-дистрибутив в /opt/ros/." >&2
  echo "Проверьте, что хотя бы один из следующих установлен: ${DISTROS[*]}" >&2
  exit 1
fi

echo "[run] Используем ROS 2-дистрибутив: $DISTRO"

### 2) Source системного ROS2
source "/opt/ros/$DISTRO/setup.bash"

### 3) Source workspace (из корня workspace!)
if [ -f "$WS_ROOT/install/setup.bash" ]; then
  source "$WS_ROOT/install/setup.bash"
elif [ -f "$WS_ROOT/install/local_setup.bash" ]; then
  source "$WS_ROOT/install/local_setup.bash"
else
  echo "Ошибка: не найден $WS_ROOT/install/setup.bash или local_setup.bash. Сначала выполните 'colcon build' из корня workspace." >&2
  exit 1
fi

### 4) Переходим в workspace
cd "$WS_ROOT"

### 5) Запускаем визуализатор
echo "[run] Запускаю map_visualizer_app"
exec ros2 run path_controller map_visualizer_app "$@"
