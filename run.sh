#!/usr/bin/env bash
# vim: set fileformat=unix

#=================================================================
# Скрипт для запуска узла map_visualizer из пакета path_controller
#=================================================================

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

# 1. Источник среды ROS 2
source "/opt/ros/$DISTRO/setup.bash"

# 2. Источник локального workspace
if [ -f "install/setup.bash" ]; then
  source "install/setup.bash"
elif [ -f "install/local_setup.bash" ]; then
  source "install/local_setup.bash"
else
  echo "Ошибка: не найден install/setup.bash или install/local_setup.bash. Сначала выполните 'colcon build'." >&2
  exit 1
fi

# 3. Запуск узла map_visualizer
exec ros2 run path_controller map_visualizer_app "$@"
