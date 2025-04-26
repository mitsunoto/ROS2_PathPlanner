#include "path_controller/snail_utils.hpp"
#include <vector>
#include <utility>

// Функция, возвращающая смещение (dx, dy) для номера ячейки 1…25 по «улитке» в сетке 5×5
std::pair<double, double> snailOffset5(int n)
{
  // статический массив оффсетов, доступный только внутри этой функции
  static const std::vector<std::pair<int,int>> offsets = {
    {0,0}, {1,0}, {2,0}, {3,0}, {4,0},
    {4,1}, {4,2}, {4,3}, {4,4},
    {3,4}, {2,4}, {1,4}, {0,4},
    {0,3}, {0,2}, {0,1},
    {1,1}, {2,1}, {3,1},
    {3,2}, {3,3},
    {2,3}, {1,3},
    {1,2},
    {2,2}
  };

  // Если номер вне диапазона — возвращаем центр
  if (n < 1 || n > 25) {
    return {2,2};
  }

  return offsets[n-1];
}