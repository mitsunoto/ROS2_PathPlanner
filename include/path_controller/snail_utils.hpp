#pragma once
#include <utility>

// Функция, возвращающая координаты (dx, dy) для номера ячейки от 1 до 25 по схеме "улитки" для сетки 5x5.
std::pair<double,double> snailOffset5(int n);

// Коэффициент преобразования: 1 крупная ячейка = 25 мелких (5x5)
constexpr double F = 25.0;