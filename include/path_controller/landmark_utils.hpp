#pragma once
#include <string>
#include <vector>

// Структура для хранения информации об ориентире
struct Landmark {
  std::string id;
  double x, y;
  int height;
};

// Функция для чтения ориентиров из файла skyscrapers.csv
std::vector<Landmark> readLandmarksFromFile(const std::string &filepath);