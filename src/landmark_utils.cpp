#include "path_controller/landmark_utils.hpp"
#include <fstream>
#include <sstream>
#include <iostream>

// Функция для чтения ориентиров из файла skyscrapers.csv
std::vector<Landmark> readLandmarksFromFile(const std::string &filepath) {
  std::vector<Landmark> landmarks;
  std::ifstream file(filepath);
  if (!file.is_open()) {
    std::cerr << "Failed to open file: " << filepath << std::endl;
    return landmarks;
  }
  
  std::string line;
  // Пропускаем заголовок
  std::getline(file, line);
  while (std::getline(file, line)) {
    std::istringstream ss(line);
    std::string token;
    Landmark lm;
    if (!std::getline(ss, token, ';')) continue;
    lm.id = token;
    if (!std::getline(ss, token, ';')) continue;
    lm.x = std::stod(token);
    if (!std::getline(ss, token, ';')) continue;
    lm.y = std::stod(token);
    if (!std::getline(ss, token, ';')) continue;
    lm.height = std::stoi(token);
    landmarks.push_back(lm);
  }
  file.close();
  return landmarks;
}