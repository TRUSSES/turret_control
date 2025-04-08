#include "config.h"
#include <iostream>

Config& Config::Instance() {
  static Config instance;
  return instance;
}

bool Config::Load(const std::string &filename) {
  try {
    config_ = YAML::LoadFile(filename);
    return true;
  } catch (const std::exception &ex) {
    std::cerr << "Failed to load config file: " << ex.what() << std::endl;
    return false;
  }
}

YAML::Node Config::GetConfig() const {
  return config_;
}
