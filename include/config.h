#pragma once
#include <string>
#include <yaml-cpp/yaml.h>

class Config {
 public:
  // Returns the singleton instance.
  static Config& Instance();

  // Loads the configuration from a YAML file.
  bool Load(const std::string &filename);

  // Returns the loaded YAML node.
  YAML::Node GetConfig() const;

 private:
  Config() = default;
  YAML::Node config_;
};
