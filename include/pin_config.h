#pragma once
#include <string>
#include <yaml-cpp/yaml.h>

class Config {
 public:
  // Returns a reference to the singleton instance.
  static Config& Instance();

  // Loads the configuration from the specified YAML file.
  bool Load(const std::string &filename);

  // Returns the root YAML::Node.
  YAML::Node GetConfig() const;

 private:
  // Private constructor (singleton pattern).
  Config() = default;
  YAML::Node config_;
};
