#pragma once

#include <string>
#include <yaml-cpp/yaml.h>
#include <iostream>

namespace vk {

template<typename T>
T param(const YAML::Node& node, const std::string& name, const T& defaultValue, const bool silent = false)
{
  if (node[name]) {
    T v = node[name].as<T>();
    return v;
  }
  if (!silent)
    std::cerr << "Using default value for parameter \"" << name << "\": " << defaultValue << std::endl;
  return defaultValue;
}

template<typename T>
bool hasParam(const YAML::Node& node, const std::string& name)
{
  return static_cast<bool>(node[name]);
}

template<typename T>
bool getParam(const YAML::Node& node, const std::string& name, T& value)
{
  if (node[name]) {
    value = node[name].as<T>();
    return true;
  }
  return false;
}

} // namespace vk
