#pragma once

#include <string>
#include <vector>
#include <map>
#include "object.hpp"

void loadModel(std::map<std::string, Object> &objects, std::string fileLocation);
std::map<std::string, Object> loadModels(std::vector<std::string> files);
void getFiles(std::vector<std::string> &files, std::string folder);
std::map<std::string, Object> loadModels(std::string folder);
