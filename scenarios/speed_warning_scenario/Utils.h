// Utils.h
#pragma once

#include <string>
#include <boost/units/systems/si.hpp>
#include <boost/units/io.hpp>
#include <fstream>
#include <iomanip>
#include <ctime>
#include <sstream>
#include <filesystem>
#include <sys/stat.h>
#include <unistd.h>

std::string getCurrentDateTime();
void createDirectory(const std::string& path);
void logToFile(const std::string& vehicleId, const std::string& message);