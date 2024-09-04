// Utils.cc
#include "Utils.h"
#include <chrono>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <sys/stat.h>
#include <omnetpp.h>

std::string getCurrentDateTime() {
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%m_%d_%H_%M_%S");
    return ss.str();
}

void createDirectory(const std::string& path) {
    mkdir(path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
}

void logToFile(const std::string& vehicleId, const std::string& message)
{
    static int fileNum = 1;
    static std::string currentLogFile;
    static std::ofstream logFile;

    if (!logFile.is_open()) {
        std::string logDir = "/home/yelfatihi/artery/scenarios/highway-police/logs";
        createDirectory(logDir);

        std::string dateTime = getCurrentDateTime();
        currentLogFile = logDir + "/" + std::to_string(fileNum) + "_" + dateTime + ".log";
        logFile.open(currentLogFile, std::ios::app);

        if (logFile.is_open()) {
            fileNum++;
            // Ouvrir le fichier dans VSCode
            std::string command = "code " + currentLogFile;
            system(command.c_str());
        }
    }

    if (logFile.is_open()) {
        auto now = omnetpp::simTime();
        logFile << std::setw(10) << vehicleId << " " 
                << std::setw(10) << std::fixed << std::setprecision(2) << now.dbl() << " " 
                << message << std::endl;
    }
}
