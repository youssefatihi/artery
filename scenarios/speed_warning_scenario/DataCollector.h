#pragma once

#include <string>
#include <fstream>
#include <mutex>
#include <map>
#include <sys/stat.h>
#include <errno.h>
#include <string.h>

class DataCollector
{
public:
    DataCollector(const std::string& dataDir);
    ~DataCollector();

    void initializeVehicleFile(const std::string& sumoId, const std::string& stationId);
    void recordVehicleData(const std::string& sumoId, double time, double speed, double posX, double posY);

    void recordDenmData(double time, const std::string& senderSumoId, const std::string& senderStationId,
                        const std::string& offenderSumoId, const std::string& offenderStationId,
                        double detectedSpeed, double posX, double posY, const std::string& violationType);

private:
    std::string mDataDir = "data";
    std::map<std::string, std::ofstream> mVehicleFiles ;
    std::ofstream mDenmFile;
    std::mutex mMutex;

    void createDirectory(const std::string& path);
};
