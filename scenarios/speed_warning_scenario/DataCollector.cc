
#include "DataCollector.h"
#include <iostream>
#include <iomanip>
#include <filesystem>


DataCollector::DataCollector(const std::string& dataDir) : mDataDir(dataDir)
{
    createDirectory(mDataDir);
}

DataCollector::~DataCollector()
{
    for (auto& file : mVehicleFiles) {
        file.second.close();
    }
    for (auto& file : mDenmFiles) {
        file.second.close();
    }
}

void DataCollector::initializeVehicleFile(const std::string& sumoId, const std::string& stationId)
{
    std::string filename = mDataDir + "/vehicle_data_" + sumoId +".csv";
    std::ofstream file(filename, std::ios::out | std::ios::trunc);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open vehicle data file: " + filename);
    }
    file << "Time,Speed,PosX,PosY,ReceivedDenm,Action" << std::endl;
    mVehicleFiles[sumoId] = std::move(file);

    initializeDenmFile(sumoId, stationId);
}

void DataCollector::initializeDenmFile(const std::string& sumoId, const std::string& stationId)
{
    std::string filename = mDataDir + "/denm_data_" + sumoId + "_" + stationId + ".csv";
    std::ofstream file(filename, std::ios::out | std::ios::trunc);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open DENM data file: " + filename);
    }
    file << "Time,SenderSumoID,SenderStationID,OffenderSumoID,OffenderStationID,DetectedSpeed,PosX,PosY,ViolationType" << std::endl;
    mDenmFiles[sumoId] = std::move(file);
}


void DataCollector::createDirectory(const std::string& path)
{
    // Créer le répertoire de données s'il n'existe pas
    int status = mkdir(path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    if (status != 0 && errno != EEXIST) {
        throw std::runtime_error("Failed to create directory: " + path + " (" + strerror(errno) + ")");
    }
}

void DataCollector::recordDenmData(const std::string& sumoId, double time, const std::string& senderSumoId, const std::string& senderStationId,
                                   const std::string& offenderSumoId, const std::string& offenderStationId,
                                   double detectedSpeed, double posX, double posY, const std::string& violationType)
{
    std::lock_guard<std::mutex> lock(mMutex);
    auto it = mDenmFiles.find(sumoId);
    if (it != mDenmFiles.end()) {
        it->second << std::fixed << std::setprecision(3)
                   << time << ","
                   << senderSumoId << ","
                   << senderStationId << ","
                   << offenderSumoId << ","
                   << offenderStationId << ","
                   << detectedSpeed << ","
                   << posX << ","
                   << posY << ","
                   << violationType << std::endl;
    }
}

void DataCollector::recordVehicleData(const std::string& sumoId, double time, double speed, double posX, double posY)
{
    std::lock_guard<std::mutex> lock(mMutex);
    auto it = mVehicleFiles.find(sumoId);
    if (it != mVehicleFiles.end()) {
        it->second << std::fixed << std::setprecision(3)
                   << time << ","
                   << speed << ","
                   << posX << ","
                   << posY << ","
                   << std::endl;
    }
}