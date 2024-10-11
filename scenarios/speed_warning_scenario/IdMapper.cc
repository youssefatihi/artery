
#include "IdMapper.h"
#include <fstream>
#include <iostream>

IdMapper& IdMapper::getInstance()
{
    static IdMapper instance;
    return instance;
}

IdMapper::IdMapper() : mMappingFilePath("id_mappings.txt")
{
    loadMappings();
}

void IdMapper::addMapping(const std::string& sumoId, const std::string& stationId)
{
    std::lock_guard<std::mutex> lock(mMutex);
    mSumoToStation[sumoId] = stationId;
    mStationToSumo[stationId] = sumoId;
    saveMappings();
}

std::string IdMapper::getStationId(const std::string& sumoId)
{
    std::lock_guard<std::mutex> lock(mMutex);
    auto it = mSumoToStation.find(sumoId);
    return (it != mSumoToStation.end()) ? it->second : "";
}

std::string IdMapper::getSumoId(const std::string& stationId)
{
    std::lock_guard<std::mutex> lock(mMutex);
    auto it = mStationToSumo.find(stationId);
    return (it != mStationToSumo.end()) ? it->second : "";
}

void IdMapper::loadMappings()
{
    std::ifstream file(mMappingFilePath);
    if (file.is_open()) {
        std::string sumoId, stationId;
        while (file >> sumoId >> stationId) {
            mSumoToStation[sumoId] = stationId;
            mStationToSumo[stationId] = sumoId;
        }
        file.close();
    }
}

void IdMapper::saveMappings()
{
    std::ofstream file(mMappingFilePath, std::ios::app);
    if (file.is_open()) {
        for (const auto& pair : mSumoToStation) {
            file << pair.first << " " << pair.second << std::endl;
        }
        file.close();
    } else {
        std::cerr << "Unable to open file for saving ID mappings." << std::endl;
    }
}