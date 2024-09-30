
#include "IdMapper.h"

IdMapper& IdMapper::getInstance()
{
    static IdMapper instance;
    return instance;
}

void IdMapper::addMapping(const std::string& sumoId, const std::string& stationId)
{
    std::lock_guard<std::mutex> lock(mMutex);
    mSumoToStation[sumoId] = stationId;
    mStationToSumo[stationId] = sumoId;
}

std::string IdMapper::getStationId(const std::string& sumoId) const
{
    std::lock_guard<std::mutex> lock(mMutex);
    auto it = mSumoToStation.find(sumoId);
    return (it != mSumoToStation.end()) ? it->second : "";
}

std::string IdMapper::getSumoId(const std::string& stationId) const
{
    std::lock_guard<std::mutex> lock(mMutex);
    auto it = mStationToSumo.find(stationId);
    return (it != mStationToSumo.end()) ? it->second : "";
}