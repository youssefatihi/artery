#pragma once

#include <string>
#include <map>
#include <mutex>

class IdMapper
{
public:
    static IdMapper& getInstance();

    void addMapping(const std::string& sumoId, const std::string& stationId);
    std::string getStationId(const std::string& sumoId) const;
    std::string getSumoId(const std::string& stationId) const;

private:
    IdMapper() = default;
    ~IdMapper() = default;
    IdMapper(const IdMapper&) = delete;
    IdMapper& operator=(const IdMapper&) = delete;

    std::map<std::string, std::string> mSumoToStation;
    std::map<std::string, std::string> mStationToSumo;
    mutable std::mutex mMutex;
};
