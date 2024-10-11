#pragma once

#include <string>
#include <map>
#include <mutex>
#include <fstream>

class IdMapper
{
public:
    static IdMapper& getInstance();

    void addMapping(const std::string& sumoId, const std::string& stationId);
    std::string getStationId(const std::string& sumoId);
    std::string getSumoId(const std::string& stationId);

private:
    IdMapper();
    ~IdMapper() = default;
    IdMapper(const IdMapper&) = delete;
    IdMapper& operator=(const IdMapper&) = delete;

    void loadMappings();
    void saveMappings();

    std::map<std::string, std::string> mSumoToStation;
    std::map<std::string, std::string> mStationToSumo;
    mutable std::mutex mMutex;
    std::string mMappingFilePath;
};
