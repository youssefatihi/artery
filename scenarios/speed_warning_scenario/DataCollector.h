#ifndef DATACOLLECTOR_H
#define DATACOLLECTOR_H

#include <string>
#include <fstream>
#include <mutex>
#include <map>

class DataCollector {
public:
    DataCollector(const std::string& vehicleDataPath, const std::string& denmDataPath);
    ~DataCollector();

    void recordVehicleData(int sumoId, double time, double speed, const std::pair<double, double>& position, bool receivedDenm, const std::string& action);
    void recordDenmData(double time, int senderSumoId, int senderStationId, double detectedSpeed, const std::pair<double, double>& position, const std::string& violationType);

    void addIdMapping(int sumoId, int stationId);

private:
    std::string m_vehicleDataPath;
    std::string m_denmDataPath;
    std::ofstream m_denmFile;
    std::mutex m_denmMutex;
    std::map<int, int> m_idMapping;  // SUMO ID to Station ID mapping

    std::ofstream& getVehicleFile(int sumoId);
    std::map<int, std::ofstream> m_vehicleFiles;
};

#endif // DATACOLLECTOR_H