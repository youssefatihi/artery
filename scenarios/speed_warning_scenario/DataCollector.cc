#include "DataCollector.h"
#include <iostream>
#include <iomanip>

DataCollector::DataCollector(const std::string& vehicleDataPath, const std::string& denmDataPath)
    : m_vehicleDataPath(vehicleDataPath), m_denmDataPath(denmDataPath) {
    m_denmFile.open(m_denmDataPath, std::ios::out | std::ios::trunc);
    if (!m_denmFile.is_open()) {
        throw std::runtime_error("Unable to open DENM data file: " + m_denmDataPath);
    }
    m_denmFile << "Time,SenderSumoId,SenderStationId,DetectedSpeed,PositionX,PositionY,ViolationType\n";
}

DataCollector::~DataCollector() {
    m_denmFile.close();
    for (auto& pair : m_vehicleFiles) {
        pair.second.close();
    }
}

void DataCollector::recordVehicleData(int sumoId, double time, double speed, const std::pair<double, double>& position, bool receivedDenm, const std::string& action) {
    std::ofstream& file = getVehicleFile(sumoId);
    file << std::fixed << std::setprecision(2)
         << time << ","
         << speed << ","
         << position.first << ","
         << position.second << ","
         << (receivedDenm ? "1" : "0") << ","
         << action << "\n";
}

void DataCollector::recordDenmData(double time, int senderSumoId, int senderStationId, double detectedSpeed, const std::pair<double, double>& position, const std::string& violationType) {
    std::lock_guard<std::mutex> lock(m_denmMutex);
    m_denmFile << std::fixed << std::setprecision(2)
               << time << ","
               << senderSumoId << ","
               << senderStationId << ","
               << detectedSpeed << ","
               << position.first << ","
               << position.second << ","
               << violationType << "\n";
}

void DataCollector::addIdMapping(int sumoId, int stationId) {
    m_idMapping[sumoId] = stationId;
}

std::ofstream& DataCollector::getVehicleFile(int sumoId) {
    auto it = m_vehicleFiles.find(sumoId);
    if (it == m_vehicleFiles.end()) {
        std::string filename = m_vehicleDataPath + "vehicle_data_" + std::to_string(sumoId) + ".csv";
        auto result = m_vehicleFiles.emplace(sumoId, std::ofstream(filename, std::ios::out | std::ios::trunc));
        if (!result.second) {
            throw std::runtime_error("Unable to create vehicle data file: " + filename);
        }
        std::ofstream& file = result.first->second;
        file << "Time,Speed,PositionX,PositionY,ReceivedDenm,Action\n";
        return file;
    }
    return it->second;
}