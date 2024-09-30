#ifndef SPEEDMONITORINGSERVICE_H_
#define SPEEDMONITORINGSERVICE_H_

#include "artery/application/ItsG5Service.h"
#include "artery/application/VehicleDataProvider.h"
#include "artery/traci/VehicleController.h"
#include "DataCollector.h"
#include "IdMapper.h"
#include <omnetpp.h>
#include <map>
#include <string>

class SpeedMonitoringService : public artery::ItsG5Service
{
public:
    void trigger() override;

protected:
    void initialize() override;
    void finish() override;

private:
    const artery::VehicleDataProvider* mVehicleDataProvider = nullptr;
    const traci::VehicleController* mVehicleController = nullptr;

    const double mSpeedLimit = 100.0 / 3.6; // 120 km/h converted to m/s
    const double mSpeedLimitWarningThreshold = 10.0 / 3.6; // 10 km/h over speed limit
    const double mSpeedLimitCriticalThreshold = 20.0 / 3.6; // 20 km/h over speed limit

    std::map<std::string, std::pair<double, omnetpp::simtime_t>> mVehicleSpeeds;
    std::string mLogFilePath;

    void checkSpeedViolation(const std::string& vehicleId, double speed);
    void sendDENM(const std::string& vehicleId, double speed, int subCauseCode);
    void recordVehicleData();

    DataCollector* mDataCollector = nullptr;
    std::string mSumoId;
    std::string mStationId;
};

#endif /* SPEEDMONITORINGSERVICE_H_ */