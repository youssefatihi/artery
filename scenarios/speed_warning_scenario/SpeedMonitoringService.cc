#include "SpeedMonitoringService.h"
#include "artery/application/Middleware.h"
#include "artery/application/StoryboardSignal.h"
#include "artery/utility/InitStages.h"
#include "artery/traci/VehicleController.h"
#include "artery/application/VehicleDataProvider.h"
#include "collision_msgs/DENMMessage_m.h"
#include "Utils.h"
#include <vanetza/btp/data_request.hpp>
#include <vanetza/dcc/profile.hpp>
#include <vanetza/geonet/interface.hpp>
#include <fstream>
#include <sstream>
#include <iostream>

using namespace omnetpp;
using namespace vanetza;
using namespace artery;

Define_Module(SpeedMonitoringService)

void SpeedMonitoringService::initialize()
{
    ItsG5Service::initialize();
    mVehicleController = &getFacilities().get_const<traci::VehicleController>();
    mVehicleDataProvider = &getFacilities().get_const<VehicleDataProvider>();

    mSumoId = mVehicleController->getVehicleId();
    mStationId = std::to_string(mVehicleDataProvider->station_id());

    mLogFilePath = "logs/vehicle_" + mStationId + "_data.txt";
    
    // Initialize DataCollector
    mDataCollector = new DataCollector("data");
    mDataCollector->initializeVehicleFile(mSumoId, mStationId);

    // Add mapping to IdMapper
    IdMapper::getInstance().addMapping(mSumoId, mStationId);

    logToFile(mSumoId, "SpeedMonitoringService initialized. Monitoring log file: " + mLogFilePath);
}

void SpeedMonitoringService::finish()
{
    delete mDataCollector;
}

void SpeedMonitoringService::trigger()
{
    Enter_Method("SpeedMonitoringService trigger");

    recordVehicleData();
    std::ifstream logFile(mLogFilePath);
    if (logFile.is_open()) {
        std::string line;
        while (std::getline(logFile, line)) {
            std::istringstream iss(line);
            std::string timestamp, vehicleId, speed, heading, longitude, latitude;

            try {
                if (std::getline(iss, timestamp, ',') &&
                    std::getline(iss, vehicleId, ',') &&
                    std::getline(iss, speed, ',') &&
                    std::getline(iss, heading, ',') &&
                    std::getline(iss, longitude, ',') &&
                    std::getline(iss, latitude)) {

                    double speedValue = std::stod(speed);
                    double timestampValue = std::stod(timestamp);
                    double x = std::stod(longitude);
                    double y = std::stod(latitude);
                    
                    logToFile(mSumoId, "Checking speed violation for vehicle " + vehicleId + " at " + std::to_string(timestampValue) + "s");
                    checkSpeedViolation(vehicleId, speedValue);
                } else {
                    logToFile(mSumoId, "Invalid log entry format: " + line);
                }
            } catch (const std::invalid_argument& e) {
                // logToFile(mSumoId, "Error parsing log entry: " + line + " - " + e.what());
            } catch (const std::out_of_range& e) {
                logToFile(mSumoId, "Number out of range in log entry: " + line + " - " + e.what());
            }
        }
        logFile.close();
    } else {
        logToFile(mSumoId, "Unable to open log file: " + mLogFilePath);
    }

    // Clean up old speed records
    auto now = simTime();
    for (auto it = mVehicleSpeeds.begin(); it != mVehicleSpeeds.end(); ) {
        if (now - it->second.second > 10) { // Remove records older than 10 seconds
            it = mVehicleSpeeds.erase(it);
        } else {
            ++it;
        }
    }

}

void SpeedMonitoringService::checkSpeedViolation(const std::string& vehicleId, double speed)
{
    mVehicleSpeeds[vehicleId] = std::make_pair(speed, simTime());

    double speedExcess = speed - mSpeedLimit;
    int subCauseCode = 0;

    std::stringstream decisionInfo;
    decisionInfo << "Speed check results for vehicle " << vehicleId << ":\n";
    decisionInfo << "  Current speed: " << std::fixed << std::setprecision(2) << speed * 3.6 << " km/h\n";
    decisionInfo << "  Speed limit: " << std::fixed << std::setprecision(2) << mSpeedLimit * 3.6 << " km/h\n";
    decisionInfo << "  Speed excess: " << std::fixed << std::setprecision(2) << speedExcess * 3.6 << " km/h\n";

    if (speedExcess > mSpeedLimitCriticalThreshold) {
        decisionInfo << "  Decision: Critical speed violation detected\n";
        logToFile(mSumoId, decisionInfo.str());
        logToFile(mSumoId, "Sending critical DENM (SubCauseCode 2)");
        subCauseCode = 2;
        sendDENM(vehicleId, speed, 2); // Critical speed violation
    } 
    else if (speedExcess > mSpeedLimitWarningThreshold) {
        decisionInfo << "  Decision: Speed violation warning\n";
        logToFile(mSumoId, decisionInfo.str());
        logToFile(mSumoId, "Sending warning DENM (SubCauseCode 1)");
        subCauseCode = 1;
        sendDENM(vehicleId, speed, 1); // Speed violation warning
    }
    else {
        decisionInfo << "  Decision: No speed violation detected\n";
        logToFile(mSumoId, decisionInfo.str());
    }

    // Record DENM data
    const auto& position = mVehicleController->getPosition();
    std::string violationType = (subCauseCode == 2) ? "Critical" : "Warning";
    // Get the station ID of the offending vehicle
    std::string offenderStationId = IdMapper::getInstance().getStationId(vehicleId);
    
    mDataCollector->recordDenmData(simTime().dbl(), mSumoId, mStationId, vehicleId, offenderStationId, speed, 
                                   position.x.value(), position.y.value(), violationType);
    logToFile(mSumoId, "");
}

void SpeedMonitoringService::sendDENM(const std::string& vehicleId, double speed, int subCauseCode)
{
    Enter_Method("SpeedMonitoringService sendDENM");

    vanetza::btp::DataRequestB req;
    req.destination_port = host_cast<port_type>(getPortNumber());
    req.gn.transport_type = geonet::TransportType::SHB;
    req.gn.traffic_class.tc_id(static_cast<unsigned>(dcc::Profile::DP2));
    req.gn.communication_profile = geonet::CommunicationProfile::ITS_G5;

    auto message = new DENMMessage();
    
    // Header
    message->setProtocolVersion(1);
    message->setMessageID(1); // DENM messageID
    message->setStationID(mSumoId.c_str());
    
    // Management Container
    message->setDetectionTime(simTime());
    message->setReferenceTime(simTime());
    message->setTermination(0); // 0 for new DENM
    
    // Situation Container
    message->setInformationQuality(7); // Highest quality
    message->setCauseCode(94); // Dangerous Situation
    message->setSubCauseCode(subCauseCode);
    
    // Location Container
    message->setEventPosition_latitude(mVehicleController->getPosition().x.value());
    message->setEventPosition_longitude(mVehicleController->getPosition().y.value());
    message->setEventSpeed(speed);
    message->setEventHeading(mVehicleController->getHeading().degree());
    
    // Alacarte Container
    message->setRelevanceDistance(speed - mSpeedLimit);

    // New fields
    message->setOffendingVehicleId(vehicleId.c_str());
    message->setCurrentSpeed(speed);
    message->setSpeedLimit(mSpeedLimit);

    message->setByteLength(128); // Adjust as needed

    logToFile(mSumoId, "Sending DENM: Offending Vehicle = " + vehicleId + 
              ", Speed = " + std::to_string(speed * 3.6) + " km/h, SubCauseCode = " + std::to_string(subCauseCode));

    request(req, message);

}

void SpeedMonitoringService::recordVehicleData()
{
    const auto& position = mVehicleController->getPosition();
    double speed = mVehicleController->getSpeed().value();

    mDataCollector->recordVehicleData(mSumoId, simTime().dbl(), speed, 
                                      position.x.value(), position.y.value()
                                      );
}