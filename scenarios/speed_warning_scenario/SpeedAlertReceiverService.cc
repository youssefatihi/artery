#include "SpeedAlertReceiverService.h"
#include "collision_msgs/DENMMessage_m.h"
#include "artery/traci/VehicleController.h"
#include "artery/application/VehicleDataProvider.h"
#include "artery/utility/IdentityRegistry.h"
#include "artery/utility/PointerCheck.h"
#include "Utils.h"
#include <omnetpp.h>
#include <boost/units/systems/si.hpp>
#include <boost/units/io.hpp>

using namespace omnetpp;

namespace artery
{

Define_Module(SpeedAlertReceiverService)

void SpeedAlertReceiverService::initialize()
{
    ItsG5Service::initialize();
    mSpeedWarningSignal = registerSignal("SpeedWarning");
    mVehicleController = &getFacilities().get_mutable<traci::VehicleController>();
    mVehicleDataProvider = &getFacilities().get_const<VehicleDataProvider>();
    mLocalEnvironmentModel = &getFacilities().get_mutable<LocalEnvironmentModel>();
    logToFile(std::to_string(mVehicleDataProvider->station_id()), "SpeedAlertReceiverService initialized");
}

void SpeedAlertReceiverService::indicate(const vanetza::btp::DataIndication& indication, cPacket* packet)
{
    logToFile(std::to_string(mVehicleDataProvider->station_id()), "Received a packet in SpeedAlertReceiverService");
    auto* denm = dynamic_cast<DENMMessage*>(packet);
    if (denm) {
        logToFile(std::to_string(mVehicleDataProvider->station_id()), "Packet is a DENMMessage");
        processDENM(denm);
    } else {
        logToFile(std::to_string(mVehicleDataProvider->station_id()), "Packet is not a DENMMessage");
    }
    delete packet;
}

void SpeedAlertReceiverService::processDENM(const DENMMessage* denm)
{
    logToFile(std::to_string(mVehicleDataProvider->station_id()),
              "Processing DENM from " + std::string(denm->getStationID()) +
              ": Offending Vehicle = " + std::string(denm->getOffendingVehicleId()) +
              ", Current Speed = " + std::to_string(denm->getCurrentSpeed() * 3.6) +
              " km/h, Speed Limit = " + std::to_string(denm->getSpeedLimit() * 3.6) +
              " km/h, SubCauseCode = " + std::to_string(denm->getSubCauseCode()));

    if (isDENMRelevant(denm)) {
        std::string myStationId = std::to_string(mVehicleDataProvider->station_id());
        
        if (myStationId == denm->getOffendingVehicleId()) {
            logToFile(std::to_string(mVehicleDataProvider->station_id()), "I am the offending vehicle. Adjusting speed.");
            double speedExcess = denm->getCurrentSpeed() - denm->getSpeedLimit();
            adjustSpeed(speedExcess, denm->getSubCauseCode());
        } else {
            logToFile(std::to_string(mVehicleDataProvider->station_id()), "Another vehicle is speeding. Increasing safety distance.");
            increaseSafetyDistance();
        }

        emit(mSpeedWarningSignal, denm->getSubCauseCode());
    } else {
        logToFile(std::to_string(mVehicleDataProvider->station_id()),
                  "Received irrelevant DENM from " + std::string(denm->getStationID()));
    }
}
bool SpeedAlertReceiverService::isDENMRelevant(const DENMMessage* denm)
{
    const auto& position = mVehicleController->getPosition();
    double distance = std::sqrt(std::pow(denm->getEventPosition_latitude() - position.x.value(), 2) +
                                std::pow(denm->getEventPosition_longitude() - position.y.value(), 2));

    std::string myStationId = std::to_string(mVehicleDataProvider->station_id());
    
    // Check if we've recently processed an alert for this vehicle
    auto lastAlertIt = mLastAlertTimes.find(denm->getOffendingVehicleId());
    if (lastAlertIt != mLastAlertTimes.end() && 
        (simTime() - lastAlertIt->second) < mAlertCooldown) {
        return false;
    }

    // Update the last alert time for this vehicle
    mLastAlertTimes[denm->getOffendingVehicleId()] = simTime();

    // Consider the alert relevant if it's within 500 meters, it's a speed violation message,
    // and either we are the offending vehicle or it's close enough to affect us
    return distance < 500.0 && denm->getCauseCode() == 94 && 
           (myStationId == denm->getOffendingVehicleId() || distance < 100.0);
}

void SpeedAlertReceiverService::adjustSpeed(double speedExcess, int subCauseCode)
{
    auto currentSpeed = mVehicleController->getSpeed();
    auto newSpeed = currentSpeed;

    if (subCauseCode == 2) {
        // Critical danger: emergency speed reduction
        newSpeed = currentSpeed * 0.5;
        logToFile(std::to_string(mVehicleDataProvider->station_id()),
                  "Emergency speed reduction to " + std::to_string(newSpeed.value()) + " m/s due to critical speed violation");
    } else if (subCauseCode == 1) {
        // Warning: moderate speed reduction
        newSpeed = currentSpeed * 0.8;
        logToFile(std::to_string(mVehicleDataProvider->station_id()),
                  "Reducing speed to " + std::to_string(newSpeed.value()) + " m/s due to speed violation warning");
    }

    mVehicleController->setSpeed(newSpeed);
}

void SpeedAlertReceiverService::increaseSafetyDistance()
{
    // Implement logic to increase safety distance
    // This could involve adjusting the car-following model parameters in SUMO
    // For simplicity, we'll just log the action here
    logToFile(std::to_string(mVehicleDataProvider->station_id()),
              "Increasing safety distance due to nearby speed violation");
}

} // namespace artery