#include "SpeedAlertReceiverService.h"
#include "collision_msgs/DENMMessage_m.h"
#include "artery/traci/VehicleController.h"
#include "artery/utility/IdentityRegistry.h"
#include "artery/utility/PointerCheck.h"
#include <omnetpp.h>
#include <boost/units/systems/si.hpp>
#include <boost/units/io.hpp>
#include "Utils.h"

using namespace omnetpp;

namespace artery {

Define_Module(SpeedAlertReceiverService)

void SpeedAlertReceiverService::initialize() {
    ItsG5Service::initialize();
    mSpeedWarningSignal = registerSignal("SpeedWarning");
    mVehicleController = &getFacilities().get_mutable<traci::VehicleController>();
    logToFile(mVehicleController->getVehicleId(), "SpeedAlertReceiverService initialized");
}

void SpeedAlertReceiverService::indicate(const vanetza::btp::DataIndication& indication, cPacket* packet) {
    logToFile(mVehicleController->getVehicleId(), "Received a packet in SpeedAlertReceiverService");
    auto* denm = dynamic_cast<DENMMessage*>(packet);
    if (denm) {
        logToFile(mVehicleController->getVehicleId(), "Packet is a DENMMessage");
        processDENM(denm);
    } else {
        logToFile(mVehicleController->getVehicleId(), "Packet is not a DENMMessage");
    }
    delete packet;
}

void SpeedAlertReceiverService::processDENM(const DENMMessage* denm) {
    logToFile(mVehicleController->getVehicleId(),
              "Processing DENM from " + std::string(denm->getStationID()) +
              ": Speed Excess = " + std::to_string(denm->getSpeedExcess()) +
              " km/h, CauseCode = " + std::to_string(denm->getCauseCode()) +
              ", SubCauseCode = " + std::to_string(denm->getSubCauseCode()));

    if (isDENMRelevant(denm)) {
        logToFile(mVehicleController->getVehicleId(),
                  "Received relevant DENM from " + std::string(denm->getStationID()) +
                  ": Speed Excess = " + std::to_string(denm->getSpeedExcess()) +
                  " km/h, SubCauseCode = " + std::to_string(denm->getSubCauseCode()));
        
        emit(mSpeedWarningSignal, denm->getSubCauseCode());
        
        if (denm->getSubCauseCode() == 2) {
            // Critical danger: reduce speed significantly
            auto newSpeed = mVehicleController->getSpeed() * 0.5;
            mVehicleController->setSpeed(newSpeed);
            logToFile(mVehicleController->getVehicleId(),
                      "Reducing speed to " + std::to_string(newSpeed.value()) + " m/s due to critical speed warning");
        } else if (denm->getSubCauseCode() == 1) {
            // Warning: slight speed reduction
            auto newSpeed = mVehicleController->getSpeed() * 0.8;
            mVehicleController->setSpeed(newSpeed);
            logToFile(mVehicleController->getVehicleId(),
                      "Reducing speed to " + std::to_string(newSpeed.value()) + " m/s due to speed warning");
        }
    } else {
        logToFile(mVehicleController->getVehicleId(),
                  "Received irrelevant DENM from " + std::string(denm->getStationID()));
    }
}

bool SpeedAlertReceiverService::isDENMRelevant(const DENMMessage* denm) {
    const auto& position = mVehicleController->getPosition();
    double distance = std::sqrt(std::pow(denm->getEventPosition_latitude() - position.x.value(), 2) +
                                std::pow(denm->getEventPosition_longitude() - position.y.value(), 2));
    
    // Consider the alert relevant if it's within 500 meters and it's a speed warning message
    return distance < 500.0 && denm->getCauseCode() == 94;
}

} // namespace artery