#include "CollisionAlertReceiverService.h"
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

Define_Module(CollisionAlertReceiverService)

void CollisionAlertReceiverService::initialize() {
    ItsG5Service::initialize();
    mCollisionWarningSignal = registerSignal("CollisionWarning");
    mVehicleController = &getFacilities().get_mutable<traci::VehicleController>();
    logToFile(mVehicleController->getVehicleId(), "CollisionAlertReceiverService initialized");
}

void CollisionAlertReceiverService::indicate(const vanetza::btp::DataIndication& indication, cPacket* packet) {
    logToFile(mVehicleController->getVehicleId(), "Received a packet in CollisionAlertReceiverService");
    auto* denm = dynamic_cast<DENMMessage*>(packet);
    if (denm) {
        logToFile(mVehicleController->getVehicleId(), "Packet is a DENMMessage");
        processDENM(denm);
    } else {
        logToFile(mVehicleController->getVehicleId(), "Packet is not a DENMMessage");
    }
    delete packet;
}

void CollisionAlertReceiverService::processDENM(const DENMMessage* denm) {
    logToFile(mVehicleController->getVehicleId(),
              "Processing DENM from " + std::string(denm->getStationID()) +
              ": TTC = " + std::to_string(denm->getTimeToCollision()) +
              "s, CauseCode = " + std::to_string(denm->getCauseCode()) +
              ", SubCauseCode = " + std::to_string(denm->getSubCauseCode()));

    if (isDENMRelevant(denm)) {
        logToFile(mVehicleController->getVehicleId(),
                  "Received relevant DENM from " + std::string(denm->getStationID()) +
                  ": TTC = " + std::to_string(denm->getTimeToCollision()) +
                  "s, SubCauseCode = " + std::to_string(denm->getSubCauseCode()));
        
        emit(mCollisionWarningSignal, denm->getSubCauseCode());
        
        if (denm->getSubCauseCode() == 2) {
            // Danger critique : freinage d'urgence
            mVehicleController->setSpeed(0);
            logToFile(mVehicleController->getVehicleId(), "Emergency braking initiated due to critical danger");
        } else if (denm->getSubCauseCode() == 1) {
            // Avertissement : réduction de la vitesse
            auto newSpeed = mVehicleController->getSpeed() * 0.7;
            mVehicleController->setSpeed(newSpeed);
            logToFile(mVehicleController->getVehicleId(),
                      "Reducing speed to " + std::to_string(newSpeed.value()) + " m/s due to collision warning");
        }
    } else {
        logToFile(mVehicleController->getVehicleId(),
                  "Received irrelevant DENM from " + std::string(denm->getStationID()));
    }
}

bool CollisionAlertReceiverService::isDENMRelevant(const DENMMessage* denm) {
    const auto& position = mVehicleController->getPosition();
    double distance = std::sqrt(std::pow(denm->getEventPosition_latitude() - position.x.value(), 2) +
                                std::pow(denm->getEventPosition_longitude() - position.y.value(), 2));
    
    // Considérer l'alerte comme pertinente si elle est à moins de 200 mètres et si c'est un message de collision
    return distance < 200.0 && denm->getCauseCode() == 97;
}

} // namespace artery