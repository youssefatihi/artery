#include "CollisionAlertReceiverService.h"
#include "collision_msgs/CollisionWarningMessage_m.h"
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
void CollisionAlertReceiverService::initialize()
{
    ItsG5Service::initialize();
    mCollisionWarningSignal = registerSignal("CollisionWarning");
    mVehicleController = &getFacilities().get_mutable<traci::VehicleController>();
    logToFile(mVehicleController->getVehicleId(), "CollisionAlertReceiverService initialized");
}
void CollisionAlertReceiverService::indicate(const vanetza::btp::DataIndication& indication, cPacket* packet)
{
    logToFile(mVehicleController->getVehicleId(), "Received a packet in CollisionAlertReceiverService");
    auto* warning = dynamic_cast<CollisionWarningMessage*>(packet);
    if (warning) {
        logToFile(mVehicleController->getVehicleId(), "Packet is a CollisionWarningMessage");
        processCollisionWarning(warning);
    } else {
        logToFile(mVehicleController->getVehicleId(), "Packet is not a CollisionWarningMessage");
    }
    delete packet;
}
void CollisionAlertReceiverService::processCollisionWarning(const CollisionWarningMessage* warning)
{   

    logToFile(mVehicleController->getVehicleId(), 
              "Processing collision warning from " + std::string(warning->getSenderId()) + 
              ": TTC = " + std::to_string(warning->getTimeToCollision()) + 
              "s, Danger Level = " + std::to_string(warning->getDangerLevel()));
    if (isWarningRelevant(warning)) {
        logToFile(mVehicleController->getVehicleId(), 
                  "Received relevant collision warning from " + std::string(warning->getSenderId()) + 
                  ": TTC = " + std::to_string(warning->getTimeToCollision()) + 
                  "s, Danger Level = " + std::to_string(warning->getDangerLevel()));
        
        emit(mCollisionWarningSignal, warning->getDangerLevel());
        
        if (warning->getDangerLevel() == 2) {
            // Danger critique : freinage d'urgence
            mVehicleController->setSpeed(0);
            logToFile(mVehicleController->getVehicleId(), "Emergency braking initiated due to critical danger");
        } else if (warning->getDangerLevel() == 1) {
            // Avertissement : réduction de la vitesse
            auto newSpeed = mVehicleController->getSpeed() * 0.7;
            mVehicleController->setSpeed(newSpeed);
            logToFile(mVehicleController->getVehicleId(), 
                      "Reducing speed to " + std::to_string(newSpeed.value()) + " m/s due to collision warning");
        }
    } else {
        logToFile(mVehicleController->getVehicleId(), 
                  "Received irrelevant collision warning from " + std::string(warning->getSenderId()));
    }
}

bool CollisionAlertReceiverService::isWarningRelevant(const CollisionWarningMessage* warning)
{
    const auto& position = mVehicleController->getPosition();
    double distance = std::sqrt(std::pow(warning->getSenderPosX() - position.x.value(), 2) +
                                std::pow(warning->getSenderPosY() - position.y.value(), 2));
    
    // Considérer l'alerte comme pertinente si elle est à moins de 200 mètres
    return distance < 200.0;
}

} // namespace artery