#include "CollisionAlertReceiverService.h"
#include "collision_msgs/DENMMessage_m.h"
#include "artery/traci/VehicleController.h"
#include "artery/utility/IdentityRegistry.h"
#include "artery/utility/PointerCheck.h"
#include <omnetpp.h>
#include <boost/units/systems/si.hpp>
#include <boost/units/io.hpp>
#include "Utils.h"
#include <cmath>
#include <iomanip>

using namespace omnetpp;

namespace artery {

Define_Module(CollisionAlertReceiverService)

void CollisionAlertReceiverService::initialize() {
    ItsG5Service::initialize();
    mCollisionWarningSignal = registerSignal("CollisionWarning");
    mVehicleController = &getFacilities().get_mutable<traci::VehicleController>();

    // Ouvrir le fichier CSV pour l'enregistrement des données
    std::string dataDir = "/home/yelfatihi/artery/scenarios/collision_warning/data";
    int status = system(("mkdir -p " + dataDir).c_str());
    if (status == -1) {
        throw omnetpp::cRuntimeError("Failed to create data directory");
    }

    std::string filename = dataDir + "/collision_alert_" + mVehicleController->getVehicleId() + ".csv";
    mDataFile.open(filename, std::ios::out | std::ios::trunc);
    if (!mDataFile.is_open()) {
        throw omnetpp::cRuntimeError("Failed to open data file for writing: %s", filename.c_str());
    }
    mDataFile << "Time,VehicleID,Speed,AlertReceived,AlertType,SenderID,TTC,Distance,Action" << std::endl;

    logToFile(mVehicleController->getVehicleId(), "CollisionAlertReceiverService initialized");
}

void CollisionAlertReceiverService::indicate(const vanetza::btp::DataIndication& indication, cPacket* packet) {
    logToFile(mVehicleController->getVehicleId(), "Received a packet in CollisionAlertReceiverService");
    auto* denm = dynamic_cast<DENMMessage*>(packet);
    if (denm) {
        logToFile(mVehicleController->getVehicleId(), "Packet is a DENMMessage");
        mAlertReceived = true;
        if (mLastDENM) {
            delete mLastDENM;
        }
        mLastDENM = denm->dup();
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
            recordData(denm, "EmergencyBraking");
        } else if (denm->getSubCauseCode() == 1) {
            // Avertissement : réduction de la vitesse
            auto currentSpeed = mVehicleController->getSpeed();
            auto newSpeed = currentSpeed * 0.7;
            mVehicleController->setSpeed(newSpeed);
            logToFile(mVehicleController->getVehicleId(),
                      "Reducing speed from " + std::to_string(currentSpeed.value()) + 
                      " to " + std::to_string(newSpeed.value()) + " m/s due to collision warning");
            recordData(denm, "SpeedReduction");
        }
    } else {
        logToFile(mVehicleController->getVehicleId(),
                  "Received irrelevant DENM from " + std::string(denm->getStationID()));
        recordData(denm, "Ignored");
    }
}

bool CollisionAlertReceiverService::isDENMRelevant(const DENMMessage* denm) {
    const auto& position = mVehicleController->getPosition();
    double distance = std::sqrt(std::pow(denm->getEventPosition_latitude() - position.x.value(), 2) +
                                std::pow(denm->getEventPosition_longitude() - position.y.value(), 2));
    
    // Considérer l'alerte comme pertinente si elle est à moins de 200 mètres et si c'est un message de collision
    return distance < 200.0 && denm->getCauseCode() == 97;
}

void CollisionAlertReceiverService::recordData(const DENMMessage* denm, const std::string& action) {
    if (mDataFile.is_open()) {
        const auto& position = mVehicleController->getPosition();
        auto speed = mVehicleController->getSpeed();

        mDataFile << std::fixed << std::setprecision(3)
                  << simTime().dbl() << ","
                  << mVehicleController->getVehicleId() << ","
                  << speed.value() << ","
                  << (mAlertReceived ? "Yes" : "No") << ",";

        if (denm) {
            double distance = std::sqrt(std::pow(denm->getEventPosition_latitude() - position.x.value(), 2) +
                                        std::pow(denm->getEventPosition_longitude() - position.y.value(), 2));

            mDataFile << (denm->getSubCauseCode() == 2 ? "Critical" : 
                         (denm->getSubCauseCode() == 1 ? "Warning" : "Unknown")) << ","
                      << denm->getStationID() << ","
                      << denm->getTimeToCollision() << ","
                      << distance << ",";
        } else {
            mDataFile << "N/A,N/A,N/A,N/A,";
        }

        mDataFile << action << std::endl;
    }
}

void CollisionAlertReceiverService::trigger() {
    Enter_Method("CollisionAlertReceiverService trigger");
    
    if (mAlertReceived) {
        // Si une alerte a été reçue, on a déjà enregistré les données dans processDENM
        mAlertReceived = false;
    } else {
        // Si aucune alerte n'a été reçue, on enregistre quand même les données
        recordData();
    }
}

void CollisionAlertReceiverService::finish() {
    ItsG5Service::finish();
    if (mDataFile.is_open()) {
        mDataFile.close();
    }
    if (mLastDENM) {
        delete mLastDENM;
    }
}

} // namespace artery