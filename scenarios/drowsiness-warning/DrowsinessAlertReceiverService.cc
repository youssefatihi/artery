#include "DrowsinessAlertReceiverService.h"
#include "drowsiness_msgs/DENMMessage_m.h"
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

Define_Module(DrowsinessAlertReceiverService)

void DrowsinessAlertReceiverService::initialize()
{
    ItsG5Service::initialize();
    mDrowsinessWarningSignal = registerSignal("DrowsinessWarning");
    mVehicleController = &getFacilities().get_mutable<traci::VehicleController>();

    // Ouvrir le fichier CSV pour l'enregistrement des données
    std::string dataDir = "/home/yelfatihi/artery/scenarios/drowsiness_warning/data";
    int status = system(("mkdir -p " + dataDir).c_str());
    if (status == -1) {
        throw omnetpp::cRuntimeError("Failed to create data directory");
    }

    std::string filename = dataDir + "/drowsiness_alert_" + mVehicleController->getVehicleId() + ".csv";
    mDataFile.open(filename, std::ios::out | std::ios::trunc);
    if (!mDataFile.is_open()) {
        throw omnetpp::cRuntimeError("Failed to open data file for writing: %s", filename.c_str());
    }
    mDataFile << "Time,VehicleID,Speed,AlertReceived,AlertType,SenderID,DrowsinessLevel,ReactionTime,Distance,Action" << std::endl;

    logToFile(mVehicleController->getVehicleId(), "DrowsinessAlertReceiverService initialized");
}

void DrowsinessAlertReceiverService::indicate(const vanetza::btp::DataIndication& indication, cPacket* packet)
{
    logToFile(mVehicleController->getVehicleId(), "Received a packet in DrowsinessAlertReceiverService");
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

void DrowsinessAlertReceiverService::processDENM(const DENMMessage* denm)
{
    logToFile(mVehicleController->getVehicleId(),
              "Processing DENM from " + std::string(denm->getStationID()) +
              ": DrowsinessLevel = " + std::to_string(denm->getDrowsinessLevel()) +
              ", AlertType = " + std::to_string(denm->getAlertType()) +
              ", ReactionTime = " + std::to_string(denm->getReactionTime()));

    if (isDENMRelevant(denm)) {
        emit(mDrowsinessWarningSignal, denm->getAlertType());
        
        if (denm->getAlertType() == 2) {
            // Danger critique : réduction importante de la vitesse
            auto currentSpeed = mVehicleController->getSpeed();
            auto newSpeed = currentSpeed * 0.5;
            mVehicleController->setSpeed(newSpeed);
            logToFile(mVehicleController->getVehicleId(),
                      "Reducing speed significantly from " + std::to_string(currentSpeed.value()) + 
                      " to " + std::to_string(newSpeed.value()) + " m/s due to critical drowsiness alert");
            recordData(denm, "CriticalSpeedReduction");
        } else if (denm->getAlertType() == 1) {
            // Avertissement : légère réduction de la vitesse
            auto currentSpeed = mVehicleController->getSpeed();
            auto newSpeed = currentSpeed * 0.8;
            mVehicleController->setSpeed(newSpeed);
            logToFile(mVehicleController->getVehicleId(),
                      "Reducing speed slightly from " + std::to_string(currentSpeed.value()) + 
                      " to " + std::to_string(newSpeed.value()) + " m/s due to drowsiness warning");
            recordData(denm, "SlightSpeedReduction");
        }
    } else {
        logToFile(mVehicleController->getVehicleId(),
                  "Received irrelevant DENM from " + std::string(denm->getStationID()));
        recordData(denm, "Ignored");
    }
}

bool DrowsinessAlertReceiverService::isDENMRelevant(const DENMMessage* denm)
{
    const auto& position = mVehicleController->getPosition();
    double distance = std::sqrt(std::pow(denm->getEventPosition_latitude() - position.x.value(), 2) +
                                std::pow(denm->getEventPosition_longitude() - position.y.value(), 2));
    if (distance >500.0 ) {
        logToFile(mVehicleController->getVehicleId(), "DENM is not relevant: distance = " + std::to_string(distance));
    }
    if (denm->getCauseCode() != 91) {
        logToFile(mVehicleController->getVehicleId(), "DENM is not relevant: cause code = " + std::to_string(denm->getCauseCode()));
    }
    // Considérer l'alerte comme pertinente si elle est à moins de 500 mètres et si c'est un message de somnolence
    return distance < 500.0 && denm->getCauseCode() == 91;
}

void DrowsinessAlertReceiverService::recordData(const DENMMessage* denm, const std::string& action)
{
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

            mDataFile << (denm->getAlertType() == 2 ? "Critical" : 
                         (denm->getAlertType() == 1 ? "Warning" : "Unknown")) << ","
                      << denm->getStationID() << ","
                      << denm->getDrowsinessLevel() << ","
                      << denm->getReactionTime() << ","
                      << distance << ",";
        } else {
            mDataFile << "N/A,N/A,N/A,N/A,N/A,";
        }

        mDataFile << action << std::endl;
    }
}

void DrowsinessAlertReceiverService::trigger()
{
    Enter_Method("DrowsinessAlertReceiverService trigger");
    
    if (mAlertReceived) {
        // Si une alerte a été reçue, on a déjà enregistré les données dans processDENM
        mAlertReceived = false;
    } else {
        // Si aucune alerte n'a été reçue, on enregistre quand même les données
        recordData();
    }
}

void DrowsinessAlertReceiverService::finish()
{
    ItsG5Service::finish();
    if (mDataFile.is_open()) {
        mDataFile.close();
    }
    if (mLastDENM) {
        delete mLastDENM;
    }
}

} // namespace artery