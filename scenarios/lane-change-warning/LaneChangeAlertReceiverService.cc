#include "LaneChangeAlertReceiverService.h"
#include "lane_change_msgs/DENMMessage_m.h"
#include "artery/traci/VehicleController.h"
#include "artery/utility/IdentityRegistry.h"
#include "artery/utility/PointerCheck.h"
#include "Utils.h"
#include <omnetpp.h>
#include <boost/units/systems/si.hpp>
#include <boost/units/io.hpp>
#include <cmath>
#include <iomanip>

using namespace omnetpp;

namespace artery {

Define_Module(LaneChangeAlertReceiverService)

void LaneChangeAlertReceiverService::initialize() {
    ItsG5Service::initialize();
    mLaneChangeWarningSignal = registerSignal("LaneChangeWarning");
    mVehicleController = &getFacilities().get_mutable<traci::VehicleController>();

    std::string dataDir = "/home/yelfatihi/artery/scenarios/lane_change_warning/data";
    int status = system(("mkdir -p " + dataDir).c_str());
    if (status == -1) {
        throw omnetpp::cRuntimeError("Failed to create data directory");
    }

    std::string filename = dataDir + "/lane_change_alert_" + mVehicleController->getVehicleId() + ".csv";
    mDataFile.open(filename, std::ios::out | std::ios::trunc);
    if (!mDataFile.is_open()) {
        throw omnetpp::cRuntimeError("Failed to open data file for writing: %s", filename.c_str());
    }
    mDataFile << "Time,VehicleID,Speed,AlertReceived,AlertType,SenderID,Risk,CurrentLane,TargetLane,Action" << std::endl;

    logToFile(mVehicleController->getVehicleId(), "LaneChangeAlertReceiverService initialized");
}

void LaneChangeAlertReceiverService::indicate(const vanetza::btp::DataIndication& indication, cPacket* packet) {
    logToFile(mVehicleController->getVehicleId(), "Received a packet in LaneChangeAlertReceiverService");
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

void LaneChangeAlertReceiverService::processDENM(const DENMMessage* denm) {
    logToFile(mVehicleController->getVehicleId(),
              "Processing DENM from " + std::string(denm->getStationID()) +
              ": Risk = " + std::to_string(denm->getSubCauseCode()) +
              ", CauseCode = " + std::to_string(denm->getCauseCode()));

    if (isDENMRelevant(denm)) {
        logToFile(mVehicleController->getVehicleId(),
                  "Received relevant DENM from " + std::string(denm->getStationID()) +
                  ": Risk = " + std::to_string(denm->getSubCauseCode()) +
                  ", CurrentLane = " + std::to_string(denm->getCurrentLane()) +
                  ", TargetLane = " + std::to_string(denm->getTargetLane()));
        
    double riskValue = static_cast<double>(denm->getSubCauseCode()) / 100.0;
    emit(mLaneChangeWarningSignal, riskValue);        
        reactToLaneChange(denm);
    } else {
        logToFile(mVehicleController->getVehicleId(),
                  "Received irrelevant DENM from " + std::string(denm->getStationID()));
        recordData(denm, "Ignored");
    }
}

bool LaneChangeAlertReceiverService::isDENMRelevant(const DENMMessage* denm) {
    const auto& position = mVehicleController->getPosition();
    double distance = std::sqrt(std::pow(denm->getEventPosition_latitude() - position.x.value(), 2) +
                                std::pow(denm->getEventPosition_longitude() - position.y.value(), 2));
    
    // Considérer l'alerte comme pertinente si elle est à moins de 100 mètres et si c'est un message de changement de voie dangereux
    return distance < 100.0 && denm->getCauseCode() == 98;
}
void LaneChangeAlertReceiverService::reactToLaneChange(const DENMMessage* denm)
{
    double risk = denm->getRisk() / 100.0; // Convert back to 0-1 scale
    
    const std::string id = mVehicleController->getVehicleId();
    auto& vehicle_api = mVehicleController->getTraCI()->vehicle;
    int currentLane = vehicle_api.getLaneIndex(id);
    
    std::string action;
    if (risk > 0.7) {
        // Danger élevé : freinage d'urgence
        mVehicleController->setSpeed(0);
        action = "EmergencyBraking";
        logToFile(mVehicleController->getVehicleId(), "Emergency braking initiated due to high risk lane change");
    } else if (risk > 0.4) {
        // Risque modéré : réduction de la vitesse et changement de voie si possible
        auto currentSpeed = mVehicleController->getSpeed();
        auto newSpeed = currentSpeed * 0.7;
        mVehicleController->setSpeed(newSpeed);
        
        if (denm->getCurrentLane() == currentLane) {
            int newLane = currentLane + (currentLane > 0 ? -1 : 1);
            // vehicle_api.changeLane(id, newLane, SimTime(10, SIMTIME_S).dbl());
            action = "SpeedReduction_LaneChange";
        } else {
            action = "SpeedReduction";
        }
        
        logToFile(mVehicleController->getVehicleId(),
                  "Reducing speed and changing lane if necessary due to moderate risk lane change");
    } else {
        // Faible risque : être vigilant
        action = "IncreasedVigilance";
        logToFile(mVehicleController->getVehicleId(), "Increased vigilance due to low risk lane change");
    }
    
    recordData(denm, action);
}

void LaneChangeAlertReceiverService::recordData(const DENMMessage* denm, const std::string& action) {
    if (mDataFile.is_open()) {
        auto speed = mVehicleController->getSpeed();

        mDataFile << std::fixed << std::setprecision(3)
                  << simTime().dbl() << ","
                  << mVehicleController->getVehicleId() << ","
                  << speed.value() << ","
                  << (mAlertReceived ? "Yes" : "No") << ",";

        if (denm) {
            mDataFile << "LaneChange" << ","
                      << denm->getStationID() << ","
                      << denm->getSubCauseCode() / 100.0 << ","
                      << denm->getCurrentLane() << ","
                      << denm->getTargetLane() << ",";
        } else {
            mDataFile << "N/A,N/A,N/A,N/A,N/A,";
        }

        mDataFile << action << std::endl;
    }
}

void LaneChangeAlertReceiverService::trigger() {
    Enter_Method("LaneChangeAlertReceiverService trigger");
    
    if (mAlertReceived) {
        // Si une alerte a été reçue, on a déjà enregistré les données dans processDENM
        mAlertReceived = false;
    } else {
        // Si aucune alerte n'a été reçue, on enregistre quand même les données
        recordData();
    }
}

void LaneChangeAlertReceiverService::finish() {
    ItsG5Service::finish();
    if (mDataFile.is_open()) {
        mDataFile.close();
    }
    if (mLastDENM) {
        delete mLastDENM;
    }
}

} // namespace artery