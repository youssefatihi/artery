#include "DrowsinessDetectionService.h"
#include "drowsiness_msgs/DENMMessage_m.h"
#include "artery/utility/IdentityRegistry.h"
#include <vanetza/btp/data_request.hpp>
#include <vanetza/dcc/profile.hpp>
#include <vanetza/geonet/interface.hpp>
#include "Utils.h"
#include <fstream>
#include <iomanip>

using namespace omnetpp;
using namespace vanetza;

Define_Module(DrowsinessDetectionService)

void DrowsinessDetectionService::initialize()
{
    ItsG5Service::initialize();
    mVehicleController = &getFacilities().get_const<traci::VehicleController>();
    mLastUpdateTime = simTime();
    
    // Initialisation du système de collecte de données
    std::string dataDir = "/home/yelfatihi/artery/scenarios/drowsiness-warning/data";
    int status = system(("mkdir -p " + dataDir).c_str());
    if (status == -1) {
        throw omnetpp::cRuntimeError("Failed to create data directory");
    }

    std::string filename = dataDir + "/drowsiness_data_" + mVehicleController->getVehicleId() + ".csv";
    mDataFile.open(filename, std::ios::out | std::ios::trunc);
    if (!mDataFile.is_open()) {
        throw omnetpp::cRuntimeError("Failed to open data file for writing: %s", filename.c_str());
    }
    mDataFile << "Time,DrowsinessLevel,AlertType" << std::endl;
    
    logToFile(mVehicleController->getVehicleId(), "DrowsinessDetectionService initialized");
}

void DrowsinessDetectionService::finish()
{
    ItsG5Service::finish();
    if (mDataFile.is_open()) {
        mDataFile.close();
    }
}

void DrowsinessDetectionService::trigger()
{
    Enter_Method("DrowsinessDetectionService trigger");
    updateDrowsinessLevel();
    checkDrowsinessLevel();
    recordData();
}

void DrowsinessDetectionService::updateDrowsinessLevel()
{
    const double MAX_DROWSINESS_RATE = 0.1;  // Maximum d'augmentation par heure
    const double RANDOM_FACTOR = 0.05;  // Facteur aléatoire maximal

    simtime_t now = simTime();
    double elapsedHours = (now - mLastUpdateTime).dbl() / 15.0;  
    // Augmentation basée sur le temps
    double increase = elapsedHours * MAX_DROWSINESS_RATE;

    // Facteur aléatoire
    double randomFactor = uniform(-RANDOM_FACTOR/2, RANDOM_FACTOR);

    mDrowsinessLevel += increase + randomFactor;
    mDrowsinessLevel = std::max(0.0, std::min(1.0, mDrowsinessLevel));  // Limiter entre 0 et 1

    mLastUpdateTime = now;

    logToFile(mVehicleController->getVehicleId(), "Updated drowsiness level: " + std::to_string(mDrowsinessLevel));
}

void DrowsinessDetectionService::checkDrowsinessLevel()
{
    mCurrentAlertType = 0;  // Par défaut, pas d'alerte
    if (mDrowsinessLevel >= mCriticalThreshold) {
        logToFile(mVehicleController->getVehicleId(), "Critical drowsiness detected");
        sendDrowsinessDENM(2);  // Alerte critique
        mCurrentAlertType = 2;
    } else if (mDrowsinessLevel >= mWarningThreshold) {
        logToFile(mVehicleController->getVehicleId(), "Warning drowsiness detected");
        sendDrowsinessDENM(1);  // Avertissement
        mCurrentAlertType = 1;
    }
}

void DrowsinessDetectionService::recordData()
{
    if (mDataFile.is_open()) {
        mDataFile << std::fixed << std::setprecision(3)
                  << simTime().dbl() << ","
                  << mDrowsinessLevel << ","
                  << mCurrentAlertType << std::endl;
    }
}

double DrowsinessDetectionService::estimateReactionTime() const
{
    // Estimation simple : le temps de réaction augmente avec le niveau de somnolence
    const double BASE_REACTION_TIME = 0.5;  // Temps de réaction de base en secondes
    const double MAX_ADDITIONAL_TIME = 2.0;  // Temps additionnel maximal

    return BASE_REACTION_TIME + (mDrowsinessLevel * MAX_ADDITIONAL_TIME);
}

void DrowsinessDetectionService::sendDrowsinessDENM(int alertType)
{
    Enter_Method("DrowsinessDetectionService sendDrowsinessDENM");

    btp::DataRequestB req;
    req.destination_port = host_cast<port_type>(getPortNumber());
    req.gn.transport_type = geonet::TransportType::SHB;
    req.gn.traffic_class.tc_id(static_cast<unsigned>(dcc::Profile::DP2));
    req.gn.communication_profile = geonet::CommunicationProfile::ITS_G5;

    auto message = new DENMMessage();
    
    // Header
    message->setProtocolVersion(1);
    message->setMessageID(1); // DENM messageID
    message->setStationID(mVehicleController->getVehicleId().c_str());
    
    // Management Container
    message->setDetectionTime(simTime());
    message->setReferenceTime(simTime());
    message->setTermination(0); // 0 pour nouveau DENM
    
    // Situation Container
    message->setInformationQuality(7); // Qualité la plus élevée
    message->setCauseCode(93); // human problem 
    message->setSubCauseCode(0); 
    
    // Location Container
    message->setEventPosition_latitude(mVehicleController->getPosition().x.value());
    message->setEventPosition_longitude(mVehicleController->getPosition().y.value());
    message->setEventSpeed(mVehicleController->getSpeed().value());
    message->setEventHeading(mVehicleController->getHeading().degree());
    
    // Alacarte Container
    message->setDrowsinessLevel(mDrowsinessLevel);
    message->setAlertType(alertType);
    message->setReactionTime(estimateReactionTime());

    message->setByteLength(128); // 128 bytes 

    logToFile(mVehicleController->getVehicleId(), "Sending Drowsiness DENM: Level = " + std::to_string(mDrowsinessLevel) + 
              ", AlertType = " + std::to_string(alertType) + 
              ", ReactionTime = " + std::to_string(estimateReactionTime()));

    request(req, message);
}