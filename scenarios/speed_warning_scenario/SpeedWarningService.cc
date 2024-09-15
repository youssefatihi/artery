#include "SpeedWarningService.h"
#include "artery/traci/VehicleController.h"
#include "artery/application/Middleware.h"
#include "artery/application/StoryboardSignal.h"
#include "artery/utility/InitStages.h"
#include "collision_msgs/DENMMessage_m.h"

#include <vanetza/btp/data_request.hpp>
#include <vanetza/dcc/profile.hpp>
#include <vanetza/geonet/interface.hpp>

#include <fstream>
#include <iomanip>
#include "Utils.h"

using namespace omnetpp;

Define_Module(SpeedWarningService)

void SpeedWarningService::initialize()
{
    ItsG5Service::initialize();
    mVehicleController = &getFacilities().get_const<traci::VehicleController>();
}

void SpeedWarningService::trigger()
{
    Enter_Method("SpeedWarningService trigger");
    checkSpeedLimit();
}

void SpeedWarningService::checkSpeedLimit()
{
    auto currentSpeed = mVehicleController->getSpeed().value() * 3.6; // Convert to km/h
    auto speedLimit = mVehicleController->getMaxSpeed().value() * 3.6; // Convert to km/h
    
    double speedExcess = currentSpeed - speedLimit;

    std::stringstream decisionInfo;
    decisionInfo << "Speed check results:\n";
    decisionInfo << "  Current speed: " << std::fixed << std::setprecision(2) << currentSpeed << " km/h\n";
    decisionInfo << "  Speed limit: " << std::fixed << std::setprecision(2) << speedLimit << " km/h\n";
    decisionInfo << "  Speed excess: " << std::fixed << std::setprecision(2) << speedExcess << " km/h\n";

    if (speedExcess > mSpeedLimitCriticalThreshold) {
        decisionInfo << "  Decision: Critical speed violation detected\n";
        logToFile(mVehicleController->getVehicleId(), decisionInfo.str());
        logToFile(mVehicleController->getVehicleId(), "Sending critical DENM (SubCauseCode 2)");
        sendDENM(speedExcess, 2); // Critical speed violation
    } 
    else if (speedExcess > mSpeedLimitWarningThreshold) {
        decisionInfo << "  Decision: Speed violation warning\n";
        logToFile(mVehicleController->getVehicleId(), decisionInfo.str());
        logToFile(mVehicleController->getVehicleId(), "Sending warning DENM (SubCauseCode 1)");
        sendDENM(speedExcess, 1); // Speed violation warning
    }
    else {
        decisionInfo << "  Decision: No speed violation detected\n";
        logToFile(mVehicleController->getVehicleId(), decisionInfo.str());
    }

    logToFile(mVehicleController->getVehicleId(), "");
}

void SpeedWarningService::sendDENM(double speedExcess, int subCauseCode)
{
    Enter_Method("SpeedWarningService sendDENM");

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
    message->setTermination(0); // 0 for new DENM
    
    // Situation Container
    message->setInformationQuality(7); // Highest quality
    message->setCauseCode(99); // Dangerous Situation
    message->setSubCauseCode(subCauseCode);
    
    // Location Container
    message->setEventPosition_latitude(mVehicleController->getPosition().x.value());
    message->setEventPosition_longitude(mVehicleController->getPosition().y.value());
    message->setEventSpeed(mVehicleController->getSpeed().value());
    message->setEventHeading(mVehicleController->getHeading().degree());
    
    // Alacarte Container
    message->setSpeedExcess(speedExcess);

    message->setByteLength(128); // Adjust as needed

    logToFile(mVehicleController->getVehicleId(), "Sending DENM: Speed Excess = " + std::to_string(speedExcess) + " km/h, SubCauseCode = " + std::to_string(subCauseCode));

    request(req, message);
}
