#include "CollisionWarningService.h"
#include "collision_msgs/CollisionWarningMessage_m.h"
#include "artery/traci/VehicleController.h"
#include "artery/utility/IdentityRegistry.h"
#include "artery/utility/Geometry.h"
#include <vanetza/btp/data_request.hpp>
#include <vanetza/dcc/profile.hpp>
#include <vanetza/geonet/interface.hpp>
#include <vanetza/units/velocity.hpp>
#include <vanetza/units/length.hpp>
#include "artery/envmod/LocalEnvironmentModel.h"
#include "artery/envmod/EnvironmentModelObject.h"
#include "artery/envmod/sensor/Sensor.h"
#include <stdexcept>
#include "artery/traci/VehicleController.h"
#include <limits>
#include <cmath>
#include "Utils.h"

using namespace omnetpp;
using namespace vanetza;

Define_Module(CollisionWarningService)



void CollisionWarningService::initialize()
{
    ItsG5Service::initialize();
    
    try {
        mVehicleController = &getFacilities().get_const<traci::VehicleController>();
        logToFile(mVehicleController->getVehicleId(), "VehicleController found in Facilities");
    } catch (const std::out_of_range& e) {
        logToFile(mVehicleController->getVehicleId(), "VehicleController not found in Facilities: " + std::string(e.what()));
    }
    
    try {
        mLocalEnvironmentModel = &getFacilities().get_const<artery::LocalEnvironmentModel>();
        logToFile(mVehicleController->getVehicleId(), "LocalEnvironmentModel found in Facilities");
    } catch (const std::out_of_range& e) {
        logToFile(mVehicleController->getVehicleId(), "LocalEnvironmentModel not found in Facilities: " + std::string(e.what()));
    }
    
    logToFile(mVehicleController->getVehicleId(), "CollisionWarningService initialized");
}

void CollisionWarningService::trigger()
{
    Enter_Method("CollisionWarningService trigger");
    checkCollisionRisk(); }
double CollisionWarningService::calculateTimeToCollision(const artery::LocalEnvironmentModel::Tracking& tracking)
{
    const auto& hostPosition = mVehicleController->getPosition();
    const auto& hostSpeed = mVehicleController->getSpeed();

    std::stringstream debugInfo;
    debugInfo << "-------------------Calculating ------------------:\n";
    
    debugInfo << "Calculating TTC:\n";
    debugInfo << "  Host position: (" << hostPosition.x.value() << ", " << hostPosition.y.value() << ")\n";
    debugInfo << "  Host speed: " << hostSpeed.value() << " m/s\n";
    debugInfo << "  Tracking ID: " << tracking.id() << "\n";

    const auto& sensors = tracking.sensors();
    debugInfo << "  Number of sensors: " << sensors.size() << "\n";
    
    if (sensors.empty()) {
        debugInfo << "  No sensors detected the object. Returning infinity.\n";
        logToFile(mVehicleController->getVehicleId(), debugInfo.str());
        return std::numeric_limits<double>::infinity();
    }

    const auto& sensor = sensors.begin()->first;
    const auto& trackingTime = sensors.begin()->second;

    debugInfo << "  Sensor type: " << sensor->getSensorCategory() << "\n";
    debugInfo << "  Tracking time: " << trackingTime.first().dbl() << " to " << trackingTime.last().dbl() << "\n";

    artery::SensorDetection detection = sensor->detectObjects();
    
    debugInfo << "  Number of detected objects: " << detection.objects.size() << "\n";

    for (const auto& obj : detection.objects) {
        debugInfo << "    Object ID: " << obj->getExternalId() << "\n";
        debugInfo << "    Object Position: (" << obj->getCentrePoint().x.value() << ", " << obj->getCentrePoint().y.value() << ")\n";
    }

    const artery::EnvironmentModelObject* detectedObject = nullptr;
    for (const auto& obj : detection.objects) {
        
            detectedObject = obj.get();
            break;
    }

    if (!detectedObject) {
        debugInfo << "  Detected object not found. Returning infinity.\n";
        logToFile(mVehicleController->getVehicleId(), debugInfo.str());
        return std::numeric_limits<double>::infinity();
    }

    const auto& objectPosition = detectedObject->getCentrePoint();
    
    double dx = objectPosition.x.value() - hostPosition.x.value();
    double dy = objectPosition.y.value() - hostPosition.y.value();
    double relativeDistance = std::sqrt(dx*dx + dy*dy);

    debugInfo << "  Object position: (" << objectPosition.x.value() << ", " << objectPosition.y.value() << ")\n";
    debugInfo << "  Relative distance: " << relativeDistance << " m\n";

    auto timeSinceLastUpdate = (omnetpp::simTime() - trackingTime.last()).dbl();
    debugInfo << "  Time since last update: " << timeSinceLastUpdate << " s\n";

    double estimatedSpeed = 0;
    if (timeSinceLastUpdate > 0) {
        estimatedSpeed = relativeDistance / timeSinceLastUpdate;
        debugInfo << "  Estimated object speed: " << estimatedSpeed << " m/s\n";
    } else {
        debugInfo << "  Unable to estimate object speed (time since last update is 0)\n";
    }

    double relativeSpeed = std::abs(estimatedSpeed - hostSpeed.value());
    debugInfo << "  Relative speed: " << relativeSpeed << " m/s\n";

    double ttc;
    if (relativeSpeed > 0) {
        ttc = relativeDistance / relativeSpeed;
        debugInfo << "  Calculated TTC: " << ttc << " s\n";
    } else {
        ttc = std::numeric_limits<double>::infinity();
        debugInfo << "  Relative speed is 0. TTC is infinity.\n";
    }

    logToFile(mVehicleController->getVehicleId(), debugInfo.str());
    return ttc;
}

void CollisionWarningService::checkCollisionRisk()
{
    const auto& objects = mLocalEnvironmentModel->allObjects();
    
    logToFile(mVehicleController->getVehicleId(), "Total objects detected: " + std::to_string(objects.size()));

    for (const auto& pair : objects) {
        const auto& objectPtr = pair.first;
        const auto& tracking = pair.second;
        if (auto object = objectPtr.lock()) {
            std::stringstream objectDetails;
            objectDetails << "Processing object ID: " << tracking.id() << "\n";
            
            const auto& objectPosition = object->getCentrePoint();
            const auto& hostPosition = mVehicleController->getPosition();
            
            double dx = objectPosition.x.value() - hostPosition.x.value();
            double dy = objectPosition.y.value() - hostPosition.y.value();
            double distance = std::sqrt(dx*dx + dy*dy);
            
            objectDetails << "  Position: (" << objectPosition.x.value() << ", " << objectPosition.y.value() << ")\n"
                          << "  Distance: " << distance << " m\n"
                          << "  Object external ID: " << object->getExternalId() << "\n";

            // Log sensor information
            const auto& sensors = tracking.sensors();
            objectDetails << "  Number of sensors detecting this object: " << sensors.size() << "\n";
            for (const auto& sensorPair : sensors) {
                const auto& sensor = sensorPair.first;
                objectDetails << "    Sensor type: " << sensor->getSensorCategory() << "\n";
            }

            logToFile(mVehicleController->getVehicleId(), objectDetails.str());

            double ttc = calculateTimeToCollision(tracking);

            std::stringstream decisionInfo;
            decisionInfo << "TTC calculation results:\n";
            decisionInfo << "  Calculated TTC: " << ttc << "s\n";

            if (ttc < mCriticalThreshold) {
                decisionInfo << "  Decision: Critical risk detected\n";
                logToFile(mVehicleController->getVehicleId(), decisionInfo.str());
                
                logToFile(mVehicleController->getVehicleId(), "Sending critical collision warning message (Danger Level 2)");
                sendCollisionWarning(ttc, 2); // Critical danger
            } 
            else if (ttc < mWarningThreshold) {
                decisionInfo << "  Decision: Warning risk detected\n";
                logToFile(mVehicleController->getVehicleId(), decisionInfo.str());
                
                logToFile(mVehicleController->getVehicleId(), "Sending collision warning message (Danger Level 1)");
                sendCollisionWarning(ttc, 1); // Warning
            }
            else {
                decisionInfo << "  Decision: No significant risk detected\n";
                logToFile(mVehicleController->getVehicleId(), decisionInfo.str());
            }

            logToFile(mVehicleController->getVehicleId(), "");
        }
    }
}
void CollisionWarningService::sendCollisionWarning(double ttc, int dangerLevel)
{
    Enter_Method("CollisionWarningService sendCollisionWarning");

    btp::DataRequestB req;
    req.destination_port = host_cast<port_type>(getPortNumber());
    req.gn.transport_type = geonet::TransportType::SHB;
    req.gn.traffic_class.tc_id(static_cast<unsigned>(dcc::Profile::DP2));
    req.gn.communication_profile = geonet::CommunicationProfile::ITS_G5;

    auto message = new CollisionWarningMessage();
    message->setSenderId(mVehicleController->getVehicleId().c_str());
    message->setSenderPosX(mVehicleController->getPosition().x.value());
    message->setSenderPosY(mVehicleController->getPosition().y.value());
    message->setSenderSpeed(mVehicleController->getSpeed().value());
    message->setSenderHeading(mVehicleController->getHeading().degree());
    message->setTimeToCollision(ttc);
    message->setDangerLevel(dangerLevel);
    message->setByteLength(64);

    logToFile(mVehicleController->getVehicleId(), "Sending collision warning: TTC = " + std::to_string(ttc) + "s, Danger Level = " + std::to_string(dangerLevel));

    request(req, message);
}