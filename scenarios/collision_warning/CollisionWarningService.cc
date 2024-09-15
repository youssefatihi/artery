#include "CollisionWarningService.h"
#include "collision_msgs/DENMMessage_m.h"
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
#include <vanetza/units/angle.hpp>
#include <boost/units/systems/si/plane_angle.hpp>
#include <boost/units/systems/angle/degrees.hpp>
#include <stdexcept>
#include <limits>
#include <cmath>
#include "Utils.h"

using namespace omnetpp;
using namespace vanetza;

Define_Module(CollisionWarningService)

struct ObjectHistory {
    std::deque<std::pair<omnetpp::simtime_t, artery::Position>> positions;
    double lastEstimatedSpeed = 0.0;
};
std::map<int, ObjectHistory> mObjectHistories;


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
    checkCollisionRisk();
}
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

    const artery::EnvironmentModelObject* detectedObject = nullptr;
    for (const auto& obj : detection.objects) {
        debugInfo << "    Object ID: " << obj->getExternalId() << "\n";
        debugInfo << "    Object Position: (" << obj->getCentrePoint().x.value() << ", " << obj->getCentrePoint().y.value() << ")\n";
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

    // Mise à jour de l'historique de l'objet
    auto& history = mObjectHistories[tracking.id()];
    history.positions.emplace_back(omnetpp::simTime(), objectPosition);
    
    // Garder seulement les 10 dernières positions
    if (history.positions.size() > 10) {
        history.positions.pop_front();
    }

    double estimatedSpeed = 0;
    if (history.positions.size() > 1) {
        const auto& oldestPosition = history.positions.front();
        const auto& newestPosition = history.positions.back();
        
        double totalDistance = std::sqrt(
            std::pow(newestPosition.second.x.value() - oldestPosition.second.x.value(), 2) +
            std::pow(newestPosition.second.y.value() - oldestPosition.second.y.value(), 2)
        );
        double totalTime = (newestPosition.first - oldestPosition.first).dbl();
        
        if (totalTime > 0) {
            estimatedSpeed = totalDistance / totalTime;
        }
    }

    // Appliquer un filtre passe-bas pour lisser l'estimation de la vitesse
    const double ALPHA = 0.2;  // Facteur de lissage
    history.lastEstimatedSpeed = ALPHA * estimatedSpeed + (1 - ALPHA) * history.lastEstimatedSpeed;
    estimatedSpeed = history.lastEstimatedSpeed;

    // Vérification de cohérence
    const double MAX_REASONABLE_SPEED = 60.0; // m/s, environ 216 km/h
    if (estimatedSpeed > MAX_REASONABLE_SPEED) {
        debugInfo << "  Warning: Estimated speed exceeds maximum reasonable speed. Capping at " << MAX_REASONABLE_SPEED << " m/s\n";
        estimatedSpeed = MAX_REASONABLE_SPEED;
    }

    debugInfo << "  Estimated object speed: " << estimatedSpeed << " m/s\n";

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
                
                logToFile(mVehicleController->getVehicleId(), "Sending critical DENM (SubCauseCode 2)");
                sendDENM(ttc, 2); // Critical danger
            } 
            else if (ttc < mWarningThreshold) {
                decisionInfo << "  Decision: Warning risk detected\n";
                logToFile(mVehicleController->getVehicleId(), decisionInfo.str());
                
                logToFile(mVehicleController->getVehicleId(), "Sending warning DENM (SubCauseCode 1)");
                sendDENM(ttc, 1); // Warning
            }
            else {
                decisionInfo << "  Decision: No significant risk detected\n";
                logToFile(mVehicleController->getVehicleId(), decisionInfo.str());
            }

            logToFile(mVehicleController->getVehicleId(), "");
        }
    }
}

void CollisionWarningService::sendDENM(double ttc, int subCauseCode)
{
    Enter_Method("CollisionWarningService sendDENM");

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
    message->setCauseCode(97); // Collision Risk
    message->setSubCauseCode(subCauseCode);
    
    // Location Container
    message->setEventPosition_latitude(mVehicleController->getPosition().x.value());
    message->setEventPosition_longitude(mVehicleController->getPosition().y.value());
    message->setEventSpeed(mVehicleController->getSpeed().value());
    message->setEventHeading(mVehicleController->getHeading().degree());
    
    // Alacarte Container
    message->setTimeToCollision(ttc);

    message->setByteLength(128); // 128 bytes 

    logToFile(mVehicleController->getVehicleId(), "Sending DENM: TTC = " + std::to_string(ttc) + "s, SubCauseCode = " + std::to_string(subCauseCode));

    request(req, message);
}

//DENM to RSU (pas encore implémenté)
void CollisionWarningService::sendDENMtoRSU(double ttc, int subCauseCode)
{
    Enter_Method("CollisionWarningService sendDENMtoRSU");

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
    message->setCauseCode(97); // Collision Risk
    message->setSubCauseCode(subCauseCode);
    
    // Location Container
    message->setEventPosition_latitude(mVehicleController->getPosition().x.value());
    message->setEventPosition_longitude(mVehicleController->getPosition().y.value());
    message->setEventSpeed(mVehicleController->getSpeed().value());
    message->setEventHeading(mVehicleController->getHeading().degree());
    
    // Alacarte Container
    message->setTimeToCollision(ttc);

    message->setByteLength(128);
    
    vanetza::geonet::Area destination;
    vanetza::geonet::Circle circle;
    circle.r = vanetza::units::Length(1000 * vanetza::units::si::meters);
    destination.shape = circle;
    
    auto position = mVehicleController->getPosition();
    

// Convertir les mètres en degrés (approximation)
const double metersPerDegree = 111319.9; // Approximation à l'équateur
double latitudeDegrees = position.x.value() / metersPerDegree;
double longitudeDegrees = position.y.value() / metersPerDegree;

// Convertir les degrés en GeoAngle
destination.position.latitude = vanetza::units::GeoAngle(latitudeDegrees * vanetza::units::degree);
destination.position.longitude = vanetza::units::GeoAngle(longitudeDegrees * vanetza::units::degree);

    vanetza::btp::DataRequestB request;
    request.destination_port = host_cast<port_type>(getPortNumber());
    request.gn.transport_type = vanetza::geonet::TransportType::GBC;
    request.gn.destination = destination;
    request.gn.traffic_class.tc_id(static_cast<unsigned>(vanetza::dcc::Profile::DP2));
    request.gn.communication_profile = vanetza::geonet::CommunicationProfile::ITS_G5;

    // Utilisez la méthode request du service ItsG5 directement
    this->request(request, std::move(message));

    EV_INFO << "Sending DENM to RSU: TTC = " << ttc << "s, SubCauseCode = " << subCauseCode << "\n";
}