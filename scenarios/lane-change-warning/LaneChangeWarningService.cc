#include "LaneChangeWarningService.h"
#include "lane_change_msgs/DENMMessage_m.h"
#include "artery/traci/VehicleController.h"
#include "artery/envmod/sensor/Sensor.h"
#include "artery/envmod/EnvironmentModelObject.h"

#include "artery/utility/IdentityRegistry.h"
#include "artery/utility/Geometry.h"
#include <vanetza/btp/data_request.hpp>
#include <vanetza/dcc/profile.hpp>
#include <vanetza/geonet/interface.hpp>
#include "Utils.h"
#include <deque>
#include <map>

using namespace omnetpp;
using namespace vanetza;

Define_Module(LaneChangeWarningService)

void LaneChangeWarningService::initialize()
{
    ItsG5Service::initialize();
    mLocalEnvironmentModel = &getFacilities().get_const<artery::LocalEnvironmentModel>();
    mVehicleController = &getFacilities().get_const<traci::VehicleController>();
    
    mLaneChangeTime = simTime() + SimTime(par("laneChangeDelay").doubleValue());
    
    const std::string id = mVehicleController->getVehicleId();
    auto& vehicle_api = mVehicleController->getTraCI()->vehicle;
    mCurrentLane = vehicle_api.getLaneIndex(id);
    mTargetLane = mCurrentLane + 1;  // Changement vers la voie de droite par défaut

    logToFile(mVehicleController->getVehicleId(), "LaneChangeWarningService initialized");
}

void LaneChangeWarningService::trigger()
{
    Enter_Method("LaneChangeWarningService trigger");

    if (simTime() >= mLaneChangeTime && !mLaneChangeInitiated) {
        initiateLaneChange();
    }

    if (mLaneChangeInitiated) {
        checkLaneChangeRisk();
    }
}

void LaneChangeWarningService::initiateLaneChange()
{
    mLaneChangeInitiated = true;
    auto& vehicle_api = mVehicleController->getTraCI()->vehicle;
    const std::string id = mVehicleController->getVehicleId();
    vehicle_api.changeLane(id, mTargetLane, SimTime(10, SIMTIME_S).dbl());
    logToFile(mVehicleController->getVehicleId(), "Lane change initiated to lane " + std::to_string(mTargetLane));
}

void LaneChangeWarningService::checkLaneChangeRisk()
{
    const auto& objects = mLocalEnvironmentModel->allObjects();
    double maxRisk = 0.0;
    std::string dangerVehicleId;
    
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

            logToFile(mVehicleController->getVehicleId(), objectDetails.str());

            double ttc = calculateTimeToCollision(tracking);
            double risk = calculateRisk(distance, ttc, tracking);
            
            if (risk > maxRisk) {
                maxRisk = risk;
                dangerVehicleId = object->getExternalId();
            }

            if (risk > par("riskThreshold").doubleValue()) {
                int subCauseCode = static_cast<int>(risk * 100);  // Convert risk to 0-100 scale
                sendLaneChangeDENM(risk, subCauseCode);
            }
        }
    }

    logToFile(mVehicleController->getVehicleId(), "Max risk: " + std::to_string(maxRisk) + 
              ", Danger vehicle ID: " + dangerVehicleId);
}
double LaneChangeWarningService::calculateRisk(double distance, double ttc, const artery::LocalEnvironmentModel::Tracking& tracking)
{
    const auto& hostPosition = mVehicleController->getPosition();
    const auto& hostSpeed = mVehicleController->getSpeed();
    
    // Facteur de distance
    double distanceFactor = 1.0 - std::min(1.0, distance / mMaxDetectionDistance);

    // Facteur de TTC
    double ttcFactor = (ttc < mMinTTC) ? 1.0 : (mMinTTC / ttc);

    // Facteur de vitesse relative
    double estimatedSpeed = 0.0;
    auto& history = mObjectHistories[tracking.id()];
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

    double relativeSpeed = std::abs(estimatedSpeed - hostSpeed.value());
    double speedFactor = std::min(1.0, relativeSpeed / mMaxRelativeSpeed);

    // Calcul du risque global
    double risk = (mW1 * distanceFactor +
                   mW2 * ttcFactor +
                   mW3 * speedFactor);

    // Normaliser le risque entre 0 et 1
    risk = std::min(1.0, std::max(0.0, risk));

    logToFile(mVehicleController->getVehicleId(), 
              "Risk calculation: Distance factor = " + std::to_string(distanceFactor) +
              ", TTC factor = " + std::to_string(ttcFactor) +
              ", Speed factor = " + std::to_string(speedFactor) +
              ", Estimated speed = " + std::to_string(estimatedSpeed) +
              ", Total risk = " + std::to_string(risk));

    return risk;
}
double LaneChangeWarningService::calculateTimeToCollision(const artery::LocalEnvironmentModel::Tracking& tracking)
{
    const auto& hostPosition = mVehicleController->getPosition();
    const auto& hostSpeed = mVehicleController->getSpeed();

    std::stringstream debugInfo;
    debugInfo << "-------------------Calculating TTC for lane change------------------:\n";
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
void LaneChangeWarningService::sendLaneChangeDENM(double risk, int subCauseCode)
{
    auto message = new DENMMessage();
    
    message->setProtocolVersion(1);
    message->setMessageID(1);
    message->setStationID(mVehicleController->getVehicleId().c_str());
    
    message->setDetectionTime(simTime());
    message->setReferenceTime(simTime());
    message->setTermination(0);
    
    message->setInformationQuality(7);
    message->setCauseCode(98); // dangerousLaneChange
    message->setSubCauseCode(subCauseCode);
    
    message->setEventPosition_latitude(mVehicleController->getPosition().x.value());
    message->setEventPosition_longitude(mVehicleController->getPosition().y.value());
    message->setEventSpeed(mVehicleController->getSpeed().value());
    message->setEventHeading(mVehicleController->getHeading().degree());

    // Ajout des champs personnalisés
    const std::string id = mVehicleController->getVehicleId();
    auto& vehicle_api = mVehicleController->getTraCI()->vehicle;

    message->setTimeToCollision(mMinTTC); // Utilisez la valeur appropriée ici
    message->setCurrentLane(vehicle_api.getLaneIndex(id));
    message->setTargetLane(mTargetLane);
    
    // Pour la vitesse latérale, nous devons la calculer car elle n'est pas directement disponible
    double lateralSpeed = 0.0; // À calculer si possible
    message->setLateralSpeed(lateralSpeed);
    
    message->setLongitudinalSpeed(mVehicleController->getSpeed().value());

    message->setRisk(risk * 100); // Convertir en échelle 0-100
    
    // Vérifiez si le véhicule est un véhicule d'urgence
    bool isEmergency = (vehicle_api.getVehicleClass(id) == "emergency");
    message->setEmergencyVehicle(isEmergency);

    message->setByteLength(128);

    btp::DataRequestB req;
    req.destination_port = host_cast<port_type>(getPortNumber());
    req.gn.transport_type = geonet::TransportType::SHB;
    req.gn.traffic_class.tc_id(static_cast<unsigned>(dcc::Profile::DP2));
    req.gn.communication_profile = geonet::CommunicationProfile::ITS_G5;

    logToFile(mVehicleController->getVehicleId(), "Sending DENM: Risk = " + std::to_string(risk) + 
              ", Current Lane = " + std::to_string(vehicle_api.getLaneIndex(id)) + 
              ", Target Lane = " + std::to_string(mTargetLane));

    request(req, message);
}