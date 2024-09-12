#include "RSU.h"
#include "artery/utility/InitStages.h"
#include <vanetza/btp/ports.hpp>
#include <vanetza/dcc/profile.hpp>
#include <vanetza/units/length.hpp>
#include <vanetza/units/angle.hpp>
#include <vanetza/common/byte_order.hpp>
#include <vanetza/net/osi_layer.hpp>
#include <omnetpp.h>
#include "Utils.h"
#include <memory>
using namespace omnetpp;
using namespace artery;

Define_Module(RSU)

void RSU::initialize(int stage)
{
    StationaryMiddleware::initialize(stage);
    if (stage == InitStages::Prepare) {
        mPosition = Position(par("posX").doubleValue(), par("posY").doubleValue());
        mTransmissionRange = par("transmissionRange").doubleValue();
        mRsuId = "RSU_" + std::to_string(getId());
        logToFile(mRsuId, "RSU initialized at position: " + std::to_string(mPosition.x.value()) + ", " + std::to_string(mPosition.y.value()));
    }
}

void RSU::finish()
{
    StationaryMiddleware::finish();
    logToFile(mRsuId, "RSU finished");
}

void RSU::RelayService::initialize()
{
    ItsG5Service::initialize();
    logToFile(getRSU()->getStationId(), "RSURelayService initialized");
}

void RSU::RelayService::indicate(const vanetza::btp::DataIndication& indication, cPacket* packet)
{
    Enter_Method("RSURelayService indicate");

    auto* denm = dynamic_cast<DENMMessage*>(packet);
    if (denm) {
        logToFile(getRSU()->getStationId(), "Received DENM from station " + std::string(denm->getStationID()));
        relayDENM(denm);
    }

    delete packet;
}void RSU::RelayService::relayDENM(const DENMMessage* originalDenm)
{
    Enter_Method("RSURelayService relayDENM");

    // Créer une copie du message original
    auto message = new DENMMessage(*originalDenm);
    
    vanetza::geonet::Area destination;
    vanetza::geonet::Circle circle;
    
    auto rsu = getRSU();
    auto position = rsu->getPosition();
    auto transmissionRange = rsu->getTransmissionRange();

    circle.r = vanetza::units::Length(transmissionRange * vanetza::units::si::meters);
    destination.shape = circle;

    const double metersPerDegree = 111319.9;
    double latitudeDegree = position.x.value() / metersPerDegree;
    double longitudeDegree = position.y.value() / metersPerDegree;

    destination.position.latitude = vanetza::units::GeoAngle(latitudeDegree * vanetza::units::degree);
    destination.position.longitude = vanetza::units::GeoAngle(longitudeDegree * vanetza::units::degree);

    vanetza::btp::DataRequestB request;
    request.destination_port = vanetza::btp::port_type(2003);
    request.gn.transport_type = vanetza::geonet::TransportType::GBC;
    request.gn.destination = destination;
    request.gn.traffic_class.tc_id(static_cast<unsigned>(vanetza::dcc::Profile::DP2));
    request.gn.communication_profile = vanetza::geonet::CommunicationProfile::ITS_G5;

    std::stringstream ss;
    ss << "Relaying DENM: TTC = " << message->getTimeToCollision() 
       << "s, SubCauseCode = " << message->getSubCauseCode();
    logToFile(rsu->getStationId(), ss.str());

    // Envoyer le message
    this->request(request, message);
}
RSU* RSU::RelayService::getRSU()
{
    return check_and_cast<RSU*>(getParentModule()->getParentModule());
}