#include "PoliceService.h"
#include "police_msgs/PoliceClearLane_m.h"
#include "artery/traci/VehicleController.h"
#include <vanetza/btp/data_request.hpp>
#include <vanetza/dcc/profile.hpp>
#include <vanetza/geonet/interface.hpp>

#include <boost/units/systems/si.hpp>
#include <boost/units/io.hpp>
#include <fstream>
#include <iomanip>
#include <ctime>
#include <sstream>
#include <filesystem>
#include <sys/stat.h>
#include <unistd.h>
#include "Utils.h"
using namespace omnetpp;
using namespace vanetza;

Define_Module(PoliceService)



void PoliceService::initialize()
{
    ItsG5BaseService::initialize();
    mVehicleController = &getFacilities().get_const<traci::VehicleController>();
    logToFile(mVehicleController->getVehicleId(), " PoliceService initialized");

}

void PoliceService::trigger()
{
    Enter_Method("PoliceService trigger");
    btp::DataRequestB req;
    req.destination_port = host_cast<PoliceService::port_type>(getPortNumber());
    req.gn.transport_type = geonet::TransportType::SHB;
    req.gn.traffic_class.tc_id(static_cast<unsigned>(dcc::Profile::DP1));
    req.gn.communication_profile = geonet::CommunicationProfile::ITS_G5;

    const std::string id = mVehicleController->getVehicleId();
    auto& vehicle_api = mVehicleController->getTraCI()->vehicle;

    auto packet = new PoliceClearLane();
    packet->setEdgeName(vehicle_api.getRoadID(id).c_str());
    packet->setLaneIndex(vehicle_api.getLaneIndex(id));
    packet->setByteLength(40);
        logToFile(mVehicleController->getVehicleId(), "Sending clear lane request");

    request(req, packet);
}
