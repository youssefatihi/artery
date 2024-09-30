#ifndef SPEEDALERTRECEIVERSERVICE_H_
#define SPEEDALERTRECEIVERSERVICE_H_

#include "artery/application/ItsG5Service.h"
#include "artery/application/Middleware.h"
#include "artery/envmod/LocalEnvironmentModel.h"

#include "artery/application/VehicleDataProvider.h"
#include "artery/traci/VehicleController.h"
#include "collision_msgs/DENMMessage_m.h"
#include <omnetpp.h>
#include <map>
namespace artery
{

class VehicleDataProvider;
class LocalEnvironmentModel;

class SpeedAlertReceiverService : public ItsG5Service
{
    public:
        void initialize() override;
        void indicate(const vanetza::btp::DataIndication&, omnetpp::cPacket*) override;

    private:
        void processDENM(const DENMMessage* denm);
        bool isDENMRelevant(const DENMMessage* denm);
        void adjustSpeed(double speedExcess, int subCauseCode);
        void increaseSafetyDistance();

    
        LocalEnvironmentModel* mLocalEnvironmentModel = nullptr;
        const VehicleDataProvider* mVehicleDataProvider = nullptr;
        traci::VehicleController* mVehicleController = nullptr;

        omnetpp::simsignal_t mSpeedWarningSignal;

        std::map<std::string, omnetpp::simtime_t> mLastAlertTimes;
        const omnetpp::simtime_t mAlertCooldown = omnetpp::SimTime(10, omnetpp::SIMTIME_S);
};

} // namespace artery

#endif
