#ifndef SPEEDALERTRECEIVERSERVICE_H_
#define SPEEDALERTRECEIVERSERVICE_H_

#include "artery/application/ItsG5Service.h"
#include "artery/traci/VehicleController.h"

#include <omnetpp.h>
#include <vanetza/btp/data_indication.hpp>

class DENMMessage;

namespace artery {

class SpeedAlertReceiverService : public ItsG5Service
{
public:
    void initialize() override;
    void indicate(const vanetza::btp::DataIndication&, omnetpp::cPacket*) override;

protected:
    void processDENM(const DENMMessage* denm);
    bool isDENMRelevant(const DENMMessage* denm);

private:
    omnetpp::simsignal_t mSpeedWarningSignal;
    traci::VehicleController* mVehicleController = nullptr;
};

} // namespace artery

#endif /* SPEEDALERTRECEIVERSERVICE_H_ */