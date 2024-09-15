#ifndef SPEEDALERTRECEIVERSERVICE_H_
#define SPEEDALERTRECEIVERSERVICE_H_

#include "artery/application/ItsG5Service.h"
#include "artery/traci/VehicleController.h"

#include <omnetpp.h>
#include <vanetza/btp/data_indication.hpp>

class SpeedWarningMessage;

namespace artery {

class SpeedAlertReceiverService : public ItsG5Service
{
public:
    void initialize() override;
    void indicate(const vanetza::btp::DataIndication&, omnetpp::cPacket*) override;

protected:
    void processDENM(const SpeedWarningMessage* denm);
    bool isDENMRelevant(const SpeedWarningMessage* denm);

private:
    omnetpp::simsignal_t mSpeedWarningSignal;
    traci::VehicleController* mVehicleController = nullptr;
};

} // namespace artery

#endif /* SPEEDALERTRECEIVERSERVICE_H_ */