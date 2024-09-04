#ifndef COLLISIONALERTRECEIVERSERVICE_H_
#define COLLISIONALERTRECEIVERSERVICE_H_

#include "artery/application/ItsG5Service.h"
#include "artery/traci/VehicleController.h"

#include <omnetpp.h>
#include <vanetza/btp/data_indication.hpp>

class CollisionWarningMessage;

namespace artery {

class CollisionAlertReceiverService : public ItsG5Service
{
public:
    void initialize() override;
    void indicate(const vanetza::btp::DataIndication&, omnetpp::cPacket*) override;

protected:
    void processCollisionWarning(const CollisionWarningMessage* warning);
    bool isWarningRelevant(const CollisionWarningMessage* warning);

private:
    omnetpp::simsignal_t mCollisionWarningSignal;
    traci::VehicleController* mVehicleController = nullptr;
};

} // namespace artery

#endif /* COLLISIONALERTRECEIVERSERVICE_H_ */