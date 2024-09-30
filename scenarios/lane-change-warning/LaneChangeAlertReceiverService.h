#ifndef LANECHANGEALERTRECEIVERSERVICE_H_
#define LANECHANGEALERTRECEIVERSERVICE_H_

#include "artery/application/ItsG5Service.h"
#include "artery/traci/VehicleController.h"

#include <omnetpp.h>
#include <vanetza/btp/data_indication.hpp>
#include <fstream>

class DENMMessage;

namespace artery {

class LaneChangeAlertReceiverService : public ItsG5Service
{
public:
    void initialize() override;
    void indicate(const vanetza::btp::DataIndication&, omnetpp::cPacket*) override;
    void finish() override;
    void trigger() override;

protected:
    void processDENM(const DENMMessage* denm);
    bool isDENMRelevant(const DENMMessage* denm);
    void recordData(const DENMMessage* denm = nullptr, const std::string& action = "NoAlert");
    void reactToLaneChange(const DENMMessage* denm);

private:
    omnetpp::simsignal_t mLaneChangeWarningSignal;
    traci::VehicleController* mVehicleController = nullptr;
    std::ofstream mDataFile;
    bool mAlertReceived = false;
    DENMMessage* mLastDENM = nullptr;
};

} // namespace artery

#endif /* LANECHANGEALERTRECEIVERSERVICE_H_ */