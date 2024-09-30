#ifndef DROWSINESSALERTRECEIVERSERVICE_H_
#define DROWSINESSALERTRECEIVERSERVICE_H_

#include "artery/application/ItsG5Service.h"
#include "artery/traci/VehicleController.h"
#include <omnetpp.h>
#include <vanetza/btp/data_indication.hpp>
#include <fstream>

class DENMMessage;

namespace artery {

class DrowsinessAlertReceiverService : public ItsG5Service
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

private:
    omnetpp::simsignal_t mDrowsinessWarningSignal;
    traci::VehicleController* mVehicleController = nullptr;
    std::ofstream mDataFile;
    bool mAlertReceived = false;
    DENMMessage* mLastDENM = nullptr;
};

} // namespace artery

#endif /* DROWSINESSALERTRECEIVERSERVICE_H_ */