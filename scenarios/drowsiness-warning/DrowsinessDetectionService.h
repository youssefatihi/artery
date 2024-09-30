#ifndef DROWSINESSDETECTIONSERVICE_H_
#define DROWSINESSDETECTIONSERVICE_H_

#include "artery/application/ItsG5Service.h"
#include "artery/traci/VehicleController.h"
#include <omnetpp.h>

class DrowsinessDetectionService : public artery::ItsG5Service
{
public:
    void trigger() override;

protected:
    void initialize() override;

private:
    double mDrowsinessLevel = 0.0;
    double mWarningThreshold = 0.6;  // 60% de somnolence
    double mCriticalThreshold = 0.8;  // 80% de somnolence
    
    omnetpp::simtime_t mLastUpdateTime;
    const traci::VehicleController* mVehicleController = nullptr;
    
    void updateDrowsinessLevel();
    void checkDrowsinessLevel();
    void sendDrowsinessDENM(int alertType);
    double estimateReactionTime() const;
};

#endif /* DROWSINESSDETECTIONSERVICE_H_ */