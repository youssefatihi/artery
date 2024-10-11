#ifndef DROWSINESSDETECTIONSERVICE_H_
#define DROWSINESSDETECTIONSERVICE_H_

#include "artery/application/ItsG5Service.h"
#include "artery/traci/VehicleController.h"
#include <omnetpp.h>
#include <fstream>


class DrowsinessDetectionService : public artery::ItsG5Service
{
public:
    void trigger() override;
    void finish() override;

protected:
    void initialize() override;

private:
    std::ofstream mDataFile;
    int mCurrentAlertType = 0;
    void recordData();
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