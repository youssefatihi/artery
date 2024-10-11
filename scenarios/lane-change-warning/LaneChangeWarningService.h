#ifndef LANECHANGEWARNINGSERVICE_H_
#define LANECHANGEWARNINGSERVICE_H_

#include "artery/application/ItsG5Service.h"
#include "artery/envmod/LocalEnvironmentModel.h"
#include "artery/traci/VehicleController.h"
#include <vanetza/units/velocity.hpp>
#include <vanetza/units/length.hpp>
#include <omnetpp.h>
#include <deque>
#include <map>

struct ObjectHistory {
    std::deque<std::pair<omnetpp::simtime_t, artery::Position>> positions;
    double lastEstimatedSpeed = 0.0;
};

class LaneChangeWarningService : public artery::ItsG5Service
{
public:
    void trigger() override;

protected:
    void initialize() override;

private:
    const artery::LocalEnvironmentModel* mLocalEnvironmentModel = nullptr;
    const traci::VehicleController* mVehicleController = nullptr;
    double calculateRisk(double distance, double ttc, const artery::LocalEnvironmentModel::Tracking& tracking);


    omnetpp::simtime_t mLaneChangeTime = 5.0;
    bool mLaneChangeInitiated = false;
    int mCurrentLane;
    int mTargetLane;

    std::map<int, ObjectHistory> mObjectHistories;

    void initiateLaneChange();
    void checkLaneChangeRisk();
    double calculateTimeToCollision(const artery::LocalEnvironmentModel::Tracking& tracking);
    void sendLaneChangeDENM(double risk);

    // Paramètres pour l'évaluation du risque
    double mMaxDetectionDistance = 100.0; // mètres
    double mMaxRelativeSpeed = 50.0; // m/s
    double mMinTTC = 1.0; // secondes
    double mMinAvailableSpace = 10.0; // mètres

    // Poids pour les facteurs de risque
    double mW1 = 0.2; // Distance
    double mW2 = 0.2; // Vitesse relative
    double mW3 = 0.6; // TTC
};

#endif /* LANECHANGEWARNINGSERVICE_H_ */