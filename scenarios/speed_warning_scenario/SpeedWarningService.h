#ifndef SPEEDWARNINGSERVICE_H_
#define SPEEDWARNINGSERVICE_H_

#include "artery/application/ItsG5Service.h"
#include "artery/envmod/LocalEnvironmentModel.h"
#include <vanetza/units/velocity.hpp>

namespace traci { class VehicleController; }

class SpeedWarningService : public artery::ItsG5Service
{
    public:
        void trigger() override;

    protected:
        void initialize() override;

    private:
        const traci::VehicleController* mVehicleController = nullptr;

        double mSpeedLimitWarningThreshold = 10.0; // km/h over speed limit
        double mSpeedLimitCriticalThreshold = 20.0; // km/h over speed limit

        void checkSpeedLimit();
        void sendDENM(double speedExcess, int subCauseCode);
};
#endif /* SPEEDWARNINGSERVICE_H_ */