#ifndef COLLISIONWARNINGSERVICE_H_
#define COLLISIONWARNINGSERVICE_H_

#include "artery/application/ItsG5Service.h"
#include "artery/envmod/LocalEnvironmentModel.h"
#include <vanetza/units/velocity.hpp>

namespace traci { class VehicleController; }

class CollisionWarningService : public artery::ItsG5Service
{
    public:
        void trigger() override;

    protected:
        void initialize() override;

    private:
        const artery::LocalEnvironmentModel* mLocalEnvironmentModel = nullptr;
        const traci::VehicleController* mVehicleController = nullptr;

        double mWarningThreshold = 3.0; // seconds
        double mCriticalThreshold = 1.5; // seconds

        void checkCollisionRisk();
        double calculateTimeToCollision(const artery::LocalEnvironmentModel::Tracking& object);
        void sendDENM(double ttc, int subCauseCode);
        void sendDENMtoRSU(double ttc, int subCauseCode);
        

};
#endif /* COLLISIONWARNINGSERVICE_H_ */