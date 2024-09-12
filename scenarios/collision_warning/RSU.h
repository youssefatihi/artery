#ifndef RSU_H_
#define RSU_H_

#include "artery/application/StationaryMiddleware.h"
#include "artery/application/ItsG5Service.h"
#include "artery/utility/Geometry.h"
#include "collision_msgs/DENMMessage_m.h"

namespace artery
{

class RSU : public StationaryMiddleware
{
public:
    void initialize(int stage) override;
    void finish() override;
    const Position& getPosition() const { return mPosition; }
    double getTransmissionRange() const { return mTransmissionRange; }
    std::string getStationId() const { return mRsuId; }

protected:
    Position mPosition;
    double mTransmissionRange;
    std::string mRsuId;

    class RelayService : public ItsG5Service
    {
    public:
        void initialize() override;
        void indicate(const vanetza::btp::DataIndication&, omnetpp::cPacket*) override;

    private:
        void relayDENM(const DENMMessage* denm);
        RSU* getRSU();
    };
};

} // namespace artery

#endif /* RSU_H_ */