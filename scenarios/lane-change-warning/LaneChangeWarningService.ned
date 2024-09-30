import artery.application.ItsG5Service;

simple LaneChangeWarningService like artery.application.ItsG5Service
{
    parameters:
        @class(LaneChangeWarningService);
        double laneChangeDelay @unit(s) = default(5s);
        double riskThreshold = default(0.3);
}