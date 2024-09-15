
simple SpeedWarningService like artery.application.ItsG5Service
{
    parameters:
        @class(SpeedWarningService);
        double speedLimitWarningThreshold @unit(kmph) = default(10kmph);
        double speedLimitCriticalThreshold @unit(kmph) = default(20kmph);
}