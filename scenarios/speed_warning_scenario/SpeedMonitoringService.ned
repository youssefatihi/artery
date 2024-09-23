import artery.application.ItsG5Service;

simple SpeedMonitoringService like artery.application.ItsG5Service
{
    parameters:
        @class(SpeedMonitoringService);

        @signal[CamReceived](type=CaObject);
        @signal[CamSent](type=CaObject);

        double speedLimit @unit(mps) = default(33.33mps);  // 120 km/h
        double speedLimitWarningThreshold @unit(mps) = default(2.78mps);  // 10 km/h
        double speedLimitCriticalThreshold @unit(mps) = default(5.56mps);  // 20 km/h
}