import artery.application.ItsG5Service;

simple DrowsinessDetectionService like ItsG5Service
{
    parameters:
        @class(DrowsinessDetectionService);
        double warningThreshold = default(0.6);
        double criticalThreshold = default(0.8);
}