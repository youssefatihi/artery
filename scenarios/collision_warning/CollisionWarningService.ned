import artery.application.ItsG5Service;
simple CollisionWarningService like  artery.application.ItsG5Service
{
    parameters:
        @class(CollisionWarningService);
        double warningThreshold @unit(s) = default(3s);
        double criticalThreshold @unit(s) = default(1.5s);
}