import artery.application.ItsG5Service;

simple CollisionAlertReceiverService like artery.application.ItsG5Service
{
    parameters:
        @class(artery::CollisionAlertReceiverService);
        @signal[CollisionWarning](type=long);
        @statistic[collisionWarnings](title="Collision Warnings Received"; source="CollisionWarning"; record=count,vector);
}