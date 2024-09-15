
simple SpeedAlertReceiverService like artery.application.ItsG5Service
{
    parameters:
        @class(artery::SpeedAlertReceiverService);
        @signal[SpeedWarning](type=long);
        @statistic[speedWarnings](title="Speed Warnings Received"; source="SpeedWarning"; record=count,vector);
}