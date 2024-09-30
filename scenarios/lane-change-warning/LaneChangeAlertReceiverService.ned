import artery.application.ItsG5Service;

simple LaneChangeAlertReceiverService like artery.application.ItsG5Service
{
    parameters:
        @class(artery::LaneChangeAlertReceiverService);
        @signal[LaneChangeWarning](type=double);
        @statistic[laneChangeWarnings](title="Lane Change Warnings Received"; source="LaneChangeWarning"; record=count,vector);
}