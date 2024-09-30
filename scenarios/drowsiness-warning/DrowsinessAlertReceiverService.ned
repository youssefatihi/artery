import artery.application.ItsG5Service;

simple DrowsinessAlertReceiverService like artery.application.ItsG5Service
{
    parameters:
        @class(artery::DrowsinessAlertReceiverService);
        @signal[DrowsinessWarning](type=long);
        @statistic[drowsinessWarnings](title="Drowsiness Warnings Received"; source="DrowsinessWarning"; record=count,vector);
}