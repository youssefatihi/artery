import artery.application.StationaryMiddleware;

module RSU extends StationaryMiddleware
{
    parameters:
        @class(RSU);
        double posX @unit(m);
        double posY @unit(m);
        double transmissionRange @unit(m);

    submodules:
        relayService: <default("RSU.RelayService")> like artery.application.ItsG5Service {
            @display("p=250,300");
        }
}