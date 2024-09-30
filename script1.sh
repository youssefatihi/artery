
cp scenarios/collision-warning/omnetpp.ini scenarios/lane-change-warning/
cp scenarios/collision-warning/services.xml scenarios/lane-change-warning/
cp scenarios/collision-warning/services-rsu.xml scenarios/lane-change-warning/
cp scenarios/collision-warning/sensors.xml scenarios/lane-change-warning/
cp scenarios/collision-warning/DENMMessage.msg scenarios/lane-change-warning/
cp scenarios/collision-warning/highway.net.xml scenarios/lane-change-warning/
cp scenarios/collision-warning/highway.rou.xml scenarios/lane-change-warning/
cp scenarios/collision-warning/highway.sumocfg scenarios/lane-change-warning/
cp scenarios/collision-warning/rsu.add.xml scenarios/lane-change-warning/
cp scenarios/collision-warning/additional.add.xml scenarios/lane-change-warning/
cp scenarios/collision-warning/CMakeLists.txt scenarios/lane-change-warning/
mv scenarios/lane-change-warning/CollisionWarningService.cc scenarios/lane-change-warning/LaneChangeWarningService.cc
mv scenarios/lane-change-warning/CollisionWarningService.h scenarios/lane-change-warning/LaneChangeWarningService.h
mv scenarios/lane-change-warning/CollisionWarningService.ned scenarios/lane-change-warning/LaneChangeWarningService.ned
mv scenarios/lane-change-warning/CollisionAlertReceiverService.cc scenarios/lane-change-warning/LaneChangeAlertReceiverService.cc
mv scenarios/lane-change-warning/CollisionAlertReceiverService.h scenarios/lane-change-warning/LaneChangeAlertReceiverService.h
mv scenarios/lane-change-warning/CollisionAlertReceiverService.ned scenarios/lane-change-warning/LaneChangeAlertReceiverService.ned