packet DENMMessage
{
    // Header
    int protocolVersion;
    int messageID;
    string stationID;
    
    // Management Container
    simtime_t detectionTime;
    simtime_t referenceTime;
    int termination;
    
    // Situation Container  
    int informationQuality;
    int causeCode;
    int subCauseCode;
    
    // Location Container
    double eventPosition_latitude;
    double eventPosition_longitude;
    double eventSpeed;
    double eventHeading;
    
    // Alacarte Container (optional)
    double timeToCollision;
    double drowsinessLevel;
    int alertType;  // 0: Pas d'alerte, 1: Avertissement, 2: Critique
    double reactionTime;  // Temps de réaction estimé du conducteur
}