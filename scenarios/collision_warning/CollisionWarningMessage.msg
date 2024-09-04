packet CollisionWarningMessage
{
    string senderId;
    double senderPosX;
    double senderPosY;
    double senderSpeed;
    double senderHeading;
    double timeToCollision;
    int dangerLevel;
}