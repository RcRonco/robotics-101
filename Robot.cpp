#include "Robot.h"

Robot::Robot(HamsterAPI::Hamster *hamster, LocalizationManager *localizationManager, double resolution) {
    this->_hamster = hamster;
    this->_localizationMgr = localizationManager;
    this->_res = resolution;
}

positionState Robot::GetRealHamsterLocation() {
    HamsterAPI::Pose currPose = _hamster->getPose();
    double currX = (currPose.getX()) / _res;
    double currY = ((currPose.getY() * -1)) / _res;
    double currYaw = getYawInOneCiricle(currPose.getHeading());

    position currentPos = {.x = currX, .y = currY};
    positionState currPosState = {.pos = currentPos, .yaw = 360 - currYaw};

    return currPosState;
}

void Robot::updateHamsterRealLocation() {
    realLocation = GetRealHamsterLocation();

    prevBeliefedLocation = currBeliefedLocation;
    currBeliefedLocation = _localizationMgr->getPosition();

    _localizationMgr->moveParticales(currBeliefedLocation.pos.x - prevBeliefedLocation.pos.x,
                                     currBeliefedLocation.pos.y - prevBeliefedLocation.pos.y,
                                     currBeliefedLocation.yaw - prevBeliefedLocation.yaw);
}

void Robot::setHamsterSpeed(double speed, double wheelsAngle) {
    this->_hamster->sendSpeed(speed, wheelsAngle);
}

