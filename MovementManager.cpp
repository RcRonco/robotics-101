#include "MovementManager.h"
#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <iostream>
#include <thread>

MovementManager::MovementManager(Robot *robot, MapDrawer *mapDrawer) {
    this->_robot = robot;
    this->_mapDrawer = mapDrawer;
}

void MovementManager::NavigateToWaypoint(Node* way_point) {
    this->_wayPoint = way_point;
    _robot->realLocation = _robot->prevBeliefedLocation = _robot->currBeliefedLocation = _robot->GetRealHamsterLocation();

    recalculateDistanceFromWaypoint();

    while(_distanceWaypoint > DISTANCE_FROM_WAYPOINT_TOLERANCE) {
        _robot->prevBeliefedLocation = _robot->currBeliefedLocation;
        _robot->realLocation = _robot->currBeliefedLocation = _robot->GetRealHamsterLocation();
        recalculateDeltaYaw();

        calculateTargetYaw(way_point);

        if(isRequiredAngleAdjustment()) {
            std::cout << "Turning, targetYaw: " << _targetYaw << " currYaw: " << _robot->currBeliefedLocation.yaw
                      << " deltaYaw: " << _deltaYaw << " w: (" << way_point->getX() << ", " << way_point->getY()
                      << ") r: (" << _robot->currBeliefedLocation.pos.x << ", " << _robot->currBeliefedLocation.pos.y << ")"
                      << endl;
            turnToWaypoint();
        } else {
            cout << "Forward, way_point: (" << way_point->getX() << ", " << way_point->getY() <<
                 ") robot: (" << _robot->currBeliefedLocation.pos.x << ", " << _robot->currBeliefedLocation.pos.y << ")"
                 << endl;

            moveToWaypoint();
        }

        recalculateDistanceFromWaypoint();
        _mapDrawer->Show(_robot->GetRealHamsterLocation());
        std::this_thread::sleep_for(1.5s);
    }

    cout << "The way_point: (" << way_point->getX() << ", " << way_point->getY() << ") has been reached" << endl;
    stopMoving();

    return;
}


void MovementManager::recalculateDeltaYaw() {
    // Is in circle
    if((_targetYaw > 360 - DELTA_ANGLE_TOL && _robot->currBeliefedLocation.yaw < DELTA_ANGLE_TOL) ||
       (_robot->currBeliefedLocation.yaw > 360 - DELTA_ANGLE_TOL && _targetYaw < DELTA_ANGLE_TOL)) {
        _deltaYaw = fabs((_targetYaw + _robot->currBeliefedLocation.yaw) - 360);
    } else {
        _deltaYaw = fabs(_targetYaw - _robot->currBeliefedLocation.yaw);
    }
}

bool MovementManager::isRequiredAngleAdjustment() {
    return !isDeltaAngleOnEndOfCiricle() && _deltaYaw > YAW_TOLERANCE;
}

// Forward movement more stable
bool MovementManager::isDeltaAngleOnEndOfCiricle() {
    return (_targetYaw > (360 - YAW_TOLERANCE) && _robot->currBeliefedLocation.yaw < YAW_TOLERANCE) ||
           (_robot->currBeliefedLocation.yaw > (360 - YAW_TOLERANCE) && _targetYaw < YAW_TOLERANCE);
}

void MovementManager::calculateTargetYaw(Node *waypoint) {
    _targetYaw = getYawInOneCiricle(convertRadiansToDegrees(atan2((waypoint->getY() - _robot->currBeliefedLocation.pos.y),
                                                                 (waypoint->getX() -
                                                                  _robot->currBeliefedLocation.pos.x))));
}

void MovementManager::turnToWaypoint() {
    _robot->setHamsterSpeed(calculateTurnSpeed(), calculateTurningDirection());
}

void MovementManager::moveToWaypoint() {
    _robot->setHamsterSpeed(calculateForwardSpeed(), 0.0);
}

void MovementManager::stopMoving() {
    _robot->setHamsterSpeed(0.0, 0.0);
}

double MovementManager::calculateTurningDirection() {
    if(_robot->currBeliefedLocation.yaw > _targetYaw) {
        if(360 - _robot->currBeliefedLocation.yaw + _targetYaw <
           _robot->currBeliefedLocation.yaw - _targetYaw) {
            return calculateWheelsAngle() * -1;
        } else {
            return calculateWheelsAngle();
        }
    } else {
        if(360 - _targetYaw + _robot->currBeliefedLocation.yaw <
           _targetYaw - _robot->currBeliefedLocation.yaw) {
            return calculateWheelsAngle();
        } else {
            return calculateWheelsAngle() * -1;
        }
    }
}

float MovementManager::calculateWheelsAngle() {
    if(_deltaYaw > MAX_WHEELS_ANGLE) {
        return MAX_WHEELS_ANGLE;
    }

    if(_deltaYaw < MIN_WHEELS_ANGLE) {
        return MIN_WHEELS_ANGLE;
    }

    return _deltaYaw;
}

void MovementManager::recalculateDistanceFromWaypoint() {
    _distanceWaypoint =
            sqrt(pow(_robot->currBeliefedLocation.pos.x - _wayPoint->getX(), 2) +
                 pow(_robot->currBeliefedLocation.pos.y - _wayPoint->getY(), 2));
}

double MovementManager::calculateTurnSpeed() {
    double yawRatio = _deltaYaw / _targetYaw;
    if(yawRatio > 1) {
        yawRatio = 1 / yawRatio;
    }

    return MIN_TURN_SPEED + ((MAX_TURN_SPEED - MIN_TURN_SPEED) * yawRatio);
}

double MovementManager::calculateForwardSpeed() {
    if(_distanceWaypoint > MIN_DISTANCE_FOR_FULL_SPEED) {
        return MAX_MOVE_SPEED;
    }

    return (double) _distanceWaypoint / FORWARD_SPEED_FACTOR;
}

MovementManager::~MovementManager() {
}
