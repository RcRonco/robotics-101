/*
 * MovementManager.h
 *
 *  Created on: Jul 2, 2017
 *      Author: user
 */

#ifndef MOVEMENTMANAGER_H_
#define MOVEMENTMANAGER_H_

#include "Robot.h"
#include <HamsterAPIClientCPP/Hamster.h>
#include "Utils/AngleUtils.h"
#include <vector>
#include <math.h>
#include "Constants.h"

using namespace HamsterAPI;

#define DISTANCE_FROM_WAYPOINT_TOLERANCE 5

class MovementManager {
    private:
        Robot* _robot;
        MapDrawer* _mapDrawer;
        Node* _wayPoint;
        double _distanceWaypoint;
        double _targetYaw, _deltaYaw;

        void turnToWaypoint();

        void moveToWaypoint();

        double GetAdjustedYaw(double yawToAdjust) const;

        double calculateTurningDirection();

        void recalculateDistanceFromWaypoint();

        double calculateTurnSpeed();

        double calculateForwardSpeed();

        bool isRequiredAngleAdjustment();

        bool isDeltaAngleOnEndOfCiricle();

        float calculateWheelsAngle();

        void calculateTargetYaw(Node *waypoint);

        void stopMoving();

        void recalculateDeltaYaw();

    public:
        MovementManager(Robot *robot, MapDrawer *mapDrawer);

        void NavigateToWaypoint(Node *way_point);

        virtual ~MovementManager();
};

#endif /* MOVEMENTMANAGER_H_ */
