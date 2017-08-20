#ifndef ROBOT_H_
#define ROBOT_H_

#include <HamsterAPIClientCPP/Hamster.h>
#include <set>
#include <math.h>
#include "NodeMap/Node.h"
#include "NodeMap/NodeMap.h"
#include "Structs.h"
#include "Gui/MapDrawer.h"
#include "Constants.h"
#include "Utils/AngleUtils.h"
#include "Localization/LocalizationManager.h"

class Robot {
    private:
        HamsterAPI::Hamster* _hamster;
        LocalizationManager* _localizationMgr;
        double _res;

    public:
        positionState prevBeliefedLocation, currBeliefedLocation, realLocation;

        Robot(HamsterAPI::Hamster *hamster, LocalizationManager *localizationManager, double resolution);

        positionState GetRealHamsterLocation();

        void updateHamsterRealLocation();

        void setHamsterSpeed(double speed, double wheelsAngle);

        virtual ~Robot() {}
};

#endif /* ROBOT_H_ */
