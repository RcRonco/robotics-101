
#ifndef LOCALIZATIONMANAGER_H_
#define LOCALIZATIONMANAGER_H_

#include "Particle.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include <HamsterAPIClientCPP/Hamster.h>
#include "../Constants.h"
#include "../Structs.h"

#define NUM_OF_PARTICALES 350
#define TRY_TO_BACK 20
#define TOP_PARTICALES 80
#define BAD_PARTICALES 80

using namespace HamsterAPI;

//this class manage all the particals in the map
class LocalizationManager {

private:
        Hamster* _hamster;
        std::vector<Particle *> _particlesVec;
        double _mapRes;


        //return back the particales which out of the free cells range to free cells range
        bool tryReturnBackOutOfRangeParticle(Particle* particle);

        //update the particale's belief
        double updateBelief(Particle* particle);

        //create a random particale
        void createRandomParticle(Particle *particle);

        //close the bad particale near to a particale with a high belief
        void createNeighborParticales(Particle *prevParticale, Particle *newParticale);

        void initSourceParticle(positionState *ps);

        double randNumberFactor(int32_t  level);

        double randNumberFactorYaw(int32_t  level);

        void calculateRealPos(Particle* particle, double deltaX,
                              double deltaY, double deltaYaw);

        void calculateYaw(Particle* particle, double deltaYaw);

        void calculatePositionOnMap(Particle* particle);

        void replaceBadOutOfRangeParticle(Particle *particle, size_t size);

    public:

        cv::Mat *map;

        //constructor
        LocalizationManager(cv::Mat *map, Hamster *hamster, double mapResolution);

        //getter
        std::vector<Particle *> *getParticles();

        //create new random particals on the map
        void InitParticalesOnMap(positionState* posState);

        //move the particales according to the robot's movement
        void moveParticales(double deltaX, double deltaY, double deltaYaw);

        positionState getPosition();

        double getBestBelief();

        virtual ~LocalizationManager();
};

#endif /* LOCALIZATIONMANAGER_H_ */
