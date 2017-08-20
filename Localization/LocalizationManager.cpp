
#include "LocalizationManager.h"
#include <iostream>
#include <algorithm>
#include <random>
#include <cstdint>

using namespace std;

template<typename T>
T randNumber(T start_val = 0, T end_val = std::numeric_limits<T>::max()) {
    if(!std::is_integral<T>())
        return 0;

    std::random_device dev;
    std::mt19937 gen;
    gen.seed(dev());

    if(std::is_floating_point<T>()) {
        std::uniform_real_distribution<> dst(start_val, end_val);
        return dst(gen);
    } else {
        std::uniform_int_distribution<> dst(start_val, end_val);
        return dst(gen);
    }
}

LocalizationManager::LocalizationManager(cv::Mat *map, Hamster *hamster, double mapResolution)
        : _hamster(hamster), map(map), _mapRes(mapResolution) {}

void LocalizationManager::createRandomParticle(Particle *particle) {
    // Random an angle
    particle->yaw = randNumber<int>(0, 360);
    cv::Vec3b pnt(255, 255, 255);

    //set random column and row while the random cell chosen isn't free
    while(!(pnt[0] == 255 && pnt[1] == 255 && pnt[2] == 255)) {
        particle->col = randNumber<int>(0, map->cols);
        particle->row = randNumber<int>(0, map->rows);
        pnt = map->at<cv::Vec3b>(particle->row, particle->col);
    }

    //Convting the point into particle
    particle->x = (particle->col - ROBOT_START_X) * _mapRes;
    particle->y = (ROBOT_START_Y - particle->row) * _mapRes;


}

double LocalizationManager::randNumberFactor(int level) {
    switch (level) {
        case 3:
            return 0.4 - 0.8 * (double) randNumber<int>() / (double) RAND_MAX;
        case 2:
            return 0.2 - 0.4 * (double) randNumber<int>() / (double) RAND_MAX;
        default:
            return 0.1 - 0.2 * (double) randNumber<int>() / (double) RAND_MAX;
    }
}

double LocalizationManager::randNumberFactorYaw(int level) {
    switch (level) {
        case 2:
            return 10 - randNumber<int>(0, 20);
        case 3:
            return 30 - randNumber<int>(0, 30);
        case 4:
            return 90 - randNumber<int>(0, 80);
        case 5:
            return 180 - randNumber<int>(0, 360);
        default:
            return 5 - randNumber<int>(0, 10);
    }
}

void LocalizationManager::createNeighborParticales(Particle* prevParticale, Particle* newParticale) {
    cv::Vec3b pnt(255, 255, 255);

    while(!(pnt[0] == 255 && pnt[1] == 255 && pnt[2] == 255)) {
        if(newParticale->belief < 0.3) {
            prevParticale->x = newParticale->x + randNumberFactor(3);
            prevParticale->y = newParticale->y + randNumberFactor(3);
        } else if(newParticale->belief < 0.6) {
            prevParticale->x = newParticale->x + randNumberFactor(2);
            prevParticale->y = newParticale->y + randNumberFactor(2);
        } else {
            prevParticale->x = newParticale->x + randNumberFactor(1);
            prevParticale->y = newParticale->y + randNumberFactor(1);
        }

        prevParticale->row = (double) ROBOT_START_Y - prevParticale->y / _mapRes;
        prevParticale->col = prevParticale->x / _mapRes + ROBOT_START_X;

        pnt = map->at<cv::Vec3b>(prevParticale->row, prevParticale->col);

    }

    if(newParticale->belief < 0.2)
        prevParticale->yaw = (newParticale->yaw + (randNumberFactorYaw(5)));
    else if(newParticale->belief < 0.4)
        prevParticale->yaw = (newParticale->yaw + (randNumberFactorYaw(4)));
    else if(newParticale->belief < 0.6)
        prevParticale->yaw = (newParticale->yaw + (randNumberFactorYaw(3)));
    else if(newParticale->belief < 0.8)
        prevParticale->yaw = (newParticale->yaw + (randNumberFactorYaw(2)));
    else
        prevParticale->yaw = (newParticale->yaw + (randNumberFactorYaw(1)));

    if(prevParticale->yaw >= 360)
        prevParticale->yaw -= 360;
    if(prevParticale->yaw < 0)
        prevParticale->yaw += 360;
}

void LocalizationManager::InitParticalesOnMap(positionState *posState) {
    _particlesVec.resize(NUM_OF_PARTICALES);
    cv::Vec3b coloredPoint;

    initSourceParticle(posState);

    for(size_t i = 0; i < _particlesVec.size() - 1; i++) {
        _particlesVec[i] = new Particle();

        //Randomizing an angle
        double degYaw = posState->yaw * 180 / M_PI + 180;
        if(degYaw >= 360)
            degYaw -= 360;
        if(degYaw < 0)
            degYaw += 360;

        _particlesVec[i]->yaw = randNumber<int>(0, 360);

        //set random column and row while the random cell chosen isn't free
        do {
            _particlesVec[i]->col = posState->pos.x + randNumber<int>(0, 5);
            _particlesVec[i]->row = posState->pos.y + randNumber<int>(0, 5);

            coloredPoint = map->at<cv::Vec3b>(_particlesVec[i]->row, _particlesVec[i]->col);

        } while(!(coloredPoint[0] == 255 && coloredPoint[1] == 255 && coloredPoint[2] == 255));

        //Conversion
        _particlesVec[i]->x = (_particlesVec[i]->col - ROBOT_START_X) * _mapRes;
        _particlesVec[i]->y = (ROBOT_START_Y - _particlesVec[i]->row) * _mapRes;

    }


}

void LocalizationManager::initSourceParticle(positionState *posState) {
    _particlesVec[_particlesVec.size() - 1] = new Particle();
    _particlesVec[_particlesVec.size() - 1]->col = posState->pos.x;
    _particlesVec[_particlesVec.size() - 1]->row = posState->pos.y;
    _particlesVec[_particlesVec.size() - 1]->yaw = posState->yaw;
    _particlesVec[_particlesVec.size() - 1]->belief = 1;
}

double LocalizationManager::updateBelief(Particle *particle) {
    cv::Vec3b pnt;
    LidarScan lidarScan = _hamster->getLidarScan();

   int32_t hits = 0;
   int32_t misses = 0;

    for(int i = 0; i < lidarScan.getScanSize(); i++) {
        double angle = lidarScan.getScanAngleIncrement() * i * DEG2RAD;

        if(lidarScan.getDistance(i) < lidarScan.getMaxRange() - 0.001) {
            double x_wall =
                    particle->x + lidarScan.getDistance(i) * cos(angle + particle->yaw * DEG2RAD - 180 * DEG2RAD);
            double y_wall =
                    particle->y + lidarScan.getDistance(i) * sin(angle + particle->yaw * DEG2RAD - 180 * DEG2RAD);

           int32_t row = int32_t(ROBOT_START_Y - (y_wall / _mapRes));
           int32_t col = int32_t(ROBOT_START_X + (x_wall / _mapRes));

            pnt = map->at<cv::Vec3b>(row, col);

            // Maybe change to cell occupied
            if(!(pnt[0] == 255 && pnt[1] == 255 && pnt[2] == 255)) {
                hits++;
            } else {
                misses++;
            }
        }
    }

    return (double) hits / (hits + misses);
}

bool LocalizationManager::tryReturnBackOutOfRangeParticle(Particle *particle) {
    cv::Vec3b pnt(255, 255, 255);
    Particle newParticle(*particle);
    int32_t distant;
    int32_t count = 0;
    while(!(pnt[0] == 255 && pnt[1] == 255 && pnt[2] == 255) && count < TRY_TO_BACK) {
        //+-7 for distant
        distant = 14 - randNumber<int32_t>(0, 28);
        particle->col = newParticle.col + distant;
        distant = 14 - randNumber<int32_t>(0, 28);
        particle->row = newParticle.row + distant;
        pnt = map->at<cv::Vec3b>(newParticle.row, newParticle.col);

        count++;
    }

    //Conversion
    particle->x = (particle->col - ROBOT_START_X) * _mapRes;
    particle->y = (ROBOT_START_Y - particle->row) * _mapRes;

    return count < TRY_TO_BACK;
}

void LocalizationManager::calculateYaw(Particle *particle, double deltaYaw) {
    particle->yaw += deltaYaw;
    if(particle->yaw >= 360) {
        particle->yaw -= 360;
    }
    if(particle->yaw < 0) {
        particle->yaw += 360;
    }
}

void LocalizationManager::calculateRealPos(Particle* particle, double deltaX,
                                           double deltaY, double deltaYaw) {
    double distance = sqrt(deltaX * deltaX + deltaY * deltaY);
    particle->x += distance * cos(particle->yaw * DEG2RAD);
    particle->y += distance * sin(particle->yaw * DEG2RAD);

    calculateYaw(particle, deltaYaw);
}

void LocalizationManager::calculatePositionOnMap(Particle *particle) {
    particle->row = int32_t(ROBOT_START_Y - (particle->y / _mapRes));
    particle->col = int32_t(ROBOT_START_X + (particle->x / _mapRes));
}

void LocalizationManager::replaceBadOutOfRangeParticle(Particle *particle, size_t size) {
    size_t indexFromTop = size - randNumber<int32_t>(0, TOP_PARTICALES - 1);

    if(_particlesVec[indexFromTop]->belief > 0.4) {
        createNeighborParticales(particle, _particlesVec[indexFromTop]);
    } else {
        createRandomParticle(particle);
    }
}

void LocalizationManager::moveParticales(double deltaX, double deltaY, double deltaYaw) {
    cv::Vec3b pnt;

    for (Particle* particle : _particlesVec) {
        pnt = map->at<cv::Vec3b>(particle->row, particle->col);
        if(!(pnt[0] == 255 && pnt[1] == 255 && pnt[2] == 255) &&
           (particle->belief <= MIN_BELIEF || tryReturnBackOutOfRangeParticle(particle))) {
            replaceBadOutOfRangeParticle(particle, _particlesVec.size());
        }

        particle->belief = updateBelief(particle);
    }

    std::sort(_particlesVec.begin(), _particlesVec.end(), [](Particle* a, Particle* b)->bool {
        return a->belief < b->belief;
    });

    for(int i = 1; i <= BAD_PARTICALES; i++) {
        if(MIN_BELIEF < _particlesVec[_particlesVec.size() - i]->belief) {
            createNeighborParticales(_particlesVec[i - 1], _particlesVec[_particlesVec.size() - i]);
            updateBelief(_particlesVec[i - 1]);
        } else {
            createRandomParticle(_particlesVec[i - 1]);
            updateBelief(_particlesVec[i - 1]);
        }
    }

}

vector<Particle*>* LocalizationManager::getParticles() {
    return &_particlesVec;
}

positionState LocalizationManager::getPosition() {
    Particle *localizationParticle = _particlesVec[_particlesVec.size() - 1];

    positionState positionState;
    positionState.pos.x = localizationParticle->col;
    positionState.pos.y = localizationParticle->row;
    positionState.yaw = localizationParticle->yaw;

    return positionState;
}

double LocalizationManager::getBestBelief() {
    return _particlesVec[_particlesVec.size() - 1]->belief;
}

LocalizationManager::~LocalizationManager() {
}

