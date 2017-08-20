#ifndef GUI_MAPDRAWER_H_
#define GUI_MAPDRAWER_H_

#include <HamsterAPIClientCPP/Hamster.h>
#include "MapPointType.cpp"
#include "../NodeMap/Node.h"
#include "../NodeMap/NodeMap.h"
#include "../Localization/Particle.h"
#include "../Utils/PositionUtils.h"
#include "../Utils/AngleUtils.h"
#include "../Robot.h"
#include "opencv2/imgproc.hpp"
using namespace HamsterAPI;
using namespace std;

class MapDrawer {
private:
	const string WINDOW_TITLE;
	cv::Mat* _map;
	void SetPointColor(int x, int y, int red, int green, int blue);
public:
	MapDrawer(int width, int height);

	void SetPointType(int x, int y, MapPointType mapPointType);
	void DrawMap(OccupancyGrid* occupancyGridMap, double rotationAngle);
	void DrawNodeMap(NodeMap* nodeMap);
	void DrawPath(Node* goal);
	void DrawRobot(positionState pos, cv::Mat * map);
	void Show(positionState robotPos);
	double DrawPatricles(std::vector<Particle *>* particles);
	cv::Mat* getMap();
	//void DrawLidarScan(std::vector<positionState> obstacles);
};

#endif /* GUI_MAPDRAWER_H_ */
