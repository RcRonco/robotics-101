#include <HamsterAPIClientCPP/Hamster.h>
#include <iostream>
#include "NodeMap/NodeMap.h"
#include "PathPlanner.h"
#include <stdlib.h>
#include "Gui/MapDrawer.h"
#include "Localization/LocalizationManager.h"
#include "Structs.h"
#include "Robot.h"
#include "MovementManager.h"
#include "Utils/PositionUtils.h"
#include "math.h"
#include <chrono>

using namespace std;
using namespace HamsterAPI;

HamsterAPI::Hamster * hamster;

void startRobotAction();

int main() {
	try {
		startRobotAction();
	} catch (const HamsterAPI::HamsterError & connection_error) {
		HamsterAPI::Log::i("Client", connection_error.what());
	}
	return 0;
}

bool isWaypointReached(const positionState& currLocation,
		const Node& hamsterWaypoint) {
	double distanceFromWaypoint = sqrt(
			pow(currLocation.pos.x - hamsterWaypoint.getX(), 2)
					+ pow(currLocation.pos.y - hamsterWaypoint.getY(), 2));

	return distanceFromWaypoint <= DISTANCE_FROM_WAYPOINT_TOLERANCE;
}

void updateParticalesOnRobot(OccupancyGrid roomRealMapFromMemory, NodeMap roomBlownMap,
								 LocalizationManager* localizationManager, MapDrawer* mapDrawer,
								 Node* goalPos, double deltaX = 0, double deltaY = 0, double deltaYaw = 0) {
	cout << "Initialize particales on robot" << endl;
	double bestParticalesAvrageBelief = 0;
	auto start_time = std::chrono::high_resolution_clock::now();
	auto end_time = std::chrono::high_resolution_clock::now();
	double seconds = 0;
	while (bestParticalesAvrageBelief < TARGET_START_BELIEF && seconds < 10) {
		localizationManager->moveParticales(deltaX, deltaY, deltaYaw);
		mapDrawer->DrawMap(&roomRealMapFromMemory, MAP_ROTATION);
		mapDrawer->DrawNodeMap(&roomBlownMap);
		mapDrawer->DrawPath(goalPos);
		bestParticalesAvrageBelief = mapDrawer->DrawPatricles(
				localizationManager->getParticles());
		positionState a;
		mapDrawer->Show(a);
		cout << "Target belief is: " << TARGET_START_BELIEF
				<< " current average belief: " << bestParticalesAvrageBelief
				<< endl;
		end_time = std::chrono::high_resolution_clock::now();
		seconds = std::chrono::duration<double, milli>(end_time - start_time).count() / 1000;
	}
}

void startRobotAction() {

 cv::namedWindow(WINDOW_NAME);
 hamster = new HamsterAPI::Hamster(1);
 sleep(3);

 NodeMap roomRealMap;
 NodeMap roomBlownMap;
 PathPlanner *pathPlanner;
 OccupancyGrid roomRealMapFromMemory = hamster->getSLAMMap();
 MapDrawer mapDrawer(roomRealMapFromMemory.getWidth(), roomRealMapFromMemory.getHeight());


 mapDrawer.DrawMap(&roomRealMapFromMemory, 0);
 cv::Mat drawedMap(roomRealMapFromMemory.getWidth(), roomRealMapFromMemory.getHeight(),CV_8UC3,cv::Scalar(0,0,0));
 rotateMapOnOrigin(mapDrawer.getMap(), &drawedMap, MAP_ROTATION);

 // Init nodemaps
 roomRealMap.loadMap(&drawedMap);
 roomBlownMap.loadBlowMap(&drawedMap);

 // Init the robot start and goal positions
 Node *startPos = roomBlownMap.getNodeAtIndex(ROBOT_START_X, ROBOT_START_Y);
 Node *goalPos = roomBlownMap.getNodeAtIndex(GOAL_X, GOAL_Y);

 cout << "Is goal an obstacle: " << roomBlownMap.getNodeAtIndex(goalPos->getX(), goalPos->getY())->getIsObstacle() << endl;

 // Find the path
 pathPlanner->findShortestPath(&roomBlownMap,startPos,goalPos);

 // Mark the waypoints
 std::list<Node*> waypoints = pathPlanner->markWaypoints(startPos, goalPos);

 // First draw
 mapDrawer.DrawMap(&roomRealMapFromMemory, MAP_ROTATION);
 mapDrawer.DrawNodeMap(&roomBlownMap);
 mapDrawer.DrawPath(goalPos);


 Pose robotStartPose = hamster->getPose();
 position startPosition = {.x = ROBOT_START_X + robotStartPose.getX(),
 						   .y = ROBOT_START_Y - robotStartPose.getX()};

 positionState startPositionState = {.pos = startPosition,
		 	 	 	 	 	 	 	 .yaw = robotStartPose.getHeading()};


 double mapResolution = roomRealMapFromMemory.getResolution();

 LocalizationManager localizationManager(&drawedMap, hamster, mapResolution);
 Robot robot(hamster,&localizationManager, roomRealMapFromMemory.getHeight(), roomRealMapFromMemory.getWidth(), mapResolution);
 localizationManager.InitParticalesOnMap(&startPositionState);

 updateParticalesOnRobot(roomRealMapFromMemory, roomBlownMap, &localizationManager, &mapDrawer, goalPos);

 //mapDrawer->DrawRobot(robot.GetRealHamsterLocation());
 MovementManager movementManager(&robot, &mapDrawer);

 if (hamster->isConnected()) {
	// foreach _wayPoint
	for (std::list<Node*>::reverse_iterator iter = waypoints.rbegin(); iter != waypoints.rend(); ++iter) {
		Node* currWaypoint = *iter;

		Node hamsterWaypoint = ConvertToHamsterLocation(currWaypoint);
		//robot.updateHamsterRealLocation();
		robot.realLocation = robot.GetRealHamsterLocation();
								//robot.currBeliefedLocation.pos.x - robot.prevBeliefedLocation.pos.x,
								//robot.currBeliefedLocation.pos.y - robot.prevBeliefedLocation.pos.y,
								//robot.currBeliefedLocation.yaw   - robot.prevBeliefedLocation.yaw);

		if (isWaypointReached(robot.currBeliefedLocation, hamsterWaypoint)) {
			cout << endl << "Reached _wayPoint (" << hamsterWaypoint.getX() << ", " << hamsterWaypoint.getY() << ")" << endl << endl;
		}
		else {
			movementManager.NavigateToWaypoint(&hamsterWaypoint);
		}
		position current_pos = {.x = ROBOT_START_X + robot.realLocation.pos.x,
		 						.y = ROBOT_START_Y + robot.realLocation.pos.y};
		positionState pos_state = {.pos = current_pos,
								   .yaw = robot.realLocation.yaw };
		localizationManager.InitParticalesOnMap(&pos_state);
		updateParticalesOnRobot(roomRealMapFromMemory, roomBlownMap, &localizationManager, &mapDrawer, goalPos);
	}

 	 cout << "The Robot reached the _wayPoint: (" << GOAL_X << ", " << GOAL_Y << ") and our grade is 100" << endl;
 }
}



