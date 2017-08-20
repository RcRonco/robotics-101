#include "MapDrawer.h"

MapDrawer::MapDrawer(int width, int height) : WINDOW_TITLE(WINDOW_NAME) {
	cv::namedWindow(WINDOW_NAME);
	_map = new cv::Mat(width, height,CV_8UC3,cv::Scalar(0,0,0));

}

void MapDrawer::DrawMap(OccupancyGrid* occupancyGridMap, double rotationAngle)
{
	int width = occupancyGridMap->getWidth();
	int height = occupancyGridMap->getHeight();

	for (int x = 0; x < height; x++)
	{
		 for (int y = 0; y < width; y++)
		 {
			MapDrawer::SetPointType(x,y, Free);
		 }
	}

	for (int x = 0; x < height; x++)
	{
	     for (int y = 0; y < width; y++)
	     {
				  if (occupancyGridMap->getCell(x, y) == CELL_FREE)
				  {
					  MapDrawer::SetPointType(x,y, Free);
				  }
				  else if (occupancyGridMap->getCell(x, y) == CELL_OCCUPIED)
				  {
					  MapDrawer::SetPointType(x,y, Obstacle);
				  }
				  else
				  {
					  MapDrawer::SetPointType(x,y, Unknown);
				  }

	     }
	}

	if(rotationAngle != 0)
	{
		rotateMapOnOrigin(_map, _map, rotationAngle);
	}
}

void MapDrawer::DrawNodeMap(NodeMap* nodeMap)
{
	int width = nodeMap->getWidth();
	int height = nodeMap->getHeight();

	for (int y = 0; y < height; y++) {
	     for (int x = 0; x < width; x++) {
	      if (nodeMap->getNodeAtIndex(y,x)->getIsObstacle())
	      {
	    	  MapDrawer::SetPointType(x, y, Obstacle);
	      }
	   }

	}
}

void MapDrawer::DrawRobot(positionState pos, cv::Mat * map)
{
	float robot_i, robot_j;
	robot_i = ROBOT_START_Y + pos.pos.y;
	robot_j = pos.pos.x + ROBOT_START_X;
	cv::Scalar_<int> * color = new cv::Scalar_<int>(255,0,0);
	cv::Point_<float>* position = new cv::Point_<float>(robot_j,robot_i);
	cv:circle(*map, *position,4,*color,1,8,0);
}

void MapDrawer::DrawPath(Node* goal)
{
	MapDrawer::SetPointType(goal->getY() , goal->getX() , PathEnd);

	Node* currentNode = goal->getParent();
	while(currentNode != NULL)
	{
		Node* nextNode = currentNode->getParent();

		if(nextNode == NULL) {
			MapDrawer::SetPointType(currentNode->getY() ,currentNode->getX()  , PathStart);
		}
		else if(currentNode->getIsWaypoint())
		{
			MapDrawer::SetPointType(currentNode->getY() ,currentNode->getX() , Waypoint);
		}
		else
		{
			MapDrawer::SetPointType(currentNode->getY() ,currentNode->getX() , Path);
		}

		currentNode = nextNode;
	}

	MapDrawer::SetPointType(goal->getY() ,goal->getX() , PathStart);

}

double MapDrawer::DrawPatricles(std::vector<Particle *>* particles)
{
	double bestParticalesAvrageBelief = 0;
	double particalesCounter = 0 ;

	for (unsigned i = 0; i != particles->size(); i++)
	{
		Particle* particale = (*particles)[i];
		if(i >= particles->size() - 30)
		{
			MapDrawer::SetPointType(particale->row ,particale->col, GoodParticle);
			bestParticalesAvrageBelief += particale->belief;
			particalesCounter++;
		}
		else
		{
			MapDrawer::SetPointType(particale->row ,particale->col, BadParticle);
		}
	}

	return bestParticalesAvrageBelief / particalesCounter;
}

void MapDrawer::SetPointType(int x, int y, MapPointType mapPointType)
{
	switch(mapPointType)
	{
		case(Unknown) :
		{
			MapDrawer::SetPointColor(x, y, 128, 128, 128);
			break;
		}
		case(Free) :
		{
			MapDrawer::SetPointColor(x, y, 255, 255, 255);
			break;
		}
		case(Obstacle) :
		{
			MapDrawer::SetPointColor(x, y, 0, 0, 0);
			break;
		}
		case(PathStart) : {
			MapDrawer::SetPointColor(x, y, 0, 0, 255);
			break;
		}
		case(PathEnd): {
			MapDrawer::SetPointColor(x, y, 0, 255, 0);
			break;
		}
		case(Path) :
		{
			MapDrawer::SetPointColor(x, y, 255, 0, 0);
			break;
		}
		case(Waypoint) :
		{
			MapDrawer::SetPointColor(x, y, 255, 255, 0);
			break;
		}
		case(Particle) :
		{
			MapDrawer::SetPointColor(x, y, 0, 0, 255);
			break;
		}
		case(LidarScanObstacle) :
		{
			MapDrawer::SetPointColor(x, y, 255, 0, 255);
			break;
		}
		case(GoodParticle) :
		{
			MapDrawer::SetPointColor(x, y, 0, 255, 0);
			break;
		}
		case(BadParticle) :
		{
			MapDrawer::SetPointColor(x, y, 255, 0, 0);
			break;
		}

	}
}

void MapDrawer::SetPointColor(int x, int y, int red, int green, int blue)
{
	if(x >= 0 && y >= 0)
	{
		MapDrawer::_map->at<cv::Vec3b>(x, y)[0] = blue;
		MapDrawer::_map->at<cv::Vec3b>(x, y)[1] = green;
		MapDrawer::_map->at<cv::Vec3b>(x, y)[2] = red;
	}
}

void MapDrawer::Show(positionState robotPos)
{
	cv::Point2f center(ROBOT_START_X,ROBOT_START_Y);
	cv::Mat resultMap;
	cv::Mat scaled;

	cv::Mat tempMap;
	scaled = cv::getRotationMatrix2D(center, 0, 1.0);
	cv::warpAffine(*_map, tempMap, scaled, _map->size());

	DrawRobot(robotPos,&tempMap);

	scaled = cv::getRotationMatrix2D(center, 0, 2.0);
	cv::warpAffine(tempMap, resultMap, scaled, _map->size());

	cv::imshow(MapDrawer::WINDOW_TITLE, resultMap);
	cv::waitKey(100);
}

cv::Mat* MapDrawer::getMap()
{
	return _map;
}

