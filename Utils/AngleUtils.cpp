#include <HamsterAPIClientCPP/Hamster.h>

#include "AngleUtils.h"
#include "../Constants.h"

double convertDegreesToRadians(double angleDegrees)
{
	double angleRadians = angleDegrees * M_PI / 180;

	return angleRadians;
}

double convertRadiansToDegrees(double angleRadians)
{
	double angleDegrees = angleRadians * 180 / M_PI;

	return angleDegrees;
}

double getYawInOneCiricle(double yaw)
{
	while (yaw < 0)
	{
		yaw += 360;
	}

	while (yaw > 360)
	{
		yaw =- 360;
	}

	return yaw;
}

void rotateMapOnOrigin(cv::Mat* source, cv::Mat* dest, double rotationAngle)
{
	cv::Point2f center(ROBOT_START_X,ROBOT_START_Y);
	cv::Mat rotated = cv::getRotationMatrix2D(center, rotationAngle, 1.0);
	cv::warpAffine(*source, *dest, rotated, dest->size());
}
