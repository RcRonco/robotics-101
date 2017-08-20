/*
#include "ParameterSerialization.h"

using namespace std;

std::map<std::string, std::string> loadParameterFile(char* parameterFilePath)
{
	char delimiter = ':';
	string line;
	ifstream myfile (parameterFilePath);

	if (!myfile.is_open())
	{
		cerr << "Failed to open the parameter file " << parameterFilePath << endl;
		exit(EXIT_FAILURE);
	}
	else
	{
		map<string, string> parametersMap;

		while ( getline (myfile,line) )
		{
			vector<string> lineParts = splitString(line.c_str(), delimiter);

			string key = trimString(lineParts[0]);
			string value = trimString(lineParts[1]);

			parametersMap[key] = value;
		}

		myfile.close();

		return parametersMap;
	}
}

parameters deserializeParametersFile(char* parameterFilePath)
{
	map<string, string> parameterMap = loadParameterFile(parameterFilePath);

	parameters result;

	result.mapFilePath = parameterMap["map"].c_str();

	vector<string> startLocationParts = splitString(parameterMap["startLocation"].c_str(), ' ');
	struct positionState startPositionState;
	struct position startPosition;
	startPosition.x = atof(startLocationParts[0].c_str());
	startPosition.y = atof(startLocationParts[1].c_str());

	startPositionState.pos = startPosition;
	startPositionState.yaw = atof(startLocationParts[2].c_str());

	result.startLocation = startPositionState;

	vector<string> goalParts = splitString(parameterMap["goal"].c_str(), ' ');
	struct position goal;
	goal.x = atof(goalParts[0].c_str());
	goal.y = atof(goalParts[1].c_str());

	result.goal = goal;

	vector<string> sizeParts = splitString(parameterMap["robotSize"].c_str(), ' ');
	struct size robotSize;
	robotSize.width = atof(sizeParts[0].c_str());
	robotSize.length = atof(sizeParts[1].c_str());

	result.robotSize = robotSize;

	result.mapResolutionCM = atof(parameterMap["MapResolutionCM"].c_str());

	result.gridResolutionCM = atof(parameterMap["GridResolutionCM"].c_str());

	return result;
}
*/
