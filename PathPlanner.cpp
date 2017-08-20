#include "PathPlanner.h"

void PathPlanner::findShortestPath(NodeMap *map, Node *startNode, Node *goalNode) {
    std::set<Node*> openList;
    std::set<Node*> closedList;
    Node* currNode;

    initHeuristicValues(map, goalNode);
    closedList.insert(startNode);
    startNode->setIsInClosedList(true);
    handleNeighbors(map, startNode, goalNode, openList, closedList);

    while(!openList.empty()) {
        currNode = getMinimalFNode(openList);

        openList.erase(currNode);
        currNode->setIsInOpenList(false);
        closedList.insert(currNode);
        currNode->setIsInClosedList(true);

        handleNeighbors(map, currNode, goalNode, openList, closedList);

    }
}

void PathPlanner::initHeuristicValues(NodeMap *map, Node *goalNode) {
    for(int rowIndex = 0; rowIndex < map->getHeight(); rowIndex++) {
        for(int colIndex = 0; colIndex < map->getWidth(); colIndex++) {
            Node *currNode = map->getNodeAtIndex(colIndex, rowIndex);

            if(!currNode->getIsObstacle()) {
                currNode->setH(calculateDistance(currNode, goalNode));
            }
        }
    }
}

double PathPlanner::calculateDistance(Node *source, Node *target) {
    return sqrt(pow(source->getX() - target->getX(), 2) + pow(source->getY() - target->getY(), 2));
}

void PathPlanner::handleNeighbors(NodeMap *map, Node *currNode, Node *goalNode,
                                  std::set<Node*>& openList, std::set<Node*>& closedList) {
    for(int rowIndex = currNode->getY() - 1; rowIndex <= currNode->getY() + 1; rowIndex++) {
        for(int colIndex = currNode->getX() - 1; colIndex <= currNode->getX() + 1; colIndex++) {
            // Checks if we're out of bounds and if the current neighbor is not an obstacle
            if(colIndex >= 0 && rowIndex >= 0 &&
               colIndex < map->getWidth() && rowIndex < map->getHeight() &&
               !map->getNodeAtIndex(colIndex, rowIndex)->getIsObstacle()) {
                // Makes sure the current node is not scanned
                if(colIndex != currNode->getX() || rowIndex != currNode->getY()) {
                    // Checks if the current neighbor is in the closed list
                    Node *currNeighbor = map->getNodeAtIndex(colIndex, rowIndex);

                    if(!currNeighbor->getIsInClosedList()) {
                        double tempGCost =
                                calculateDistance(currNode, currNeighbor) + currNode->getG();

                        // Checks if the current neighbor is already in the open list
                        if(!currNeighbor->getIsInOpenList()) {
                            currNeighbor->setG(tempGCost);
                            currNeighbor->setF(currNeighbor->getH() + tempGCost);
                            currNeighbor->setParent(currNode);

                            // Checking if we have reached the goal
                            if(goalNode->getX() == colIndex && goalNode->getY() == rowIndex) {
                                openList.clear();
                                return;
                            }

                            // Adding the node to the open list for the first time
                            openList.insert(currNeighbor);
                            currNeighbor->setIsInOpenList(true);
                        }
                            // The node was already in the open list, check if we found a shorter path
                        else {
                            if(tempGCost < currNeighbor->getG()) {
                                currNeighbor->setG(tempGCost);
                                currNeighbor->setF(currNeighbor->getH() + tempGCost);
                                currNeighbor->setParent(currNode);
                            }
                        }
                    }
                }
            }
        }
    }
}

Node* PathPlanner::getMinimalFNode(const std::set<Node *> &openList) {
    Node* minNode = *(openList.begin());
    Node* currNode;

    for(set<Node *>::iterator iter = openList.begin(); iter != openList.end(); iter++) {
        currNode = *iter;

        if(currNode->getF() < minNode->getF()) {
            minNode = currNode;
        }
    }

    return minNode;
}

std::list<Node*> PathPlanner::markWaypoints(Node *start, Node *currNode) {
    std::list<Node*> way_points;
    Node *firstNode, *secondNode, *thirdNode;
    firstNode = currNode;
    secondNode = currNode->getParent();
    int skipCounter = 0;

    if(secondNode == NULL) {
        return way_points;
    }

    while(firstNode->getX() != start->getX() || firstNode->getY() != start->getY()) {

        thirdNode = secondNode->getParent();

        if(thirdNode == NULL) {
            way_points.push_back(secondNode);
            return way_points;
        }

        if((getIncline(firstNode, secondNode) != getIncline(secondNode, thirdNode)) ||
           skipCounter >= WAYPOINT_TOLERENCE) {
            secondNode->setIsWaypoint(true);
            way_points.push_back(secondNode);
            skipCounter = 0;
        } else {
            skipCounter++;
        }


        firstNode = secondNode;
        secondNode = thirdNode;
    }

    return way_points;
}

double PathPlanner::getIncline(Node *a, Node *b) {
    if(b->getX() - a->getX() == 0)
        return 0;

    return (b->getY() - a->getY()) / (b->getX() - a->getX());
}



