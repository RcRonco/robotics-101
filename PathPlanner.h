#ifndef PATHPLANNER_H_
#define PATHPLANNER_H_

#include <set>
#include <math.h>
#include <list>
#include "Structs.h"
#include "NodeMap/NodeMap.h"
#include "Constants.h"

class PathPlanner {
    public:
        double calculateDistance(Node *source, Node *target);

        void initHeuristicValues(NodeMap *map, Node *goalNode);

        void handleNeighbors(NodeMap *map, Node *currNode, Node *goalNode,
                             set<Node*>& openList, set<Node*>& closedList);

        void findShortestPath(NodeMap *map, Node *startNode, Node *goalNode);

        std::list<Node*> markWaypoints(Node *start, Node *currNode);

    private:
        Node* getMinimalFNode(const std::set<Node *>& openList);

        double getIncline(Node *a, Node *b);
};

#endif  PATHPLANNER_H_

