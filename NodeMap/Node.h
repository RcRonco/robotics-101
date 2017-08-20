#ifndef NODE_H_
#define NODE_H_

#include <cstdlib>

class Node {
    bool _isObstacle;
    bool _isInOpenList;
    bool _isInClosedList;
    bool _isWaypoint;
    double _g;
    double _h;
    double _f;
    double _x;
    double _y;
    Node *_parent;


public:
    bool getIsObstacle() const { return _isObstacle; }
    void setIsObstacle(bool isObstacle) { _isObstacle = isObstacle; }

    bool getIsInOpenList() const { return _isInOpenList; }
    void setIsInOpenList(bool isInOpenList) { _isInOpenList = isInOpenList; }

    bool getIsInClosedList() const { return _isInClosedList; }
    void setIsInClosedList(bool isInClosedList) { _isInClosedList = isInClosedList; }

    bool getIsWaypoint() const { return _isWaypoint; }
    void setIsWaypoint(bool isWaypoint) { _isWaypoint = isWaypoint; }

    double getG() const { return _g; }
    void setG(double g) { _g = g; }

    double getH() const { return _h; }
    void setH(double h) { _h = h; }

    double getX() const { return _x; }
    void setX(double x) { _x = x; }

    double getY() const { return _y; }
    void setY(double y) { _y = y; }

    double getF() const { return _f; }
    void setF(double f) { _f = f; }

    Node *getParent() const { return _parent; }
    void setParent(Node* parent) { _parent = parent; }

    Node(double x = 0, double y = 0) : _parent(nullptr), _g(0), _h(0), _f(0), _x(x), _y(y),
                                       _isWaypoint(false), _isInClosedList(false),
                                       _isInOpenList(false), _isObstacle(false) {}

    virtual ~Node(){}

    bool operator<(const Node &node) const {
        if((_y < node._y) || (_y == node._y && _x <= node._x)) {
            return true;
        }

        return false;
    }
};

#endif  NODE_H_

