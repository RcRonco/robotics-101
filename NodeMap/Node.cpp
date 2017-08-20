#include "Node.h"
#include "stdlib.h"

Node::Node()
{
	_parent = NULL;
	setG(0);
	setH(0);
	setIsInClosedList(false);
	setIsInOpenList(false);
	setIsWaypoint(false);
}

Node::Node(double x, double y) : Node()
{
	_x = x;
	_y = y;
}

Node::~Node()
{
}

bool Node::getIsObstacle() const
{
	return _isObstacle;
}

void Node::setIsObstacle(bool isObstacle)
{
	_isObstacle = isObstacle;
}

bool Node::getIsInOpenList() const
{
	return _isInOpenList;
}

void Node::setIsInOpenList(bool isInOpenList)
{
	_isInOpenList = isInOpenList;
}

bool Node::getIsInClosedList() const
{
	return _isInClosedList;
}

void Node::setIsInClosedList(bool isInClosedList)
{
	_isInClosedList = isInClosedList;
}

bool Node::getIsWaypoint() const
{
	return _isWaypoint;
}

void Node::setIsWaypoint(bool isWaypoint)
{
	_isWaypoint = isWaypoint;
}

double Node::getG() const
{
	return _g;
}

void Node::setG(double g)
{
	_g = g;
}

double Node::getH() const
{
	return _h;
}

void Node::setH(double h)
{
	_h = h;
}

double Node::getX() const
{
	return _x;
}

void Node::setX(double x)
{
	_x = x;
}

double Node::getY() const
{
	return _y;
}

void Node::setY(double y)
{
	_y = y;
}

double Node::getF() const
{
	return _f;
}

void Node::setF(double f)
{
	_f = f;
}

Node* Node::getParent() const
{
	return _parent;
}

void Node::setParent(Node* parent)
{
	_parent = parent;
}


