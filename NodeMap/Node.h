#ifndef NODE_H_
#define NODE_H_

#include <cstdlib>

class Node
{
	bool _isObstacle;
	bool _isInOpenList;
	bool _isInClosedList;
	bool _isWaypoint;
	double _g;
	double _h;
	double _f;
	double _x;
	double _y;
	Node* _parent;


public:
	bool getIsObstacle() const;
	void setIsObstacle(bool isObstacle);
	bool getIsInOpenList() const;
	void setIsInOpenList(bool isInOpenList);
	bool getIsInClosedList() const;
	void setIsInClosedList(bool isInClosedList);
	bool getIsWaypoint() const;
	void setIsWaypoint(bool isWaypoint);
	double getG() const;
	void setG(double g);
	double getH() const;
	void setH(double h);
	double getX() const;
	void setX(double x);
	double getY() const;
	void setY(double y);
	double getF() const;
	void setF(double f);
	Node* getParent() const;
	void setParent(Node* parent);
	Node();
	Node(double x, double y);
	virtual ~Node();


	bool operator<(const Node& node) const
	{
		if ((_y < node._y) || (_y == node._y && _x <= node._x))
		{
			return true;
		}

		return false;
	}
};

#endif  NODE_H_

