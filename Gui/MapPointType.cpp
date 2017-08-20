#ifndef GUI_MAPPOINTTYPE_H_
#define GUI_MAPPOINTTYPE_H_

enum MapPointType {
	Unknown,
	Free,
	Obstacle,
	Path,
	PathStart,
	PathEnd,
	Waypoint,
	Particle,
	LidarScanObstacle,
	GoodParticle,
	BadParticle
};

#endif /* GUI_MAPPOINTTYPE_H_ */
