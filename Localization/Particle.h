
#ifndef PARTICLE_H_
#define PARTICLE_H_

//this class represent a particale in the map window
class Particle {
public:
    int row, col; //the row/col index in the map
    double x, y; //the exact place of the robot; x- place in column, y- place in row
    double yaw; //the heading angle of the robot
    double belief;

    //constructor
    Particle(int i = 0, int j = 0, double x = 0,
                         double y = 0, double yaw = 0, double belief = 0)
            : row(i), col(j), x(x), y(y), yaw(yaw), belief(belief) {}

    ~Particle() {}
};

#endif /* PARTICLE_H_ */
