#ifndef STRUCTS_H_
#define STRUCTS_H_

struct rectangle {
    unsigned startingX;
    unsigned startingY;
    unsigned endingX;
    unsigned endingY;
};

struct size {
    double width;
    double length;
};

struct position {
    double x;
    double y;
};

struct positionState {
    struct position pos;
    double yaw;
};

#endif  STRUCTS_H_

