#ifndef SRC_OBSTACLE_H
#define SRC_OBSTACLE_H

static const float X_DEV = 0.5; // a deviation of the x coordinate (+/- in m)
static const float Y_DEV = 0.5; // a deviation of the y coordinate (+/- in m)
static const float Z_DEV = 0.5; // a deviation of the z coordinate (+/- in m)

class Obstacle{

public:
    float x, y, z;

    Obstacle(){}

    Obstacle(float x, float y, float z) {
        this->x = x;
        this->y = y;
        this->z = z;
    }

    bool checkObstaclesSimilarity(Obstacle& testObstacle){
        return (x - X_DEV) < testObstacle.x && testObstacle.x < (x + X_DEV) &&
               (y - Y_DEV) < testObstacle.y && testObstacle.y < (y + Y_DEV) &&
               (z - Z_DEV) < testObstacle.z && testObstacle.z < (z + Z_DEV);
    }
};

#endif //SRC_OBSTACLE_H
