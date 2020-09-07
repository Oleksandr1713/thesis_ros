#ifndef SRC_OBSTACLE_H
#define SRC_OBSTACLE_H

static const float X_DEV = 0.1; // a deviation of the x coordinate (+/- in m)
static const float Y_DEV = 0.1; // a deviation of the y coordinate (+/- in m)
static const float Z_DEV = 0.1; // a deviation of the z coordinate (+/- in m)

class Obstacle{

public:
    float x, y, z;

    Obstacle(){}

    Obstacle(float x, float y, float z) {
        this->x = x;
        this->y = y;
        this->z = z;
    }

    bool checkObstaclesSimilarity(Obstacle& test_obstacle){
        return (x - X_DEV) < test_obstacle.x && test_obstacle.x < (x + X_DEV) &&
               (y - Y_DEV) < test_obstacle.y && test_obstacle.y < (y + Y_DEV) &&
               (z - Z_DEV) < test_obstacle.z && test_obstacle.z < (z + Z_DEV);
    }
};

#endif //SRC_OBSTACLE_H
