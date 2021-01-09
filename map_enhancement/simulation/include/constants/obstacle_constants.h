#ifndef SRC_OBSTACLE_CONSTANTS_H
#define SRC_OBSTACLE_CONSTANTS_H

namespace obstacle_constants {

    static int const STOP_SIGN_ID = 9; // this id matches the id obtained using find_object_2d package during image recognition training
    static long const STOP_SIGN_LIFESPAN = 3600; // seconds

    static int const BLOCK_SIGN_ID = 2;
    static long const BLOCK_SIGN_LIFESPAN = 300; // seconds

    static int const CLOSED_SIGN_ID = 3;
    static long const CLOSED_SIGN_LIFESPAN = 900; // seconds
}
#endif //SRC_OBSTACLE_CONSTANTS_H
