#ifndef SRC_NODE_CONSTANTS_H
#define SRC_NODE_CONSTANTS_H

namespace node_constants {

/**** MAP ENRICHMENT NODE ****/

    /* Topic names */
    constexpr static const char *PUB_AUGMENTED_MAP = "/simulation/augmented_map";          // topic to publish the updated map via OccupancyGrid message

    /* Service names */
    constexpr static const char *REQ_ORIGINAL_MAP = "/static_map";                         // service name, where the original static map can be requested
    constexpr static const char *ADV_ENRICH_AUGMENTED_MAP = "/simulation/add_sign_on_map";             // name of advertised service that can be requested by other nodes to add a sign on the map
    constexpr static const char *ADV_IMPOVERISH_AUGMENTED_MAP = "/simulation/remove_sign_from_map";    // name of advertised service that can be requested by other nodes to remove a sign from the map


/**** CMD CONTROL NODE ****/

    /* Topic names */
    constexpr static const char *TOPIC_GO_TO = "/simulation/go_to";               // communication channel to request a navigation to a destination goal
    constexpr static const char *TOPIC_CANCEL_GOAL = "/simulation/cancel_goal";   // communication channel to cancel a currently running navigation goal
    constexpr static const char *TOPIC_PAUSE_GOAL = "/simulation/pause_goal";     // communication channel to pause a currently running navigation goal
    constexpr static const char *TOPIC_RESUME_GOAL = "/simulation/resume_goal";   // communication channel to resume a currently running navigation goal
    constexpr static const char *TOPIC_REPLAN_PATH = "/simulation/replan_path";   // communication channel to find a new path to the already assigned destination goal
    constexpr static const char *TOPIC_GARBAGE_COLLECTOR = "/simulation/garbage_collector";  // communication channel to trigger a smart-pointer reset

    /* Delay in seconds before starting path re-planning */
    static const double DELAY = 2.0;    // it`s needed to ensure that the static map has been updated and the node can use its updated version

    /* Orientation of AGV head at the destination point */
    static const double EAST = 0;       // degrees
    static const double NORTH = 90;     // degrees
    static const double WEST = 180;     // degrees
    static const double SOUTH = 270;    // degrees


/**** MONGO DATABASE ****/

    constexpr static const char *COLLECTION_NAME = "ros_sign_info";


/**** JOB SCHEDULER NODE ****/

    /* Service names */
    constexpr static const char *ADV_JOB_SCHEDULER = "/simulation/schedule_job";


/**** JOB SCHEDULER NODE ****/

    /* Topic names */
    constexpr static const char *TOPIC_JOB_CATCHER = "/simulation/catcher"; // this topic name must be also set in JOB SCHEDULER, where AT-unix utility is used


/**** OBSTACLE DETECTION AND POSITION CALCULATION NODE ****/

    /* Service names */
    constexpr static const char *ADV_CLEAN_CACHE = "/simulation/clean_cache";
}

#endif //SRC_NODE_CONSTANTS_H
