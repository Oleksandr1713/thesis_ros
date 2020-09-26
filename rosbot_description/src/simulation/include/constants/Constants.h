#ifndef SRC_CONSTANTS_H
#define SRC_CONSTANTS_H

namespace constants {

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

/**** MONGO DATABASE ****/

    constexpr static const char *COLLECTION_NAME = "ros_sign_info";
}

#endif //SRC_CONSTANTS_H
