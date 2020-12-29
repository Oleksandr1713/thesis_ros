#include "ros/ros.h"
#include "simulation/JobBriefInfo.h"
#include "simulation/CleanCacheMsg.h"
#include "simulation/RemoveObjectFromMapMsg.h"
#include "my_lib/auxiliary_func.h"
#include "constants/node_constants.h"

using namespace simulation;
using namespace auxiliary_func;

/* ROS node name*/
constexpr static const char* NODE_NAME = "job_receiver_node";

class JobReceiver{

private:
    ros::Subscriber subJobCatcher;
    ros::ServiceClient clientRemoveObject;
    ros::ServiceClient clientEmptyCache;

public:
    JobReceiver() {
        ros::NodeHandle nhBase;
        subJobCatcher = nhBase.subscribe(str(node_constants::TOPIC_JOB_RECEIVER), 1, &JobReceiver::jobReceiverCallback, this);
        clientRemoveObject = nhBase.serviceClient<simulation::RemoveObjectFromMapMsg>(str(node_constants::ADV_IMPOVERISH_AUGMENTED_MAP));
        clientEmptyCache = nhBase.serviceClient<simulation::CleanCacheMsg>(str(node_constants::ADV_CLEAN_CACHE));
        ROS_INFO("Job receiver node is up.");
    }

    ~JobReceiver(){
        ROS_INFO("Job receiver node is destroyed.");
    }

private:
    void jobReceiverCallback(const simulation::JobBriefInfo& msg){
        RemoveObjectFromMapMsg rofm_msg;
        rofm_msg.request.entry_id = msg.entry_id;
        // let's try to request a sign removal from the static map and, in consequence, from MongoDB
        if(clientRemoveObject.call(rofm_msg)){

            // let's check the success of the sign removal request
            if(rofm_msg.response.success){
                ROS_INFO("%s executed successfully!", str(node_constants::ADV_IMPOVERISH_AUGMENTED_MAP).c_str());

                CleanCacheMsg cc_msg;
                cc_msg.request.x_center = rofm_msg.response.x_center;
                cc_msg.request.y_center = rofm_msg.response.y_center;
                // let's try to request the cache to clear all the information about the sign deleted above
                if(clientEmptyCache.call(cc_msg)){

                    // let's check the success of the cache clearing
                    if(cc_msg.response.success){
                        ROS_INFO("%s executed successfully!", str(node_constants::ADV_CLEAN_CACHE).c_str());
                    } else{
                        ROS_ERROR("%s failed!", str(node_constants::ADV_CLEAN_CACHE).c_str());
                    }
                } else{
                    ROS_ERROR("Failed to call the %s service!", str(node_constants::ADV_CLEAN_CACHE).c_str());
                }
            } else{
                ROS_ERROR("%s failed!", str(node_constants::ADV_IMPOVERISH_AUGMENTED_MAP).c_str());
            }
        } else{
            ROS_ERROR("Failed to call the %s service!", str(node_constants::ADV_IMPOVERISH_AUGMENTED_MAP).c_str());
        }
    }
};

int main(int argc, char * argv[]) {
    ros::init(argc, argv, str(NODE_NAME));
    JobReceiver jobReceiver;
    ros::spin();
    return 0;
}
