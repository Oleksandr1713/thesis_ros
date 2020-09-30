#include "ros/ros.h"
#include "simulation/JobBriefInfo.h"
#include "simulation/CleanCacheMsg.h"
#include "simulation/RemoveSignFromMapMsg.h"
#include "my_lib/auxiliary_func.h"
#include "constants/node_constants.h"

using namespace simulation;
using namespace auxiliary_func;

/* ROS node name*/
constexpr static const char* NODE_NAME = "job_catcher_node";

class JobCatcher{

private:
    ros::Subscriber sub_job_catcher;
    ros::ServiceClient client_remove_sign;
    ros::ServiceClient client_empty_cache;

public:
    JobCatcher() {
        ros::NodeHandle nh_base;
        sub_job_catcher = nh_base.subscribe(str(node_constants::TOPIC_JOB_CATCHER), 1, &JobCatcher::callback, this);
        client_remove_sign = nh_base.serviceClient<simulation::RemoveSignFromMapMsg>(str(node_constants::ADV_IMPOVERISH_AUGMENTED_MAP));
        client_empty_cache = nh_base.serviceClient<simulation::CleanCacheMsg>(str(node_constants::ADV_CLEAN_CACHE));
        ROS_INFO("Job catcher node is up.");
    }

    ~JobCatcher(){
        ROS_INFO("Job catcher node is destroyed.");
    }

private:
    void callback(const simulation::JobBriefInfo& msg){
        RemoveSignFromMapMsg rsfm_msg;
        rsfm_msg.request.entry_id = msg.entry_id;
        // let's try to request a sign removal from the static map and, in consequence, from MongoDB
        if(client_remove_sign.call(rsfm_msg)){

            // let's check the success of the sign removal request
            if(rsfm_msg.response.success){
                ROS_INFO("%s executed successfully!", str(node_constants::ADV_IMPOVERISH_AUGMENTED_MAP).c_str());

                CleanCacheMsg cc_msg;
                cc_msg.request.x_center = rsfm_msg.response.x_center;
                cc_msg.request.y_center = rsfm_msg.response.y_center;
                // let's try to request the cache to clear all the information about the sign deleted above
                if(client_empty_cache.call(cc_msg)){

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
    JobCatcher jobCatcher;
    ros::spin();
    return 0;
}
