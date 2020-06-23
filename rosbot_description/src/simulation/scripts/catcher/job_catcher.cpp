#include "ros/ros.h"
#include "simulation/JobBriefInfo.h"

void callback(const simulation::JobBriefInfo& msg){
    ROS_INFO("I received: [%s]", msg.entry_id.c_str());
}

int main(int argc, char * argv[]) {
    ros::init(argc, argv, "job_catcher_node");
    ros::NodeHandle nh("~");

    ros::Subscriber sub = nh.subscribe("/catcher", 1, callback);


    ROS_INFO("Catcher node is up.");
    ros::spin();
    return 0;
}
