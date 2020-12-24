#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include "simulation/DestinationPose.h"
#include "control/move_base_client.h"
#include "my_lib/auxiliary_func.h"
#include "constants/node_constants.h"

using namespace auxiliary_func;

/* ROS node name*/
constexpr static const char* NODE_NAME = "cmd_control_node";

class CommandLineControl{

private:
    std::unique_ptr<MoveBaseClient> mbc;

    ros::Publisher pubGarbageCollector;
    ros::ServiceClient clientResetCostmap;
    ros::Subscriber subGarbageCollector;
    ros::Subscriber subGoTo;
    ros::Subscriber subCancelGoal;
    ros::Subscriber subPauseGoal;
    ros::Subscriber subResumeGoal;
    ros::Subscriber subReplanPath;

public:
    explicit CommandLineControl() {
        ros::NodeHandle nhBase;
        pubGarbageCollector = nhBase.advertise<std_msgs::Empty>(str(node_constants::TOPIC_GARBAGE_COLLECTOR), 1, false);
        subGarbageCollector = nhBase.subscribe(str(node_constants::TOPIC_GARBAGE_COLLECTOR), 1, &CommandLineControl::resetUniquePtrCallback, this);

        clientResetCostmap = nhBase.serviceClient<std_srvs::Empty>(str(node_constants::REQ_CLEAR_COSTMAP));

        subGoTo = nhBase.subscribe(str(node_constants::TOPIC_GO_TO), 1, &CommandLineControl::goToCallback, this);
        subCancelGoal = nhBase.subscribe(str(node_constants::TOPIC_CANCEL_GOAL), 1, &CommandLineControl::cancelGoalCallback, this);
        subPauseGoal = nhBase.subscribe(str(node_constants::TOPIC_PAUSE_GOAL), 1, &CommandLineControl::pauseGoalCallback, this);
        subResumeGoal = nhBase.subscribe(str(node_constants::TOPIC_RESUME_GOAL), 1, &CommandLineControl::resumeGoalCallback, this);

        subReplanPath = nhBase.subscribe(str(node_constants::TOPIC_REPLAN_PATH), 1, &CommandLineControl::replanPathCallback, this);
    }

private:
    void resetUniquePtrCallback(std_msgs::Empty msg) {
        mbc.reset();
    }

    double getVehicleOrientationInDegrees(const simulation::DestinationPose::ConstPtr& msg){
        switch(msg->orientation.c_str()[0]){
            case 'e': return node_constants::EAST;
            case 'n': return node_constants::NORTH;
            case 'w': return node_constants::WEST;
            case 's': return node_constants::SOUTH;
            default: return -1;
        }
    }

    void goToCallback(const simulation::DestinationPose::ConstPtr& msg){
        if(mbc != nullptr){
            ROS_INFO("New goal cannot be assigned, because the old one has not reached!");
            ROS_INFO("Status of the old goal: %s", mbc->getCurrentActionLibGoalState().toString().c_str());
        } else {
            double orientation_in_degrees = getVehicleOrientationInDegrees(msg);

            if(orientation_in_degrees == -1){
                ROS_WARN("Goal cannot be assigned, because received ros message is invalid! AGV orientation has not been set or set wrong!");
            } else{
                mbc = std::unique_ptr<MoveBaseClient>(new MoveBaseClient(pubGarbageCollector));
                boost::shared_ptr<geometry_msgs::Pose> goal = boost::shared_ptr<geometry_msgs::Pose>(new geometry_msgs::Pose);
                goal->position.x = msg.get()->x;
                goal->position.y = msg.get()->y;
                goal->position.z = 0;

                tf::Quaternion agv_orientation;
                agv_orientation = tf::createQuaternionFromRPY(0,0, getRadians(orientation_in_degrees)).normalized();

                goal->orientation.x = agv_orientation.x();
                goal->orientation.y = agv_orientation.y();
                goal->orientation.z = agv_orientation.z();
                goal->orientation.w = agv_orientation.w();

                resetCostmapToDefault();    // before starting to follow the goal, a global costmap must be equal to its primary state (reference map)
                mbc->goTo(goal);
            }
        }
    }

    void cancelGoalCallback(std_msgs::Empty msg){
        if(mbc != nullptr){
            mbc->cancelGoal();
            ROS_INFO("Goal has been canceled!");
        } else {
            ROS_WARN("Nothing to cancel!");
        }
    }

    void pauseGoalCallback(std_msgs::Empty msg){
        if(mbc != nullptr){
            mbc->pauseGoal();
            ROS_INFO("Goal has been paused!");
        } else {
            ROS_WARN("Nothing to pause!");
        }
    }

    void resumeGoalCallback(std_msgs::Empty msg){
        if(mbc != nullptr && mbc->getCurrentSelfGoalState() == MoveBaseClient::PAUSED){
            mbc->resumeGoal();
            ROS_INFO("Goal has been resumed!");
        } else {
            ROS_WARN("Nothing to resume!");
        }
    }

    void replanPathCallback(std_msgs::Empty msg){
        if(mbc != nullptr && mbc->getCurrentSelfGoalState() == MoveBaseClient::PAUSED){
            ROS_INFO("Path re-planning has been started...");

            ros::Duration(node_constants::DELAY).sleep();   // this line gives some delay to ensure that the static map update has occurred and is ready to be used

            boost::shared_ptr<geometry_msgs::Pose> source = mbc->getPSource();
            boost::shared_ptr<geometry_msgs::Pose> destination = mbc->getPDestination();

            resetUniquePtrCallback(std_msgs::Empty());
            mbc = std::unique_ptr<MoveBaseClient>(new MoveBaseClient(pubGarbageCollector));
            mbc->setPSource(source);
            mbc->goTo(destination);
        } else {
            ROS_WARN("Nothing to re-plan!");
        }
    }

    bool resetCostmapToDefault(){
        std_srvs::Empty empty;
        if (clientResetCostmap.call(empty)){
            ROS_INFO("Resetting of global costmap was SUCCESSFUL!");
            return true;
        } else {
            ROS_ERROR("Reset of global costmap FAILED!");
            return false;
        }
    }

};

int main(int argc, char** argv){
    /*changeNodeLoggerLevel(ros::console::levels::Debug);*/
    ros::init(argc, argv, str(NODE_NAME));
    CommandLineControl cmd_control;
    ros::spin();
    return 0;
}