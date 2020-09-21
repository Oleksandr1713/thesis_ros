#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_msgs/Empty.h>
#include "control/move_base_client.h"

/* ROS node name*/
const std::string NODE_NAME = "cmd_control_node";

class CommandLineControl{

public:
    std::unique_ptr<MoveBaseClient> mbc;

    ros::Publisher pub_garbage_collector;
    ros::Subscriber sub_garbage_collector;
    ros::Subscriber sub_go_to;
    ros::Subscriber sub_cancel_goal;
    ros::Subscriber sub_pause_goal;
    ros::Subscriber sub_resume_goal;
    ros::Subscriber sub_replan_path;

private:
    /* Topic names */
    const std::string TOPIC_GO_TO = "go_to";               // communication channel to request a navigation to a destination goal
    const std::string TOPIC_CANCEL_GOAL = "cancel_goal";   // communication channel to cancel a currently running navigation goal
    const std::string TOPIC_PAUSE_GOAL = "pause_goal";     // communication channel to pause a currently running navigation goal
    const std::string TOPIC_RESUME_GOAL = "resume_goal";   // communication channel to resume a currently running navigation goal
    const std::string TOPIC_REPLAN_PATH = "replan_path";   // communication channel to find a new path to the already assigned destination goal
    const std::string TOPIC_GARBAGE_COLLECTOR = "garbage_collector";  // communication channel to trigger a smart-pointer reset

public:
    explicit CommandLineControl(ros::NodeHandle& nh) {
        pub_garbage_collector = nh.advertise<std_msgs::Empty>(TOPIC_GARBAGE_COLLECTOR, 1, false);
        sub_garbage_collector = nh.subscribe(TOPIC_GARBAGE_COLLECTOR, 1, &CommandLineControl::resetUniquePtrCallback, this);

        sub_go_to = nh.subscribe(TOPIC_GO_TO, 1, &CommandLineControl::goToCallback, this);
        sub_cancel_goal = nh.subscribe(TOPIC_CANCEL_GOAL, 1, &CommandLineControl::cancelGoalCallback, this);
        sub_pause_goal = nh.subscribe(TOPIC_PAUSE_GOAL, 1, &CommandLineControl::pauseGoalCallback, this);
        sub_resume_goal = nh.subscribe(TOPIC_RESUME_GOAL, 1, &CommandLineControl::resumeGoalCallback, this);

        sub_replan_path = nh.subscribe(TOPIC_REPLAN_PATH, 1, &CommandLineControl::replanPathCallback, this);
    }

    void resetUniquePtrCallback(std_msgs::Empty msg) {
        mbc.reset();
    }

    void goToCallback(const geometry_msgs::Point::ConstPtr& msg){
        if(mbc != nullptr){
            ROS_INFO("New goal cannot be assigned, because the old one has not reached!");
            ROS_INFO("Status of the old goal: %s", mbc->getCurrentActionLibGoalState().toString().c_str());
        } else {
            mbc = std::unique_ptr<MoveBaseClient>(new MoveBaseClient(pub_garbage_collector));
            boost::shared_ptr<geometry_msgs::Point> goal = boost::shared_ptr<geometry_msgs::Point>(new geometry_msgs::Point);
            goal->x = msg.get()->x;
            goal->y = msg.get()->y;
            goal->z = msg.get()->z;
            mbc->goTo(goal);
        }
    }

    void cancelGoalCallback(std_msgs::Empty msg){
        if(mbc != nullptr){
            mbc->cancelGoal();
            ROS_INFO("Goal has been canceled!");
            ROS_INFO("Status of the goal: %s", mbc->getCurrentActionLibGoalState().toString().c_str());
            resetUniquePtrCallback(std_msgs::Empty());
        } else {
            ROS_INFO("Nothing to cancel!");
        }
    }

    void pauseGoalCallback(std_msgs::Empty msg){
        if(mbc != nullptr){
            mbc->pauseGoal();
            ROS_INFO("Goal has been paused!");
        } else {
            ROS_INFO("Nothing to pause!");
        }
    }

    void resumeGoalCallback(std_msgs::Empty msg){
        if(mbc != nullptr && mbc->getCurrentSelfGoalState() == MoveBaseClient::PAUSED){
            mbc->resumeGoal();
            ROS_INFO("Goal has been resumed!");
        } else {
            ROS_INFO("Nothing to resume!");
        }
    }

    void replanPathCallback(std_msgs::Empty msg){
        if(mbc != nullptr && mbc->getCurrentSelfGoalState() == MoveBaseClient::PAUSED){
            ROS_INFO("Path replanning has been started...");
            boost::shared_ptr<geometry_msgs::Point> source = mbc->getPSource();
            boost::shared_ptr<geometry_msgs::Point> destination = mbc->getPDestination();

            resetUniquePtrCallback(std_msgs::Empty());
            mbc = std::unique_ptr<MoveBaseClient>(new MoveBaseClient(pub_garbage_collector));
            mbc->setPSource(source);
            mbc->goTo(destination);
        } else {
            ROS_INFO("Nothing to replan!");
        }
    }
};

int main(int argc, char** argv){
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle nh("~");
    CommandLineControl cmd_control(nh);
    ros::spin();
    return 0;
}