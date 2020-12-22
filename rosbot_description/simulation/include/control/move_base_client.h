#ifndef SRC_MOVE_BASE_CLIENT_H
#define SRC_MOVE_BASE_CLIENT_H

#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ActionClient;

class MoveBaseClient{

public:
    /* Possible navigation goal states */
    enum State{
        UNASSIGNED,
        PAUSED,
        CANCELED,
        ACTIVE,
        COMPLETED
    };

private:
    ActionClient ac;
    State goalState;
    ros::Publisher pubGarbageCollector;
    boost::shared_ptr<geometry_msgs::Pose> pSource;
    boost::shared_ptr<geometry_msgs::Pose> pDestination;

public:
    explicit MoveBaseClient(ros::Publisher& pubGarbageCollector): ac("move_base", true),
                                                                  goalState(UNASSIGNED){
        this->pubGarbageCollector = pubGarbageCollector;
        ROS_INFO("Waiting for the move_base action server to come up");
        ac.waitForServer();
        ROS_INFO("Client is connected to the move_base action server successfully");
    }

    ~MoveBaseClient(){
        ROS_INFO("Client object destroyed");
    }

    const boost::shared_ptr<geometry_msgs::Pose> &getPSource() const {
        return pSource;
    }

    void setPSource(const boost::shared_ptr<geometry_msgs::Pose> &pSource) {
        MoveBaseClient::pSource = pSource;
    }

    const boost::shared_ptr<geometry_msgs::Pose> &getPDestination() const {
        return pDestination;
    }

    void setPDestination(const boost::shared_ptr<geometry_msgs::Pose> &pDestination) {
        MoveBaseClient::pDestination = pDestination;
    }

    void goTo(boost::shared_ptr<geometry_msgs::Pose>& refPtrMsg){
        setPDestination(refPtrMsg);
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose = refPtrMsg.operator*();

        ROS_INFO("Sending goal");
        ac.sendGoal(goal,
                    boost::bind(&MoveBaseClient::doneCallback, this, _1, _2),
                    boost::bind(&MoveBaseClient::activeCallback, this),
                    boost::bind(&MoveBaseClient::feedbackCallback, this, _1));
    }

    // Called once when the goal becomes active
    void activeCallback(){
        goalState = ACTIVE;
        ROS_INFO("Goal just went active");
    }

    // Called every time feedback is received for the goal
    void feedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback){
        if(pSource == nullptr){
            pSource = boost::shared_ptr<geometry_msgs::Pose>(new geometry_msgs::Pose);
            pSource->position = feedback.get()->base_position.pose.position;
            pSource->orientation = feedback.get()->base_position.pose.orientation;
        }
        ROS_DEBUG("Feedback msg received at pose (%.2f, %.2f)", feedback->base_position.pose.position.x, feedback->base_position.pose.position.y);
    }

    // Called once when the goal completes
    void doneCallback(const actionlib::SimpleClientGoalState& state,
                      const move_base_msgs::MoveBaseResultConstPtr& result){
        ROS_INFO("Finished in state [%s] & [%s]", state.toString().c_str(), toStringCurrentSelfGoalState().c_str());
        if(state == actionlib::SimpleClientGoalState::SUCCEEDED || state == actionlib::SimpleClientGoalState::PREEMPTED){
            if(goalState == ACTIVE){
                goalState = COMPLETED;
                requestSelfDestruction();
            }
        } else if(state == actionlib::SimpleClientGoalState::ABORTED){
            /*
             * This case happens when an assigned destination coordinate is outside the navigation map.
            Therefore, this coordinate cannot be reached, because it does not exist.
             */
            ROS_INFO("A valid plan to a destination point couldn't be found! [%s]", state.toString().c_str());
            goTo(pSource); // go back to source
        } else{
            ROS_INFO("Alarm! This case hasn't been considered: [%s] & [%s]", state.toString().c_str(), toStringCurrentSelfGoalState().c_str());
            requestSelfDestruction();
        }
    }

    State getCurrentSelfGoalState(){
        return goalState;
    }

    std::string toStringCurrentSelfGoalState(){
        switch (goalState) {
            case UNASSIGNED: return "UNASSIGNED";
            case PAUSED: return "PAUSED";
            case CANCELED: return "CANCELED";
            case ACTIVE: return "ACTIVE";
            case COMPLETED: return "COMPLETED";
            default: return "BUG-UNKNOWN";
        }
    }

    actionlib::SimpleClientGoalState getCurrentActionLibGoalState(){
        return ac.getState();
    }

    void cancelGoal(){
        ac.cancelGoal();
        goalState = CANCELED;
    }

    void pauseGoal(){
        ac.cancelGoal();
        goalState = PAUSED;
    }

    void resumeGoal(){
        goTo(pDestination);
    }

    void requestSelfDestruction(){
        pubGarbageCollector.publish(std_msgs::Empty());
    }
};
#endif //SRC_MOVE_BASE_CLIENT_H