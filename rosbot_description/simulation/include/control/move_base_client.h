#ifndef SRC_MOVE_BASE_CLIENT_H
#define SRC_MOVE_BASE_CLIENT_H

#include <cmath>
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
    boost::shared_ptr<geometry_msgs::Pose> pCurrent;
    boost::shared_ptr<geometry_msgs::Pose> pDestination;

    /*Log data*/
    ros::Time startSimTime;
    ros::Time endSimTime;
    ros::WallTime startWallTime;
    ros::WallTime endWallTime;
    float traveledDistance = 0;

public:
    explicit MoveBaseClient(ros::Publisher& pubGarbageCollector): ac("move_base", true),
                                                                  goalState(UNASSIGNED){
        this->pubGarbageCollector = pubGarbageCollector;
        ROS_INFO("Waiting for the move_base action server to come up...");
        ac.waitForServer();
        ROS_INFO("Client object has been created and connected to the move_base action server");
    }

    ~MoveBaseClient(){
        ROS_INFO("Client object has been destroyed");
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

        ROS_INFO("Sending goal...");
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
            startSimTime = ros::Time::now();
            startWallTime = ros::WallTime::now();

            pSource = boost::shared_ptr<geometry_msgs::Pose>(new geometry_msgs::Pose);
            pCurrent = boost::shared_ptr<geometry_msgs::Pose>(new geometry_msgs::Pose);

            pSource->position = feedback.get()->base_position.pose.position;
            pSource->orientation = feedback.get()->base_position.pose.orientation;
            pCurrent = pSource;
        } else{
            boost::shared_ptr<geometry_msgs::Pose> pTemp = boost::shared_ptr<geometry_msgs::Pose>(new geometry_msgs::Pose);
            pTemp->position = feedback.get()->base_position.pose.position;
            pTemp->orientation = feedback.get()->base_position.pose.orientation;

            traveledDistance += calculateDistance(pTemp, pCurrent);

            pCurrent.reset();
            pCurrent = pTemp;
        }
        ROS_DEBUG("Feedback msg received at pose (%.2f, %.2f)", feedback->base_position.pose.position.x, feedback->base_position.pose.position.y);
    }

    // Called once when the goal completes
    void doneCallback(const actionlib::SimpleClientGoalState& state,
                      const move_base_msgs::MoveBaseResultConstPtr& result){
        ROS_INFO("Finished in state [%s] & [%s]", state.toString().c_str(), toStringCurrentSelfGoalState().c_str());
        if(state == actionlib::SimpleClientGoalState::SUCCEEDED && goalState == ACTIVE){
            goalState = COMPLETED;
            endSimTime = ros::Time::now();
            endWallTime = ros::WallTime::now();
            requestSelfDestruction();
        } else if(state == actionlib::SimpleClientGoalState::PREEMPTED && goalState == ACTIVE){
            goalState = CANCELED;
            endSimTime = ros::Time::now();
            endWallTime = ros::WallTime::now();
            requestSelfDestruction();
        } else if(state == actionlib::SimpleClientGoalState::ABORTED){
            /*
             * This case happens when an assigned destination coordinate is outside the navigation map.
            Therefore, this coordinate cannot be reached, because it does not exist.
             */
            ROS_INFO("A valid plan to a destination point couldn't be found! [%s]", state.toString().c_str());

            if(arePositionsEqual(pSource->position, pCurrent->position)){
                requestSelfDestruction();
            } else{
                goTo(pSource); // go back to source
            }
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
    }

    void pauseGoal(){
        goalState = PAUSED;
        ac.cancelGoal();
    }

    void resumeGoal(){
        goTo(pDestination);
    }

private:

    static float calculateDistance(const boost::shared_ptr<geometry_msgs::Pose> &newPosition, const boost::shared_ptr<geometry_msgs::Pose> &oldPosition){
        float x2 = newPosition->position.x;
        float y2 = newPosition->position.y;
        float x1 = oldPosition->position.x;
        float y1 = oldPosition->position.y;
        return std::hypot(x2 - x1, y2 - y1);
    }

    static bool arePositionsEqual(geometry_msgs::Point &p1, geometry_msgs::Point &p2){
        return int(p1.x) == int(p2.x) && int(p1.y) == int(p2.y);
    }

    void printCollectedLogData(){
        if(endSimTime.isZero()){
            ROS_ERROR("Collected log data cannot be displayed. Something went wrong during path following!");
        } else{
            ROS_INFO_STREAM(">>> Traveled Simulation Time: " << endSimTime.toSec() - startSimTime.toSec() << " [s]");
            ROS_INFO_STREAM(">>> Traveled Wall Time: " << endWallTime.toSec() - startWallTime.toSec() << " [s]");
            ROS_INFO_STREAM(">>> Traveled distance: " << traveledDistance << " [m]");
        }
    }

    void requestSelfDestruction(){
        printCollectedLogData();
        pubGarbageCollector.publish(std_msgs::Empty());
    }
};
#endif //SRC_MOVE_BASE_CLIENT_H