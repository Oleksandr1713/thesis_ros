/*
Copyright (c) 2011-2014, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <find_object_2d/ObjectsStamped.h>
#include <QtCore/QString>
#include <simulation/ScheduleJobMsg.h>
#include "cache/cache.h"
#include "cache/obstacle.h"
#include "simulation/AddObjectOnMapMsg.h"
#include "simulation/CleanCacheMsg.h"
#include "simulation/RemoveObjectFromMapMsg.h"
#include "my_lib/auxiliary_func.h"
#include "constants/node_constants.h"

using namespace auxiliary_func;

/* ROS node name*/
constexpr static const char* NODE_NAME = "obstacle_det_pos_calc_node";

class ObstacleDetectionAndPositionCalculation{

private:
    std::string targetFrameId_;
    std::string objFramePrefix_;
    ros::Subscriber subImageRec;
    ros::Publisher pubPauseAgv;
    ros::Publisher pubResumeAgv;
    ros::Publisher pubReplanAgv;
    ros::ServiceClient clientAddObject;
    ros::ServiceClient clientRemoveObject;
    ros::ServiceClient clientScheduleJob;
    ros::ServiceServer srvCleanCache;
    tf::TransformListener tfListener_;
    Cache cache;

public:
    ObstacleDetectionAndPositionCalculation(): targetFrameId_("/map"), objFramePrefix_("object"){
        ros::NodeHandle nhPrivate("~");
        nhPrivate.param("target_frame_id", targetFrameId_, targetFrameId_);
        nhPrivate.param("object_prefix", objFramePrefix_, objFramePrefix_);

        ros::NodeHandle nh_base;
        subImageRec = nh_base.subscribe("objectsStamped", 1, &ObstacleDetectionAndPositionCalculation::objectsDetectedCallback, this);

        pubPauseAgv = nh_base.advertise<std_msgs::Empty>(str(node_constants::TOPIC_PAUSE_GOAL), 1, false);
        pubResumeAgv = nh_base.advertise<std_msgs::Empty>(str(node_constants::TOPIC_RESUME_GOAL), 1, false);
        pubReplanAgv = nh_base.advertise<std_msgs::Empty>(str(node_constants::TOPIC_REPLAN_PATH), 1, false);

        clientAddObject = nh_base.serviceClient<simulation::AddObjectOnMapMsg>(str(node_constants::ADV_ENRICH_AUGMENTED_MAP));
        clientRemoveObject = nh_base.serviceClient<simulation::RemoveObjectFromMapMsg>(str(node_constants::ADV_IMPOVERISH_AUGMENTED_MAP));
        clientScheduleJob = nh_base.serviceClient<simulation::ScheduleJobMsg>(str(node_constants::ADV_JOB_SCHEDULER));

        srvCleanCache = nh_base.advertiseService(str(node_constants::ADV_CLEAN_CACHE), &ObstacleDetectionAndPositionCalculation::cleanCacheCallback, this);
    }

private:

    void cleanUpWhenAddObjectSrvFailed(Obstacle& obstacle){
        cache.removeElement(obstacle);
        pubResumeAgv.publish(std_msgs::Empty());
    }

    void cleanUpWhenSchedulerSrvFailed(Obstacle& obstacle, simulation::AddObjectOnMapMsg& aoom_msg){
        simulation::RemoveObjectFromMapMsg rofm_msg;
        rofm_msg.request.entry_id = aoom_msg.response.entry_id;

        clientRemoveObject.call(rofm_msg); // here maybe can be added additional check
        cache.removeElement(obstacle);
        pubResumeAgv.publish(std_msgs::Empty());
    }

    // Here I synchronize with the ObjectsStamped topic to
    // know when the TF is ready and for which objects
    void objectsDetectedCallback(const find_object_2d::ObjectsStampedConstPtr & msg)
    {
        if(msg->objects.data.size()){
            std::string targetFrameId = targetFrameId_;
            if(targetFrameId.empty()){
                targetFrameId = msg->header.frame_id;
            }
            char multiSubId = 'b';
            int previousId = -1;
            for(unsigned int i=0; i<msg->objects.data.size(); i+=12){
                // get data
                int id = (int)msg->objects.data[i];

                QString multiSuffix;
                if(id == previousId){
                    multiSuffix = QString("_") + multiSubId++;
                }
                else{
                    multiSubId = 'b';
                }
                previousId = id;

                // "object_1", "object_1_b", "object_1_c", "object_2"
                std::string objectFrameId = QString("%1_%2%3").arg(objFramePrefix_.c_str()).arg(id).arg(multiSuffix).toStdString();
                ROS_DEBUG("ODPC_1_start");
                tf::StampedTransform pose;
                tf::StampedTransform poseCam;
                try{
                    ROS_DEBUG("ODPC_2.1");
                    // Get transformation from "object_#" frame to target frame
                    tfListener_.waitForTransform(targetFrameId_, objectFrameId, msg->header.stamp, ros::Duration(1.0));
                    tfListener_.lookupTransform(targetFrameId_, objectFrameId, msg->header.stamp, pose);
                }
                catch(tf::TransformException & ex){
                    ROS_DEBUG("ODPC_2.2");
                    ROS_WARN("%s",ex.what());
                    continue;
                }

                Obstacle obstacle(pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z());
                // let's try to add a detected object to the cache
                if(cache.addElement(obstacle)){
                    ROS_DEBUG("ODPC_3");
                    ROS_INFO("Cache has been populated with a new obstacle.");
                    pubPauseAgv.publish(std_msgs::Empty());

                    simulation::AddObjectOnMapMsg aoom_msg;
                    aoom_msg.request.x_center = pose.getOrigin().x();
                    aoom_msg.request.y_center = pose.getOrigin().y();
                    aoom_msg.request.dir_radians = tf::getYaw(pose.getRotation());
                    // let's try to reflect the detected object on a static map
                    if (clientAddObject.call(aoom_msg)){
                        ROS_DEBUG("ODPC_4.1");
                        // let's check the success of reflecting the detected object on the static map
                        if(aoom_msg.response.success){
                            ROS_DEBUG("ODPC_5.1");
                            ROS_INFO("%s executed successfully!", str(node_constants::ADV_ENRICH_AUGMENTED_MAP).c_str());

                            simulation::ScheduleJobMsg sj_msg;
                            sj_msg.request.entry_id = aoom_msg.response.entry_id;
                            sj_msg.request.object_id = id;
                            // let's try to schedule a time of a object's removal from the static map
                            if (clientScheduleJob.call(sj_msg)){
                                ROS_DEBUG("ODPC_6.1");
                                // let's check the success of scheduling
                                if(sj_msg.response.success){
                                    ROS_DEBUG("ODPC_7.1");
                                    ROS_INFO("%s executed successfully!", str(node_constants::ADV_JOB_SCHEDULER).c_str());
                                    pubReplanAgv.publish(std_msgs::Empty());
                                } else{
                                    ROS_DEBUG("ODPC_7.2");
                                    ROS_ERROR("%s failed!", str(node_constants::ADV_JOB_SCHEDULER).c_str());
                                    cleanUpWhenSchedulerSrvFailed(obstacle, aoom_msg);
                                }
                            } else{
                                ROS_DEBUG("ODPC_6.2");
                                ROS_ERROR("Failed to call the %s service!", str(node_constants::ADV_JOB_SCHEDULER).c_str());
                                cleanUpWhenSchedulerSrvFailed(obstacle, aoom_msg);
                            }
                        } else {
                            ROS_DEBUG("ODPC_5.2");
                            ROS_ERROR("%s failed!", str(node_constants::ADV_ENRICH_AUGMENTED_MAP).c_str());
                            cleanUpWhenAddObjectSrvFailed(obstacle);
                        }
                    } else {
                        ROS_DEBUG("ODPC_4.2");
                        ROS_ERROR("Failed to call the %s service!", str(node_constants::ADV_ENRICH_AUGMENTED_MAP).c_str());
                        cleanUpWhenAddObjectSrvFailed(obstacle);
                    }
                } else{
                    ROS_INFO("Cache already contains this detected obstacle. Simply ignore it.");
                }
                ROS_DEBUG("ODPC_1_end");

            }
        }
    }

    bool cleanCacheCallback(simulation::CleanCacheMsg::Request &request, simulation::CleanCacheMsg::Response &response){
        Obstacle obstacle(request.x_center, request.y_center, 0);
        bool result = cache.removeElement(obstacle);
        response.success = result;
        return result;
    }

};

int main(int argc, char * argv[]){
    changeNodeLoggerLevel(ros::console::levels::Debug);
    ros::init(argc, argv, str(NODE_NAME));
    ObstacleDetectionAndPositionCalculation obstacle_dpc;
    ros::spin();
}