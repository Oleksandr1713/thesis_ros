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
#include "simulation/AddSignOnMapMsg.h"
#include "simulation/CleanCacheMsg.h"
#include "simulation/RemoveSignFromMapMsg.h"
#include "my_lib/auxiliary_func.h"
#include "constants/node_constants.h"

using namespace auxiliary_func;

/* ROS node name*/
constexpr static const char* NODE_NAME = "obstacle_det_pos_calc_node";

class ObstacleDetectionAndPositionCalculation{

private:
    std::string targetFrameId_;
    std::string objFramePrefix_;
    ros::Subscriber subs_;
    ros::Publisher pub_pause_agv;
    ros::Publisher pub_resume_agv;
    ros::Publisher pub_replan_agv;
    ros::ServiceClient client_add_sign;
    ros::ServiceClient client_remove_sign;
    ros::ServiceClient client_schedule_job;
    ros::ServiceServer srv_clean_cache;
    tf::TransformListener tfListener_;
    Cache cache;

public:
    ObstacleDetectionAndPositionCalculation(): targetFrameId_("/map"), objFramePrefix_("object"){
        ros::NodeHandle nh_private("~");
        nh_private.param("target_frame_id", targetFrameId_, targetFrameId_);
        nh_private.param("object_prefix", objFramePrefix_, objFramePrefix_);

        ros::NodeHandle nh_base;
        subs_ = nh_base.subscribe("objectsStamped", 1, &ObstacleDetectionAndPositionCalculation::objectsDetectedCallback, this);

        pub_pause_agv = nh_base.advertise<std_msgs::Empty>(str(node_constants::TOPIC_PAUSE_GOAL), 1, false);
        pub_resume_agv = nh_base.advertise<std_msgs::Empty>(str(node_constants::TOPIC_RESUME_GOAL), 1, false);
        pub_replan_agv = nh_base.advertise<std_msgs::Empty>(str(node_constants::TOPIC_REPLAN_PATH), 1, false);

        client_add_sign = nh_base.serviceClient<simulation::AddSignOnMapMsg>(str(node_constants::ADV_ENRICH_AUGMENTED_MAP));
        client_remove_sign = nh_base.serviceClient<simulation::RemoveSignFromMapMsg>(str(node_constants::ADV_IMPOVERISH_AUGMENTED_MAP));
        client_schedule_job = nh_base.serviceClient<simulation::ScheduleJobMsg>(str(node_constants::ADV_JOB_SCHEDULER));

        srv_clean_cache = nh_base.advertiseService(str(node_constants::ADV_CLEAN_CACHE), &ObstacleDetectionAndPositionCalculation::cleanCacheCallback, this);
    }

private:

    void cleanUpWhenAddSignSrvFailed(Obstacle& obstacle){
        cache.removeElement(obstacle);
        pub_resume_agv.publish(std_msgs::Empty());
    }

    void cleanUpWhenSchedulerSrvFailed(Obstacle& obstacle, simulation::AddSignOnMapMsg& asom_msg){
        simulation::RemoveSignFromMapMsg rsfm_msg;
        rsfm_msg.request.entry_id = asom_msg.response.entry_id;

        client_remove_sign.call(rsfm_msg); // here maybe can be added additional check
        cache.removeElement(obstacle);
        pub_resume_agv.publish(std_msgs::Empty());
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
                    pub_pause_agv.publish(std_msgs::Empty());

                    simulation::AddSignOnMapMsg asom_msg;
                    asom_msg.request.x_center = pose.getOrigin().x();
                    asom_msg.request.y_center = pose.getOrigin().y();
                    asom_msg.request.dir_radians = tf::getYaw(pose.getRotation());
                    // let's try to reflect the detected object on a static map
                    if (client_add_sign.call(asom_msg)){
                        ROS_DEBUG("ODPC_4.1");
                        // let's check the success of reflecting the detected object on the static map
                        if(asom_msg.response.success){
                            ROS_DEBUG("ODPC_5.1");
                            ROS_INFO("%s executed successfully!", str(node_constants::ADV_ENRICH_AUGMENTED_MAP).c_str());

                            simulation::ScheduleJobMsg sj_msg;
                            sj_msg.request.entry_id = asom_msg.response.entry_id;
                            sj_msg.request.sign_id = id;
                            // let's try to schedule a time of a object's removal from the static map
                            if (client_schedule_job.call(sj_msg)){
                                ROS_DEBUG("ODPC_6.1");
                                // let's check the success of scheduling
                                if(sj_msg.response.success){
                                    ROS_DEBUG("ODPC_7.1");
                                    ROS_INFO("%s executed successfully!", str(node_constants::ADV_JOB_SCHEDULER).c_str());
                                    pub_replan_agv.publish(std_msgs::Empty());
                                } else{
                                    ROS_DEBUG("ODPC_7.2");
                                    ROS_ERROR("%s failed!", str(node_constants::ADV_JOB_SCHEDULER).c_str());
                                    cleanUpWhenSchedulerSrvFailed(obstacle, asom_msg);
                                }
                            } else{
                                ROS_DEBUG("ODPC_6.2");
                                ROS_ERROR("Failed to call the %s service!", str(node_constants::ADV_JOB_SCHEDULER).c_str());
                                cleanUpWhenSchedulerSrvFailed(obstacle, asom_msg);
                            }
                        } else {
                            ROS_DEBUG("ODPC_5.2");
                            ROS_ERROR("%s failed!", str(node_constants::ADV_ENRICH_AUGMENTED_MAP).c_str());
                            cleanUpWhenAddSignSrvFailed(obstacle);
                        }
                    } else {
                        ROS_DEBUG("ODPC_4.2");
                        ROS_ERROR("Failed to call the %s service!", str(node_constants::ADV_ENRICH_AUGMENTED_MAP).c_str());
                        cleanUpWhenAddSignSrvFailed(obstacle);
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