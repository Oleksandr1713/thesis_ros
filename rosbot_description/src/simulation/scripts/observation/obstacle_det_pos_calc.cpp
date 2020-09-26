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
#include "cache/cache.h"
#include "cache/obstacle.h"
#include "simulation/AddSignOnMapMsg.h"
#include "simulation/RemoveSignFromMapMsg.h"
#include "my_lib/auxiliary_func.h"
#include "constants/Constants.h"

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
    tf::TransformListener tfListener_;
    Cache cache;

public:
    ObstacleDetectionAndPositionCalculation(): targetFrameId_("/map"), objFramePrefix_("object"){
        ros::NodeHandle nh_private("~");
        nh_private.param("target_frame_id", targetFrameId_, targetFrameId_);
        nh_private.param("object_prefix", objFramePrefix_, objFramePrefix_);

        ros::NodeHandle nh_base;
        subs_ = nh_base.subscribe("objectsStamped", 1, &ObstacleDetectionAndPositionCalculation::objectsDetectedCallback, this);

        pub_pause_agv = nh_base.advertise<std_msgs::Empty>(str(constants::TOPIC_PAUSE_GOAL), 1, false);
        pub_resume_agv = nh_base.advertise<std_msgs::Empty>(str(constants::TOPIC_RESUME_GOAL), 1, false);
        pub_replan_agv = nh_base.advertise<std_msgs::Empty>(str(constants::TOPIC_REPLAN_PATH), 1, false);

        client_add_sign = nh_base.serviceClient<simulation::AddSignOnMapMsg>(str(constants::ADV_ENRICH_AUGMENTED_MAP));
        client_remove_sign = nh_base.serviceClient<simulation::RemoveSignFromMapMsg>(str(constants::ADV_IMPOVERISH_AUGMENTED_MAP));
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
                ROS_DEBUG("ODPC_1");
                tf::StampedTransform pose;
                tf::StampedTransform poseCam;
                try{
                    ROS_DEBUG("ODPC_2");
                    // Get transformation from "object_#" frame to target frame
                    tfListener_.waitForTransform(targetFrameId_, objectFrameId, msg->header.stamp, ros::Duration(1.0));
                    tfListener_.lookupTransform(targetFrameId_, objectFrameId, msg->header.stamp, pose);
                }
                catch(tf::TransformException & ex){
                    ROS_DEBUG("ODPC_2.1");
                    ROS_WARN("%s",ex.what());
                    continue;
                }

                Obstacle obstacle(pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z());
                if(cache.addElement(obstacle)){
                    ROS_DEBUG("ODPC_3");
                    ROS_INFO("Cache has been populated with a new obstacle.");
                    pub_pause_agv.publish(std_msgs::Empty());

                    simulation::AddSignOnMapMsg msg;
                    msg.request.sign_id = id;
                    msg.request.x_center = pose.getOrigin().x();
                    msg.request.y_center = pose.getOrigin().y();
                    msg.request.dir_radians = tf::getYaw(pose.getRotation());
                    if (client_add_sign.call(msg)){
                        ROS_DEBUG("ODPC_4");

                        if(msg.response.success){
                            ROS_DEBUG("ODPC_5");

                            ROS_INFO("%s executed successfully!", str(constants::ADV_ENRICH_AUGMENTED_MAP).c_str());
                            pub_replan_agv.publish(std_msgs::Empty());
                            // send request to scheduler

                        } else {
                            ROS_DEBUG("ODPC_5.1");

                            ROS_ERROR("%s failed!", str(constants::ADV_ENRICH_AUGMENTED_MAP).c_str());
                            cache.removeElement(obstacle);
                            pub_resume_agv.publish(std_msgs::Empty());
                        }
                    } else {
                        ROS_ERROR("Failed to call the %s service!", str(constants::ADV_ENRICH_AUGMENTED_MAP).c_str());
                        cache.removeElement(obstacle);
                        pub_resume_agv.publish(std_msgs::Empty());
                    }
                } else{
                    ROS_INFO("Cache already contains this detected obstacle. Simply ignore it.");
                }
                ROS_DEBUG("ODPC_6");

            }
        }
    }
};

int main(int argc, char * argv[]){
    changeNodeLoggerLevel(ros::console::levels::Debug);
    ros::init(argc, argv, str(NODE_NAME));
    ObstacleDetectionAndPositionCalculation obstacle_dpc;
    ros::spin();
}