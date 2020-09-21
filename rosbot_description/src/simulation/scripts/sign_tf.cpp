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

class SignTransform{

private:
    std::string targetFrameId_;
    std::string objFramePrefix_;
    ros::Subscriber subs_;
    tf::TransformListener tfListener_;
    Cache cache;

public:
    SignTransform(): targetFrameId_("/map"), objFramePrefix_("object"){
        ros::NodeHandle pnh("~");
        pnh.param("target_frame_id", targetFrameId_, targetFrameId_);
        pnh.param("object_prefix", objFramePrefix_, objFramePrefix_);

        ros::NodeHandle nh;
        subs_ = nh.subscribe("objectsStamped", 1, &SignTransform::objectsDetectedCallback, this);
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

                tf::StampedTransform pose;
                tf::StampedTransform poseCam;
                try{
                    // Get transformation from "object_#" frame to target frame
                    // The timestamp matches the one sent over TF
//                    tfListener_.lookupTransform(targetFrameId_, ros::Time::now(), objectFrameId, ros::Time(0), targetFrameId_,pose);
//                    tfListener_.lookupTransform(msg->header.frame_id, ros::Time::now(), objectFrameId, ros::Time(0), msg->header.frame_id, poseCam);

                    tfListener_.waitForTransform(targetFrameId_, objectFrameId, msg->header.stamp, ros::Duration(1.0));
                    tfListener_.lookupTransform(targetFrameId_, objectFrameId, msg->header.stamp, pose);
                }
                catch(tf::TransformException & ex){
                    ROS_WARN("%s",ex.what());
                    continue;
                }

                Obstacle obstacle(pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z());
                if(cache.addElement(obstacle)){
                    ROS_INFO("New element has been added to the Cache");
                } else{
                    ROS_INFO("Element cannot be added, because it is already in the Cache");
                }
                // Here "pose" is the position of the object "id" in target frame.
                double yaw = tf::getYaw(pose.getRotation());
                ROS_INFO("%f", yaw);
                ROS_INFO("Object_%d [x,y,z] [x,y,z,w] in \"%s\" frame: [%f,%f,%f] [%f,%f,%f,%f]",
                         id, targetFrameId_.c_str(),
                         pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z(),
                         pose.getRotation().x(), pose.getRotation().y(), pose.getRotation().z(), pose.getRotation().w());
//                ROS_INFO("Object_%d [x,y,z] [x,y,z,w] in \"%s\" frame: [%f,%f,%f] [%f,%f,%f,%f]",
//                         id, msg->header.frame_id.c_str(),
//                         poseCam.getOrigin().x(), poseCam.getOrigin().y(), poseCam.getOrigin().z(),
//                         poseCam.getRotation().x(), poseCam.getRotation().y(), poseCam.getRotation().z(), poseCam.getRotation().w());

            }
        }
    }
};

int main(int argc, char * argv[]){
    ros::init(argc, argv, "sign_tf_node");

    SignTransform sync;
    ros::spin();
}