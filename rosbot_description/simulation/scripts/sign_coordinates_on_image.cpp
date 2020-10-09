#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
#include <opencv2/opencv.hpp>

#define STOP_SIGN 1

int id = 0;
ros::Publisher action_pub;
std_msgs::String msg;

void objectCallback(const std_msgs::Float32MultiArrayPtr &object){
    if (object->data.size() > 0)
    {
        id = object->data[0];
        float objectWidth = object->data[1];
        float objectHeight = object->data[2];
        float x_pos;
        float y_pos;

        // Find corners OpenCV
        cv::Mat cvHomography(3, 3, CV_32F);
        std::vector<cv::Point2f> inPts, outPts;

        switch (id){
            case STOP_SIGN:
                msg.data = "Stop sign found";

                cvHomography.at<float>(0, 0) = object->data[3];
                cvHomography.at<float>(1, 0) = object->data[4];
                cvHomography.at<float>(2, 0) = object->data[5];
                cvHomography.at<float>(0, 1) = object->data[6];
                cvHomography.at<float>(1, 1) = object->data[7];
                cvHomography.at<float>(2, 1) = object->data[8];
                cvHomography.at<float>(0, 2) = object->data[9];
                cvHomography.at<float>(1, 2) = object->data[10];
                cvHomography.at<float>(2, 2) = object->data[11];

                inPts.push_back(cv::Point2f(0, 0));
                inPts.push_back(cv::Point2f(objectWidth, 0));
                inPts.push_back(cv::Point2f(0, objectHeight));
                inPts.push_back(cv::Point2f(objectWidth, objectHeight));
                cv::perspectiveTransform(inPts, outPts, cvHomography);

                x_pos = (int)(outPts.at(0).x + outPts.at(1).x + outPts.at(2).x +
                              outPts.at(3).x) / 4;
                y_pos = (int)(outPts.at(0).y + outPts.at(1).y + outPts.at(2).y +
                             outPts.at(3).y) / 4;
                ROS_INFO("x=%.2f y=%.2f", x_pos, y_pos);
                break;
            default: // other object
                msg.data = "Unknown object detected";
        }
    }
    else
    {
        // No object detected
        msg.data = "No object detected";
    }
    action_pub.publish(msg);
    ROS_INFO(msg.data.c_str());
}

int main(int argc, char **argv){

    ros::init(argc, argv, "sign_coordinates_node");
    ros::NodeHandle n("~");
    ros::Rate loop_rate(50);
    ros::Subscriber sub = n.subscribe("/objects", 1, objectCallback);
    action_pub = n.advertise<std_msgs::String>("/sign_coordinates", 1);
    msg.data = "";
    while (ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }
}
