#include <ctime>
#include <iostream>
#include <regex>
#include <fstream>

#include "ros/ros.h"
#include "std_msgs/String.h"

using namespace std;

int writeToLogFile(string data){
    ofstream file;
    file.open("../../../src/rosbot_description/src/simulation/scripts/log.txt", std::ofstream::app);
    file << data << "\n";
    file.close();
    return 0;
}

string getDateTimeForAT(long int& seconds){
    /* This function represents date and time in [[CC]YY]MMDDhhmm[.ss] format,
     that is used by AT command-line utility */

    char buffer[128];

    int year = localtime(&seconds)->tm_year + 1900;
    int month = localtime(&seconds)->tm_mon + 1;
    int day = localtime(&seconds)->tm_mday;
    int hour = localtime(&seconds)->tm_hour;
    int min = localtime(&seconds)->tm_min;
    int sec = localtime(&seconds)->tm_sec;

    sprintf(buffer, "%d%02d%02d%02d%02d.%02d", year, month, day, hour, min, sec);
    string result(buffer);

    cout << result << "\n";
    return result;
}

long int scheduleJob(long int scheduledTime){
    long int job_id = 0;
    string cmd = "at -t " + getDateTimeForAT(scheduledTime);
    // line below must point to the script that sends a message to a specific ros topic
    cmd.append(" -f ~/Documents/thesis/rosbot_ws/src/rosbot_description/src/simulation/scripts/sound.sh");
    cmd.append(" 2>&1");

    string cmd_output;
    const int max_buffer = 256;
    char buffer[max_buffer];

    FILE* stream = popen(cmd.c_str(), "r");
    if (stream) {
        while (!feof(stream)){
            if (fgets(buffer, max_buffer, stream) != nullptr){
                cmd_output.append(buffer);
            }
        }
    }
    pclose(stream);

    writeToLogFile(cmd_output); // this line is needed only for a debug purpose

    regex regexp("job ([0-9]+) at");
    smatch match;
    if (regex_search(cmd_output, match, regexp)) {
        job_id = stol(match.str(1), nullptr, 10);
    }

    return job_id;
}

void removeJob(long int& job_id){
    string cmd = "atrm " + to_string(job_id);
    system(cmd.c_str());

    writeToLogFile(cmd); // this line is needed only for a debug purpose
//    cout << cmd;
}

long int getCurrentDatetimeInSeconds(){
    time_t current = time(nullptr);
    return static_cast<long int> (current);
}

void secondsToDatetime(long int seconds){
    cout << asctime(localtime(&seconds));
}

void schedulerCallback(const std_msgs::String::ConstPtr& msg){
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}


int main(int argc, char * argv[]) {
    long int currentTime = getCurrentDatetimeInSeconds();
////    secondsToDatetime(currentTime);
//    long int a = scheduleJob(currentTime);
//    removeJob(a);
    currentTime+=60;
    long int a = scheduleJob(currentTime);
    getDateTimeForAT(currentTime);

//    string a = scheduleJob();
//    cout << a;


//    //Get the time and store it in the time variable.
//    ros::Time time = ros::Time::now();
//    std::cout << ros::Time::init();


//    ros::init(argc, argv, "scheduler");
//    ros::NodeHandle n;
//    ros::Subscriber sub = n.subscribe("job_scheduler", 1, schedulerCallback);
//    ros::Time ros_time;
//    while (ros::ok())
//    {
//        ros_time = ros::Time::now();
//        ROS_INFO("%d", ros_time.sec);
//        ros::spinOnce();
//    }
}