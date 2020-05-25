#include <ctime>
#include <iostream>
#include <ros/ros.h>
#include <regex>
#include <fstream>

using namespace std;

int writeToLogFile(string data){
    ofstream file;
    file.open("../../../src/rosbot_description/src/simulation/scripts/log.txt");
    file << data;
    file.close();
    return 0;
}

long int scheduleJob(long int scheduledTime){
    long int job_id = 0;
    string cmd = "at now +1 minutes";
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
//    cout << cmd;
}

long int getCurrentDatetimeInSeconds(){
    time_t current = time(nullptr);
    return static_cast<long int> (current);
}

void secondsToDatetime(long int seconds){
    cout << asctime(localtime(&seconds));
}

int main(int argc, char * argv[]) {
    long int currentTime = getCurrentDatetimeInSeconds();
//    secondsToDatetime(currentTime);
    long int a = scheduleJob(currentTime);
    removeJob(a);

//    string a = scheduleJob();
//    cout << a;
//    std::time_t t = std::time(nullptr);   // get time now
//    std::cout << t;
//    std::tm* now = std::localtime(&t);
//    std::cout << (now->tm_year + 1900) << '-'
//              << (now->tm_mon + 1) << '-'
//              <<  now->tm_mday << ' '
//              <<  now->tm_hour << ':'
//              <<  now->tm_min
//              << "\n";


//    //Get the time and store it in the time variable.
//    ros::Time time = ros::Time::now();
//    std::cout << ros::Time::init();


//    ros::init(argc, argv, "scheduler");
//    ros::NodeHandle n;
//    ros::Time ros_time;
//    while (ros::ok())
//    {
//        ros_time = ros::Time::now();
//        ROS_INFO("%d", ros_time.sec);
//        ros::spinOnce();
//    }
}