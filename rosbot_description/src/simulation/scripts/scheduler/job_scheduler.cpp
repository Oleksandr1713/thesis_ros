#include <ctime>
#include <regex>
#include <fstream>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "mongodb_store/message_store.h"
#include "database/db_proxy_decorator.h"
#include "my_lib/auxiliary_func.h"
#include "simulation/ScheduleJobMsg.h"
#include "simulation/JobBriefInfo.h"

using namespace std;
using namespace mongodb_store;
using namespace mongodb_proxy_decorator;
using namespace auxiliary_func;
using namespace simulation;

class JobScheduler{

private:
    MessageStoreProxy messageStore;

public:
    JobScheduler(ros::NodeHandle& nodeHandle, const string& collectionName) : messageStore(nodeHandle, collectionName){};

    ~JobScheduler()= default;

    bool callback(ScheduleJobMsg::Request& request, ScheduleJobMsg::Response& response){
        bool success = false;
        DatabaseEntry dbEntry;
        dbEntry.sign_id = request.sign_id;
        dbEntry.time_start = request.time_start;    // time in seconds since Unix epoch
        dbEntry.time_end = calculateUpperTimeOfSignValidity(request.sign_id, request.time_start);  // time in seconds since Unix epoch
        dbEntry.x_coordinate = request.x_coordinate;
        dbEntry.y_coordinate = request.y_coordinate;

        string entry_id = insertNewEntry(messageStore, dbEntry);
        response.entry_id = entry_id;

        JobBriefInfo jobInfo;
        jobInfo.entry_id = entry_id;
        long job_id = scheduleJob(dbEntry.time_end, jobInfo);

        if(job_id != 0){
            dbEntry.id = entry_id;
            dbEntry.job_id = job_id;
            success = updateEntry(messageStore, entry_id, dbEntry);
            response.job_id = job_id;
        }

        return success;
    }

private:
    long calculateUpperTimeOfSignValidity(long& sign_id, long& sign_time_start){
        /*This method must be further extended to consider information about
         * different sign types and their different time validities. */
        long current = getCurrentDatetimeInSeconds();
        return current + 120;
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

        return result;
    }

    long scheduleJob(long& scheduledTime, JobBriefInfo& msg){
        /* this function schedules the execution of the ros command, which publishes
         * the message to the ros topic. */
        long job_id = 0;
        string cmd = R"(echo "rostopic pub -1 /catcher simulation/JobBriefInfo -- \"')";
        cmd.append(msg.entry_id);
        cmd.append(R"('\"")");
        cmd.append(" | ");
        cmd.append("at -t " + getDateTimeForAT(scheduledTime));
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

    void removeJob(long& job_id){
        string cmd = "atrm " + to_string(job_id);
        system(cmd.c_str());

        writeToLogFile(cmd); // this line is needed only for a debug purpose
    }

};


/*
    method below is needed only for debugging
*/
string getDateTimeForAT_Test(long int& seconds){
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

/*
    method below is needed only for debugging
*/
long int scheduleJob_Test(long int scheduledTime){
    long int job_id = 0;
    string cmd = R"(echo "rostopic pub -1 /catcher simulation/JobBriefInfo -- \"')";
    cmd.append(to_string(111+2));
    cmd.append(R"('\"")");
//    string cmd = "echo 'rostopic pub -1 /catcher simulation/JobBriefInfo -- \"\'77\'\"'";

//    string cmd = R"(echo "spd-say 'bip bip bip'")";
    cmd.append(" | ");
    cmd.append("at -t " + getDateTimeForAT_Test(scheduledTime));
    // line below must point to the script that sends a message to a specific ros topic
//    cmd.append(" -f ~/Documents/thesis/rosbot_ws/src/rosbot_description/src/simulation/scripts/scheduler/sound.sh");
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


int main(int argc, char * argv[]) {
    ros::init(argc, argv, "job_scheduler_node");
    ros::NodeHandle nh("~");

    JobScheduler scheduler(nh, COLLECTION_NAME);
    ros::ServiceServer service = nh.advertiseService("scheduler_service", &JobScheduler::callback, &scheduler);

    ROS_INFO("Scheduler service is up.");
    ros::spin();

//    long int currentTime = getCurrentDatetimeInSeconds();
//////    secondsToDatetime(currentTime);
////    long int a = scheduleJob(currentTime);
////    removeJob(a);
//    currentTime+=0;
//    long int a = scheduleJob_Test(currentTime);
//    getDateTimeForAT_Test(currentTime);

//    string a = scheduleJob();
//    cout << a;


//    //Get the time and store it in the time variable.
//    ros::Time time = ros::Time::now();
//    std::cout << ros::Time::init();
}