#include <ctime>
#include <regex>
#include <fstream>

#include "ros/ros.h"
#include "mongodb_store/message_store.h"
#include "database/db_proxy_decorator.h"
#include "constants/node_constants.h"
#include "constants/obstacle_constants.h"
#include "my_lib/auxiliary_func.h"
#include "simulation/ScheduleJobMsg.h"
#include "simulation/JobBriefInfo.h"

using namespace std;
using namespace mongodb_store;
using namespace mongodb_proxy_decorator;
using namespace auxiliary_func;
using namespace simulation;

/* ROS node name*/
constexpr static const char* NODE_NAME = "job_scheduler_node";

class JobScheduler{

private:
    MessageStoreProxy messageStore;
    ros::ServiceServer srv_schedule_job;

public:
    JobScheduler(ros::NodeHandle& nh_base) : messageStore(nh_base, str(node_constants::COLLECTION_NAME)){
        srv_schedule_job = nh_base.advertiseService(str(node_constants::ADV_JOB_SCHEDULER), &JobScheduler::callback, this);
        ROS_INFO("Scheduler service is up.");
    };

    ~JobScheduler(){
        ROS_INFO("Scheduler service is destroyed.");
    };

    bool callback(ScheduleJobMsg::Request& request, ScheduleJobMsg::Response& response){
        bool success = false;

        ROS_DEBUG("JC_1");
        DatabaseEntryUpdate dbEntry;
        dbEntry.sign_id = request.sign_id;
        dbEntry.time_start = getCurrentDatetimeInSeconds();                                                // time in seconds since Unix epoch
        dbEntry.time_end = calculateUpperTimeOfSignValidity(request.sign_id, dbEntry.time_start);   // time in seconds since Unix epoch

        ROS_DEBUG("JC_2");
        JobBriefInfo jobInfo;
        jobInfo.entry_id = request.entry_id;
        long job_id = scheduleJob(dbEntry.time_end, jobInfo);

        ROS_DEBUG("JC_3");
        if(job_id != 0){
            ROS_DEBUG("JC_4");
            dbEntry.job_id = job_id;
            success = updateEntry(messageStore, request.entry_id, dbEntry);
            response.job_id = job_id;
        }
        ROS_DEBUG("JC_5");
        response.success = success;

        return success;
    }

private:
    long calculateUpperTimeOfSignValidity(long& sign_id, long& sign_time_start){
        long current = getCurrentDatetimeInSeconds();
        switch(sign_id) {
            case obstacle_constants::STOP_SIGN_ID:
                return current + obstacle_constants::STOP_SIGN_LIFESPAN;
            case obstacle_constants::BLOCK_SIGN_ID:
                return current + obstacle_constants::BLOCK_SIGN_LIFESPAN;
            case obstacle_constants::CLOSED_SIGN_ID:
                return current + obstacle_constants::CLOSED_SIGN_ID;
            default:
                return current + obstacle_constants::STOP_SIGN_LIFESPAN;
        }
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
        ROS_DEBUG("JC_SJ_1");
        string cmd = R"(echo "rostopic pub -1 /simulation/catcher simulation/JobBriefInfo -- \"')";
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

        ROS_DEBUG("JC_SJ_2");
        writeToLogFile(cmd_output); // this line is needed only for a debug purpose

        ROS_DEBUG("JC_SJ_3");
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


/* Method below is needed only for debugging */
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

/* Method below is needed only for debugging */
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
    changeNodeLoggerLevel(ros::console::levels::Debug);
    ros::init(argc, argv, str(NODE_NAME));
    ros::NodeHandle nh_base;
    JobScheduler scheduler(nh_base);
    ros::spin();
    return 0;

    /*long int currentTime = getCurrentDatetimeInSeconds();
    long int a = scheduleJob_Test(currentTime);
    getDateTimeForAT_Test(currentTime);*/
}