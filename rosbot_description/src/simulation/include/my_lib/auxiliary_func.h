#ifndef SRC_AUXILIARY_FUNC_H
#define SRC_AUXILIARY_FUNC_H

#include <ctime>
#include <fstream>
#include <math.h>
#include <ros/console.h>


namespace auxiliary_func {

    const std::string LOG_RELATIVE_ADDRESS = "../../../src/rosbot_description/src/simulation/scripts/scheduler/log.txt";
    const std::string SOUND_RELATIVE_ADDRESS = "../../../src/rosbot_description/src/simulation/include/my_lib/sound.sh";

    int writeToLogFile(std::string data){
        std::ofstream file;
        file.open(LOG_RELATIVE_ADDRESS, std::ofstream::app);
        file << data << "\n";
        file.close();
        return 0;
    }


    long getCurrentDatetimeInSeconds(){
        time_t current = time(nullptr);
        return static_cast<long> (current);
    }

    std::string secondsToDatetime(long int seconds){
        return  asctime(localtime(&seconds));
    }

    bool makeSound(){
        std::string cmd = "bash " + SOUND_RELATIVE_ADDRESS;
        system(cmd.c_str());
        return true;
    }

    double roundToNearest(float value, int precision){
        int desiredPrecision = (int) pow(10, precision);
        return (round(value * desiredPrecision)) / desiredPrecision;
    }

    std::string str(const char array[]){
        return std::string(array);
    }

    /* Method allows to change a Logger level of a single node */
    void changeNodeLoggerLevel(ros::console::levels::Level level){
        if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, level)) {
            ros::console::notifyLoggerLevelsChanged();
        }
    }

}

#endif //SRC_AUXILIARY_FUNC_H
