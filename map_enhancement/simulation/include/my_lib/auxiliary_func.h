#ifndef SRC_AUXILIARY_FUNC_H
#define SRC_AUXILIARY_FUNC_H

#include <ctime>
#include <fstream>
#include <math.h>
#include <ros/console.h>
#include <unistd.h>


namespace auxiliary_func {

    const std::string LOG_ABSOLUTE_PATH = "/home/oleksandr/Documents/thesis/rosbot_ws/src/map_enhancement/simulation/scripts/scheduler/log.txt";
    const std::string SOUND_ABSOLUTE_PATH = "/home/oleksandr/Documents/thesis/rosbot_ws/src/map_enhancement/simulation/include/my_lib/sound.sh";

    /* This function allows to determine a default absolute address to a folder, where
     * a compiled executable file, which invokes this function, is located */
    char* getDirectoryAddressOfCurrentExecutable(){
        return get_current_dir_name();
    }

    /* If you want to use this function, a proper absolute path to your desired log file must be set.
     * Therefore, adjust the address above according to your OS configuration */
    int writeToLogFile(std::string data){
        std::ofstream file;
        file.open(LOG_ABSOLUTE_PATH, std::ofstream::app);
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

    /* If you want to use this function, a proper absolute path to your desired bash file must be set.
     * Therefore, adjust the address above according to your OS configuration */
    bool makeSound(){
        std::string cmd = "bash " + SOUND_ABSOLUTE_PATH;
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

    double getDegrees(double radians){
        double degrees = radians * 180/M_PI;
        while(degrees < 0){
            degrees = 360 + degrees;
        }
        while(degrees > 360){
            degrees = degrees - 360;
        }
        return degrees;
    }

    double getRadians(double degrees){
        return degrees * M_PI/180;
    }

}

#endif //SRC_AUXILIARY_FUNC_H
