#ifndef SRC_AUXILIARY_FUNC_H
#define SRC_AUXILIARY_FUNC_H

#include <ctime>
#include <fstream>


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

}

#endif //SRC_AUXILIARY_FUNC_H
