#ifndef SRC_CACHE_H
#define SRC_CACHE_H

#include <vector>
#include <boost/range/adaptor/indexed.hpp>
#include <iostream>
#include "cache/obstacle.h"

using namespace boost::adaptors;

class Cache {

private:
    std::vector<Obstacle> data;

    int findElement(Obstacle& obstacle){
        int index = -1;
        for (auto const& element : data | indexed(0)) {
            if(element.value().checkObstaclesSimilarity(obstacle)){
                index = element.index();
                break;
            }
        }
        return index;
    }

public:
    Cache(){
        std::cout << "Cache Has Been Initiated" << std::endl;
    };

    ~Cache(){
        data.clear();
        std::cout << "Cache Has Been Emptied" << std::endl;
    };

    bool addElement(Obstacle& obstacle){
        int index = findElement(obstacle);
        if(index == -1){
            data.push_back(obstacle);
            return true;
        } else{
            return false;
        }
    }

    bool removeElement(int index){
        if(index >= 0 && index < data.size()){
            data.erase(data.begin() + index);
            return true;
        } else {
            return false;
        }

    }

    void display(){
        for(int i=0; i<data.size(); i++){
            std::cout << "Element " << i << ": ";
            std::cout << "( " << data[i].x << ", ";
            std::cout << data[i].y << ", ";
            std::cout << data[i].z << " )" << std::endl;
        }
    }

    int size(){
        return data.size();
    }
};

#endif //SRC_CACHE_H
