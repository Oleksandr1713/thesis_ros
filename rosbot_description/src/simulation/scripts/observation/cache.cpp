#include <iostream>
#include "cache/cache.h"

int main(int argc, char** argv){
    Cache cache;

    char answer;
    while(answer!='q') {
        std::cout << "CACHE MENU" << std::endl;
        std::cout << "==========" << std::endl;
        std::cout << "Enter <1> to Display the Cache" << std::endl;
        std::cout << "Enter <2> to Add Obstacle to the Cache" << std::endl;
        std::cout << "Enter <3> to Remove Obstacle from the Cache" << std::endl;
        std::cout << "Enter <4> to Show a size of the Cache" << std::endl;
        std::cout << "Enter <q> to Quit" << std::endl;
        std::cin >> answer;
        switch(answer) {
            case '1': {
                std::cout << "Case 1" << std::endl;
                std::cout << "Current Cache Status" << std::endl;
                std::cout << "====================" << std::endl;
                cache.display();
                std::cout << "====================" << std::endl;
                break;
            }
            case '2':{
                std::cout << "Case 2" << std::endl;
                Obstacle obstacle(1, 2, 3);
                std::cout << "Please enter x value: ";
                std::cin >> obstacle.x;
                std::cout << "Please enter y value: ";
                std::cin >> obstacle.y;
                std::cout << "Please enter z value: ";
                std::cin >> obstacle.z;
                if(cache.addElement(obstacle)){
                    std::cout << "New element has been added" << std::endl;
                    std::cout << "==========================" << std::endl;

                } else{
                    std::cout << "Element cannot be added, because it is already in the Cache" << std::endl;
                    std::cout << "===========================================================" << std::endl;
                }
                break;
            }
            case '3': {
                std::cout << "Case 3" << std::endl;
                int index;
                std::cout << "Please enter the index of the element to be removed: ";
                std::cin >> index;
                if(cache.removeElement(index)){
                    std::cout << "Element has been removed" << std::endl;
                    std::cout << "========================" << std::endl;
                } else{
                    std::cout << "Error! Index outside of the Cache size" << std::endl;
                    std::cout << "======================================" << std::endl;
                }
                break;
            }
            case '4': {
                std::cout << "Case 4" << std::endl;
                std::cout << "Cache Size: " << cache.size() << std::endl;
                std::cout << "==========" << std::endl;
                break;
            }
            case 'q': {
                std::cout << "Case Quit" << std::endl;
                break;
            }
            default: {
                std::cout << "Default Case" << std::endl;
                break;
            }
        }
    }

    return 0;
}