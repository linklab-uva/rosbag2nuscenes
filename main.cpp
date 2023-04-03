#include "rosbag2nuscenes/Bag2Scenes.hpp"
#include <iostream>


int main(int argc, char** argv) {
    if (argc != 3) {
        printf("Expected: ./rosbag2nuscenes {rosbag directory} {parameter file}\n");
        printf("Got: ");
        for (int i = 0; i < argc; i++) {
            printf("%s ", argv[i]);
        }
        printf("\n");
    }
    Bag2Scenes converter(argv[1], argv[2]);
    converter.writeScene();
}