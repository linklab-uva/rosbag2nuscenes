#include "rosbag2nuscenes/Bag2Scenes.hpp"
#include <iostream>


int main(int argc, char** argv) {
    if (argc != 4) {
        printf("Expected: ./rosbag2nuscenes {rosbag directory} {parameter file} {num workers}\n");
        printf("Got: ");
        for (int i = 0; i < argc; i++) {
            printf("%s ", argv[i]);
        }
        printf("\n");
        exit(1);
    }
    Bag2Scenes converter(argv[1], argv[2], (int) *argv[3]);
    converter.writeScene();
}