//
// Created by redwan on 4/28/22.
//
#include <iostream>
#include "controller.h"
using namespace std;


int main(int argc, char *argv[])
{
    cout << "diff controller test" << endl;

    controller controller({8.32, 13.75, 0.318}, 9, 15, 3, 0.01);
    controller.set_points(5.256, 11.346);


    do {
        auto cmd_vel = controller.compute_control();
        cout << cmd_vel.first << "," << cmd_vel.second<< endl;
    }while(!controller.isFinished());

    return 0;
}
