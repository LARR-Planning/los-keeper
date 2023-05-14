#ifndef LOS_KEEPER_SIMULATOR_UTILS_H
#define LOS_KEEPER_SIMULATOR_UTILS_H
#include <vector>
using namespace std;
struct ControlInput{
    float jx;
    float jy;
    float jz;
};
struct State{
    float px;
    float py;
    float pz;
    float vx;
    float vy;
    float vz;
    float ax;
    float ay;
    float az;
};
struct ObjectState{
    vector<float> t;
    vector<float> px;
    vector<float> py;
    vector<float> pz;
    vector<float> vx;
    vector<float> vy;
    vector<float> vz;
};

#endif