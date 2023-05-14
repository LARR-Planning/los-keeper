//
// Created by larr-planning on 23. 5. 11.
//
#include "los_keeper_simulator/los_keeper_simulator_math_utils.h"
float interpolate(const vector<float> &xData, const vector<float> &yData, const float &x) {
    int size = (int)xData.size();
    if(xData.size()<2){
        return yData[0];
    }
    if(x<xData[0])
        return yData[0];
    if(x>xData[size-1])
        return yData.back();
    float xL= xData[0], yL = yData[0], xR = xData.back(), yR = yData.back();
    for(int i =0;i<size-2;i++){
        if(xData[i]<=x and x<xData[i+1]){
            xL = xData[i]; yL = yData[i];
            xR = xData[i+1], yR = yData[i+1];
            break;
        }
    }
    float dydx = (yR-yL)/(xR-xL);
    return yL + dydx * (x-xL);
}

