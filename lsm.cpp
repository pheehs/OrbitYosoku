//
//  lsm.cpp
//  OrbitYosoku
//
//  Created by pheehs on 13/08/23.
//  Copyright (c) 2013年 Kph-lab. All rights reserved.
// This code is for Least Square Method

#include <iostream>
#include <opencv2/core/core.hpp>
#include "lsm.h"

using namespace std;

const double f = (1000 /cv::getTickFrequency());

LSM::LSM():a(0),b(0),c(0)
{
    tick();
}
void LSM::tick()
{
    // 物体が見つからなかったとき
    x = (unsigned int)-1;
    s1 = 0;
    sx = 0;
    sx2 = 0;
    sx3 = 0;
    sx4 = 0;
    sy = 0;
    sxy = 0;
    sx2y = 0;
}
void LSM::process(int y)
{
    // 物体が見つかったとき
    if (x == (unsigned int)-1) {
        //１回目
        cout << "-----------------------------------------------------" << endl;
        start_time = cv::getTickCount();
        x = 0;
        a = 0;
        b = 0;
        c = y;
        return;
    }
    //２回目以降
    x = (unsigned int)((cv::getTickCount() - start_time)*f); //ms
    
    s1++;
    sx += x;
    sx2 += x*x;
    sx3 += x*x*x;
    sx4 += x*x*x*x;
    sy += y;
    sxy += x*y;
    sx2y += x*x*y;
    //printf("x:%u\n", x);
    //printf("s1:%llu\n", s1);
    //printf("sx:%llu\n", sx);
    //printf("sx2:%llu\n", sx2);
    //printf("sx3:%llu\n", sx3);
    //printf("sx4:%llu\n", sx4);
    //printf("sy:%lld\n", sy);
    //printf("sxy:%lld\n", sxy);
    //printf("sx2y:%lld\n", sx2y);
    
    if ((sx3*sx3-sx2*sx4)!=0) {
        a = ((double)(sxy*sx3-sx2y*sx2+(sx2*sx2-sx*sx3)*c))/(sx3*sx3-sx2*sx4);
        b = ((double)(sx2y*sx3-sx4*sxy+(sx*sx4-sx2*sx3)*c))/(sx3*sx3-sx2*sx4);
    } else {
        cout << "zero divide" << endl;
    }
    /*
     if ((2*sx*sx2*sx3+s1*sx2*sx4-sx*sx*sx4-s1*sx3*sx3-sx2*sx2*sx2)!=0 && (2*sx*sx2*sx3+s1*sx2*sx4-sx*sx*sx4-s1*sx3*sx3-sx2*sx2*sx2)!=0 && (2*sx*sx2*sx3+s1*sx2*sx4-sx*sx*sx4-s1*sx3*sx3-sx2*sx2*sx2)!=0) {
        a = ((double)(s1*sx2*sx2y-sx*sx*sx2y+sx*sx2*sxy-s1*sx3*sxy+sx*sx3*sy-sx2*sx2*sy))/(2*sx*sx2*sx3+s1*sx2*sx4-sx*sx*sx4-s1*sx3*sx3-sx2*sx2*sx2);
        b = ((double)(sx*sx2*sx2y-s1*sx3*sx2y+s1*sx4*sxy-sx2*sx2*sxy+sx2*sx3*sy-sx*sx4*sy))/(2*sx*sx2*sx3+s1*sx2*sx4-sx*sx*sx4-s1*sx3*sx3-sx2*sx2*sx2);
        c = ((double)(-sx2*sx2*sx2y+sx*sx3*sx2y-sx*sx4*sxy+sx2*sx3*sxy-sx3*sx3*sy+sx2*sx4*sy))/(2*sx*sx2*sx3+s1*sx2*sx4-sx*sx*sx4-s1*sx3*sx3-sx2*sx2*sx2);
    }else {
        cout << "zero divide" << endl;
    }
     */
    
    return;
}
int LSM::predict()
{
    int x_ = (unsigned int)((cv::getTickCount() - start_time)*f); //ms
    return a * x_*x_ + b * x_ + c;
}

int LSM::predict(int x_)
{
    return a * x_*x_ + b * x_ + c;
}