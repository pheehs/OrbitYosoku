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

LSM::LSM():a(0),b(0),c(0),d(0),e(0)
{
    tick();
}
void LSM::tick()
{
    // 物体が見つからなかったとき
    t = (unsigned int)-1;
    s1 = 0;
    st = 0;
    st2 = 0;
    stx = 0;
    sx = 0;
    sx2 = 0;
    sx3 = 0;
    sx4 = 0;
    sy = 0;
    sxy = 0;
    sx2y = 0;
    z = 0;
}
void LSM::process(cv::Point3f datapoint)
{
    // 物体が見つかったとき
    if (t == (unsigned int)-1) {
        //１回目
        cout << "------------------<LSM>object found!--------------------------------" << endl;
        start_time = cv::getTickCount();
        t = 0;
        a = 0;
        b = datapoint.x;
        c = 0;
        d = 0;
        e = datapoint.y;
        z = datapoint.z;
        return;
    }
    //２回目以降
    t = (unsigned int)((cv::getTickCount() - start_time)*f); //ms
    
    s1++;
    st += t;
    st2 += t*t;
    stx += t*datapoint.x;
    sx += datapoint.x;
    sx2 += datapoint.x*datapoint.x;
    sx3 += datapoint.x*datapoint.x*datapoint.x;
    sx4 += datapoint.x*datapoint.x*datapoint.x*datapoint.x;
    sy += datapoint.y;
    sxy += datapoint.x*datapoint.y;
    sx2y += datapoint.x*datapoint.x*datapoint.y;
    //printf("t:%u\n",t);
    //printf("x:%u\n", x);
    //printf("s1:%llu\n", s1);
    //printf("st:%llu\n", st);
    //printf("st2:%llu\n", st2);
    //printf("stx:%llu\n", stx);
    //printf("sx:%llu\n", sx);
    //printf("sx2:%llu\n", sx2);
    //printf("sx3:%llu\n", sx3);
    //printf("sx4:%llu\n", sx4);
    //printf("sy:%lld\n", sy);
    //printf("sxy:%lld\n", sxy);
    //printf("sx2y:%lld\n", sx2y);
    
    if ((sx3*sx3-sx2*sx4)!=0 && (stx*st-sx*st2)!=0) {
        a = ((double)(st*sx-stx*s1)*b)/(stx*st-sx*st2);
        c = ((double)(sxy*sx3-sx2y*sx2+(sx2*sx2-sx*sx3)*c))/(sx3*sx3-sx2*sx4);
        d = ((double)(sx2y*sx3-sx4*sxy+(sx*sx4-sx2*sx3)*c))/(sx3*sx3-sx2*sx4);
    } else {
        cout << "zero divide" << endl;
    }
    
    return;
}
cv::Point3f LSM::predict()
{
    cv::Point3f dest;
    int t_ = (unsigned int)((cv::getTickCount() - start_time)*f); //ms
    dest.x = a * t_ + b;
    dest.y = c*dest.x*dest.x + d*dest.x + e;
    dest.z = z;
    return dest;
}

cv::Point3f LSM::predict(int t_)
{
    cv::Point3f dest;
    dest.x = a * t_ + b;
    dest.y = c*dest.x*dest.x + d*dest.x + e;
    dest.z = z;
    return dest;
}