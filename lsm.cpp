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

    st3 = 0;
    st4 = 0;
    sy = 0;
    sty = 0;
    st2y = 0;
    z = 0;
}
void LSM::process(cv::Point3f datapoint)
{
    cout << "parabolic: " << datapoint << endl;
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
    st3 += t*t*t;
    st4 += t*t*t*t;
    sy += datapoint.y;
    sty += t*datapoint.y;
    st2y += t*t*datapoint.y;
    printf("t:%u\n",t);
    printf("x:%f\n", datapoint.x);
    printf("y:%f\n", datapoint.y);
    printf("s1:%f\n", s1);
    printf("st:%f\n", st);
    printf("st2:%f\n", st2);
    printf("stx:%f\n", stx);
    printf("sx:%f\n", sx);
    printf("st3:%f\n", st3);
    printf("st4:%f\n", st4);
    printf("sy:%f\n", sy);
    printf("sty:%f\n", sty);
    printf("st2y:%f\n", st2y);
    
    if ((st3*st3-st2*st4)!=0 && (st2)!=0) {
        a = ((double)(stx-st*b)/(st2));
        c = ((double)(sty*st3-st2y*st2+(st2*st2-st*st3)*e))/(st3*st3-st2*st4);
        d = ((double)(-st2y*st3-st4*sty+(st*st4-st2*st3)*e))/(st3*st3-st2*st4);
        printf("x = %f t+ %f\n", a, b);
        printf("y = %f t^2+ %f t+ %f\n", c, d, e);
        printf("z = %f\n", z);
    } else {
        cout << "zero divide" << endl;
    }
    cout << "x-(at+b)" << datapoint.x - a*t+b << endl;
    cout << "y-(ct^2+dt+e)" << datapoint.y - c*t*t - d*t - e << endl;
    
    return;
}
cv::Point3f LSM::predict()
{
    cv::Point3f dest;
    int t_ = (unsigned int)((cv::getTickCount() - start_time)*f); //ms
    dest.x = a * t_ + b;
    dest.y = c*t_*t_ + d*t_ + e;
    dest.z = z;
    return dest;
}

cv::Point3f LSM::predict(int t_)
{
    cv::Point3f dest;
    dest.x = a * t_ + b;
    dest.y = c*t_*t_ + d*t_ + e;
    dest.z = z;
    return dest;
}