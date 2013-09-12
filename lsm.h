//
//  lsm.h
//  OrbitYosoku
//
//  Created by pheehs on 13/08/23.
//  Copyright (c) 2013年 Kph-lab. All rights reserved.
//

#ifndef __OrbitYosoku__lsm__
#define __OrbitYosoku__lsm__

class LSM
{
private:
    //unsigned long long
    double s1;   //sigma 1
    double st;   //sigma t
    double st2;  //sigma t^2
    double stx;  //sigma t*x
    double sx;   //sigma x

    double st3;  //sigma t^3
    double st4;  //sigma t^4
    double sy;   //sigma y
    double sty;  //sigma t*y
    double st2y; //sigma t^2*y
public:
    unsigned int t;  //time
    float z;
    unsigned long long start_time;
    double a,b,c,d,e;
    LSM();
    void tick();
    void process(cv::Point3f datapoint);
    cv::Point3f predict();
    cv::Point3f predict(int t);
};

#endif /* defined(__OrbitYosoku__lsm__) */
