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
    double sx;   //sigma x
    double sx2;  //sigma x^2
    double sx3;  //sigma x^3
    double sx4;  //sigma x^4
    double sy;   //sigma y
    double sxy;  //sigma x*y
    double sx2y; //sigma x^2*y
    
public:
    unsigned int x;  //time
    unsigned long long start_time;
    double a,b,c;
    LSM();
    void tick();
    void process(int y);
    int predict();
    int predict(int x);
};

#endif /* defined(__OrbitYosoku__lsm__) */
