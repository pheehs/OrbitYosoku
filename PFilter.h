//
//  PFilter.h
//  OrbitYosoku
//
//  Created by pheehs on 13/05/31.
//  Copyright (c) 2013年 Kph-lab. All rights reserved.
//

#ifndef __OrbitYosoku__PFilter__
#define __OrbitYosoku__PFilter__

#include <iostream>
#include <XnCppWrapper.h>
#include <opencv/cv.h>
#include <opencv2/legacy/legacy.hpp>

#define N_PARTICLE 400
#define KINECT_WIDTH 640
#define KINECT_HEIGHT 480
// object color
//#define H 20.0   //red
//#define S 127.0
//#define V 127.0
#define R 255
#define G 0
#define B 0
//#define D 0
//#define LIKELIHOOD_SIGMA 100.0

/*class Point
{
public:
    Point(int, int);
    
    int sx, sy;
    float r, g, b;
    float h, s, v;
    float depth;
    float rx, ry, rz;
};*/

class PFilter
{
public:
    // constructor
    PFilter(xn::DepthGenerator*);

    // destructor
    ~PFilter();
    
    //パーティクルをリセット
    void reset();
    
    // 重みに基づきサンプリング & 状態遷移モデルを適用
    void resample_predict();
    
    // 重み付け & パーティクルの表示
    void weight_particle(cv::Mat image, cv::Mat depth);
    
    // パーティクルの重み付き平均を出力
    XnPoint3D* measure();
    
    // パーティクル数
    int n_particle;
    // 次元
    int n_stat;
    // Condensation
    CvConDensation *cond;
    //状態変数の上限・下限
    CvMat *lowerBound;
    CvMat *upperBound;
    
    xn::DepthGenerator* depthGenerator;
    
    int Hue;
    int Saturation;
    int Value;
    int Depth;
    int LIKELIHOOD_SIGMA;
};
#endif /* defined(__OrbitYosoku__PFilter__) */
