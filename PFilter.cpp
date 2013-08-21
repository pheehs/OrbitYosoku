//
//  PFilter.cpp
//  OrbitYosoku
//
//  Created by pheehs on 13/05/31.
//  Copyright (c) 2013年 Kph-lab. All rights reserved.
//

#include "PFilter.h"

PFilter::PFilter(xn::DepthGenerator* depthGenerator)
{
    Hue = 0;
    Saturation = 127;
    Value = 127;
    Depth = 0;
    LIKELIHOOD_SIGMA = 10;
    int i;
    
    n_stat = 4;
    n_particle = N_PARTICLE;
    this->depthGenerator = depthGenerator;
    // (4)Condensation構造体を作成する．
    cond = cvCreateConDensation (n_stat, 0, n_particle);
    
    // (5)状態ベクトル各次元の取りうる最小値・最大値を指定する．
    lowerBound = cvCreateMat (4, 1, CV_32FC1);
    upperBound = cvCreateMat (4, 1, CV_32FC1);
    
    cvmSet (lowerBound, 0, 0, 0.0);
    cvmSet (lowerBound, 1, 0, 0.0);
    cvmSet (lowerBound, 2, 0, -100.0);
    cvmSet (lowerBound, 3, 0, -100.0);
    cvmSet (upperBound, 0, 0, KINECT_WIDTH);
    cvmSet (upperBound, 1, 0, KINECT_HEIGHT);
    cvmSet (upperBound, 2, 0, 100.0);
    cvmSet (upperBound, 3, 0, 100.0);
    
    // (6)Condensation構造体を初期化する
    cvConDensInitSampleSet (cond, lowerBound, upperBound);
    
    // (7)ConDensationアルゴリズムにおける状態ベクトルのダイナミクスを指定する
    float dynamics[] = {
        1.0, 0.0, 1.0, 0.0, //x
        0.0, 1.0, 0.0, 1.0, //y
        0.0, 0.0, 1.0, 0.0, //x'
        0.0, 0.0, 0.0, 1.0, //y'
    };
    for (i=0; i < 4*4; i++) {
        cond->DynamMatr[i] = dynamics[i];
    }
    
    // (8)ノイズパラメータを再設定する．
    cvRandInit (&(cond->RandS[0]), -25, 25, (int) cvGetTickCount ());
    cvRandInit (&(cond->RandS[1]), -25, 25, (int) cvGetTickCount ());
    cvRandInit (&(cond->RandS[2]), -5, 5, (int) cvGetTickCount ());
    cvRandInit (&(cond->RandS[3]), -5, 5, (int) cvGetTickCount ());
}

PFilter::~PFilter()
{
    cvReleaseConDensation (&cond);
    cvReleaseMat (&lowerBound);
    cvReleaseMat (&upperBound);
}
void PFilter::reset()
{
    // (6)Condensation構造体を初期化する
    cvConDensInitSampleSet (cond, lowerBound, upperBound);
}
void PFilter::resample_predict()
{
    // (10)次のモデルの状態を推定する
    cvConDensUpdateByTime (cond);
}

void PFilter::weight_particle(cv::Mat image, cv::Mat depth)
{
    int i;
    int xx, yy;
    int h, s, v, d;
    float dist = 0.0;
    
    // (9)各パーティクルについて尤度を計算する．
    for (i = 0; i < n_particle; i++) {
        xx = (int) (cond->flSamples[i][0]);
        yy = (int) (cond->flSamples[i][1]);
        
        std::cout << "all:" << i << "|" << xx << "," << yy << std::endl; //
        if (xx < 0.0 || xx >= KINECT_WIDTH || yy < 0.0 || yy >= KINECT_HEIGHT) { // 画面の外
            cond->flConfidence[i] = 0.0;
        } else {
            std::cout << "draw:" << i << "|" << xx << "," << yy << std::endl; //
            // 対応する座標のPointがあるか
            
            // (1)尤度の計算
            h = image.data[image.step * yy + xx * image.elemSize()];
            s = image.data[image.step * yy + xx * image.elemSize() + 1];
            v = image.data[image.step * yy + xx * image.elemSize() + 2];
            d = depth.data[depth.step * yy + xx * depth.elemSize()];
            
            dist = sqrt ((Hue - h)*(Hue - h) + (Depth - d)*(Depth - d)); //+ (S - s)*(S - s) + (V - v)*(V - v));
            cond->flConfidence[i] = 1.0 / (sqrt (2.0 * CV_PI) * LIKELIHOOD_SIGMA) * expf (-dist * dist / (2.0 * LIKELIHOOD_SIGMA * LIKELIHOOD_SIGMA));
        }
    }
}

