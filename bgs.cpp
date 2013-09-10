//
//  bgs.cpp
//  OrbitYosoku
//
//  Created by pheehs on 13/08/23.
//  Copyright (c) 2013年 Kph-lab. All rights reserved.
//
//ref: http://opencv.jp/sample/accumulation_of_background.html
// This code is for Background Subtraction

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc_c.h>

#include "bgs.h"
#include "const.h"

using namespace std;
using namespace cv;

BGS::BGS(Mat (*getFrame)())
:INIT_TIME(100),B_PARAM(1.0 / 200.0),T_PARAM(1.0 / 1000.0),Zeta(40.0)
{
    cout << "Background statistics initialization start" << endl;
    
    av_img = Mat::zeros(KINECT_HEIGHT,KINECT_WIDTH,CV_32FC3);
    sgm_img = Mat::zeros(KINECT_HEIGHT,KINECT_WIDTH,CV_32FC3);
    tmp_img = Mat(KINECT_HEIGHT,KINECT_WIDTH,CV_32FC3);
    upper_img = Mat(KINECT_HEIGHT,KINECT_WIDTH,CV_32FC3);
    lower_img = Mat(KINECT_HEIGHT,KINECT_WIDTH,CV_32FC3);
    dst_img = Mat(KINECT_HEIGHT,KINECT_WIDTH,CV_8UC3);
    msk_img = Mat(KINECT_HEIGHT,KINECT_WIDTH,CV_8UC1);
    
    // (4)背景の輝度平均の初期値を計算する
    Mat frame = Mat(KINECT_HEIGHT,KINECT_WIDTH,CV_8UC3);
    for (int i=0; i < INIT_TIME; i++) {
        frame = getFrame();
        frame.convertTo(tmp_img, CV_32FC3);
        //accumulate(tmp_img, av_img);
        av_img += tmp_img;
    }
    av_img *= 1.0 / INIT_TIME;
    
    // (5)背景の輝度振幅の初期値を計算する
    for (int i = 0; i < INIT_TIME; i++) {
        frame = getFrame();
        frame.convertTo(tmp_img, CV_32FC3);
        tmp_img = tmp_img - av_img;
        pow(tmp_img, 2.0, tmp_img);
        tmp_img *= 2.0;
        pow(tmp_img, 0.5, tmp_img);
        sgm_img += tmp_img;
    }
    sgm_img *= 1.0 / INIT_TIME;
    
    cout << "Background statistics initialization finish" << endl;

}

void BGS::process(const cv::Mat &img_input, cv::Mat &img_output)
{
    img_input.convertTo(tmp_img, CV_32FC3);
    
    // (8)背景となりうる画素の輝度値の範囲をチェックする
    lower_img = av_img - sgm_img - Zeta;
    upper_img = av_img + sgm_img + Zeta;
    inRange(tmp_img, lower_img, upper_img, msk_img);
    
    // (9)輝度振幅を再計算する
    tmp_img -= av_img;
    pow(tmp_img, 2.0, tmp_img);
    tmp_img *= 2.0;
    pow(tmp_img, 0.5, tmp_img);
    
    // (10)背景と判断された領域の背景の輝度平均と輝度振幅を更新する
    accumulateWeighted(img_input, av_img, B_PARAM, msk_img);
    //cvRunningAvg (&img_input, &av_img, B_PARAM, &msk_img);
    accumulateWeighted(tmp_img, sgm_img, B_PARAM, msk_img);
    //cvRunningAvg (&tmp_img, &sgm_img, B_PARAM, &msk_img);
    
    // (11)物体領域と判断された領域では輝度振幅のみを（背景領域よりも遅い速度で）更新する
    bitwise_not(msk_img, msk_img);
    accumulateWeighted(tmp_img, sgm_img, T_PARAM, msk_img);
    //cvRunningAvg (&tmp_img, &sgm_img, T_PARAM, &msk_img);
    
    img_output = msk_img.clone();
    return;
}
