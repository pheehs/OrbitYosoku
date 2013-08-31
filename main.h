//
//  main.h
//  OrbitYosoku
//
//  Created by pheehs on 13/06/09.
//  Copyright (c) 2013年 Kph-lab. All rights reserved.
//

#ifndef OrbitYosoku_main_h
#define OrbitYosoku_main_h

#include "const.h"

//OpenNI
xn::DepthGenerator depthGenerator;
xn::ImageGenerator imageGenerator;
xn::SceneAnalyzer sceneAnalyzer;
xn::DepthMetaData depthMD;
xn::ImageMetaData imageMD;
xn::Context context;
XnPlane3D plane;

//OpenCV
cv::Mat image;         //RGB画像
cv::Mat hsvimage;         //HSV画像
cv::Mat depth;        //深度画像16bit
cv::Mat depth_8UC1;   //深度画像8bit
cv::Mat pointCloud_XYZ; //ポイントクラウド
cv::Point3f center3d;
cv::Point3f predict3d;


//FPS
int cnt = 0; // frame数
int oldcnt = 0; // 前フレーム数
int64 nowTime = 0; // 現時刻
int64 diffTime = 0; // 経過時間

int fps = 0; // 1秒のフレーム数
const double f = (1000 /cv::getTickFrequency());
int64 startTime = cv::getTickCount();

/* Background Subtraction */
BGS* bgs_rgb;
BGS* bgs_depth;

/* Least Square Method */
LSM* lsm_x;
LSM* lsm_y;
LSM* lsm_z;
LSM* lsm_X;
LSM* lsm_Y;

pthread_t process_th;
bool status = true; //true:running/false:time to kill

cv::Mat getImage();
cv::Mat getDepth();

extern void graphic_main(int argc, char* argv[]);

#endif
