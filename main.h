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
cv::Mat image(KINECT_HEIGHT,KINECT_WIDTH,CV_8UC3);         //RGB画像;
cv::Mat hsvimage(KINECT_HEIGHT,KINECT_WIDTH,CV_8UC3);      //HSV画像
cv::Mat depth(KINECT_HEIGHT,KINECT_WIDTH,CV_16UC1);        //深度画像16bit
cv::Mat depth_8UC1(KINECT_HEIGHT,KINECT_WIDTH,CV_8UC1);    //深度画像8bit
cv::Mat pointCloud_XYZ(KINECT_HEIGHT,KINECT_WIDTH,CV_32FC3,cv::Scalar::all(0)); //ポイントクラウド
cv::Mat pointCloud_para(KINECT_HEIGHT,KINECT_WIDTH,CV_32FC3,cv::Scalar::all(0)); //放物線計算用座標
cv::Point3f center3d;
cv::Point3f center3d_graphic;
cv::Point3f predict3d;
cv::Point3f center3d_prev;
cv::Mat vvector = cv::Mat::zeros(3,1,CV_32FC1); //物体の速度ベクトル
cv::Mat Xaxis = cv::Mat::zeros(3,1,CV_32FC1); //変換後の座標系でのX軸方向の単位ベクトル
cv::Mat Yaxis = cv::Mat::zeros(3,1,CV_32FC1); //Y軸方向単位ベクトル
cv::Mat Zaxis = cv::Mat::zeros(3,1,CV_32FC1); //Z軸方向単位ベクトル
cv::Mat Xaxis_prev = cv::Mat::zeros(3,1,CV_32FC1); //変換後の座標系でのX軸方向の単位ベクトル
cv::Mat Yaxis_prev = cv::Mat::zeros(3,1,CV_32FC1); //Y軸方向単位ベクトル
cv::Mat Zaxis_prev = cv::Mat::zeros(3,1,CV_32FC1); //Z軸方向単位ベクトル
cv::Mat Rotate = cv::Mat::zeros(3, 3, CV_32FC1);
cv::Mat Rotate_inv = cv::Mat::zeros(3, 3, CV_32FC1);
cv::Mat white_img(KINECT_HEIGHT, KINECT_WIDTH, CV_8UC3, cv::Scalar::all(255));
cv::Mat yellow_img(KINECT_HEIGHT, KINECT_WIDTH, CV_8UC3, cv::Scalar(0,255,255));
cv::Mat blue_img(KINECT_HEIGHT, KINECT_WIDTH, CV_8UC3, cv::Scalar(255,0,0));

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
LSM* lsm_XY;

pthread_t process_th;
bool status = true; //true:running/false:time to kill
bool running = true;
cv::Point3f target[2];

cv::Mat getImage();
cv::Mat getDepth();
void camera_test();
void register_target();
float at_float(cv::Mat,int,int);

extern void graphic_main(int argc, char* argv[]);
extern cv::Point3f orbit3d_list[100];
extern int crashing[100];

#endif
