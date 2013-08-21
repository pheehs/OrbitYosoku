//
//  main.h
//  OrbitYosoku
//
//  Created by pheehs on 13/06/09.
//  Copyright (c) 2013年 Kph-lab. All rights reserved.
//

#ifndef OrbitYosoku_main_h
#define OrbitYosoku_main_h

#define KINECT_WIDTH 640
#define KINECT_HEIGHT 480

void display();
//void resize(int, int);

//OpenNI
xn::DepthGenerator depthGenerator;
xn::ImageGenerator imageGenerator;
xn::DepthMetaData depthMD;
xn::ImageMetaData imageMD;
xn::Context context;

//OpenCV
cv::Mat image;         //RGB画像
cv::Mat hsvimage;         //HSV画像
cv::Mat depth;        //深度画像

int key;
//FPS
int cnt = 0; // frame数
int oldcnt = 0; // 前フレーム数
int64 nowTime = 0; // 現時刻
int64 diffTime = 0; // 経過時間

int fps = 0; // 1秒のフレーム数
const double f = (1000 /cv::getTickFrequency());
int64 startTime = cv::getTickCount();

//openGLのための宣言・定義
//---変数宣言---
int FormWidth = KINECT_WIDTH;
int FormHeight = KINECT_HEIGHT;
int mButton;
float twist, elevation, azimuth;
float cameraDistance = 0,cameraX = 0,cameraY = 0;
int xBegin, yBegin;
cv::Mat pointCloud_XYZ(KINECT_HEIGHT,KINECT_WIDTH,CV_32FC3,cv::Scalar::all(0));

//---マクロ定義---
#define glFovy 45        //視角度
#define glZNear 1.0        //near面の距離
#define glZFar 150.0    //far面の距離

void retrievePointCloudMap(cv::Mat &depth,cv::Mat &pointCloud_XYZ);    //3次元ポイントクラウドのための座標変換
void drawPointCloud(cv::Mat &rgbImage,cv::Mat &pointCloud_XYZ);        //ポイントクラウド描画
void polarview();

#endif
