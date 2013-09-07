//
//  graphic.h
//  OrbitYosoku
//
//  Created by pheehs on 13/08/27.
//  Copyright (c) 2013年 Kph-lab. All rights reserved.
//

#ifndef __OrbitYosoku__graphic__
#define __OrbitYosoku__graphic__

#include "const.h"

//---マクロ定義---
#define glFovy 45        //視角度
#define glZNear 1.0        //near面の距離
#define glZFar 150.0    //far面の距離

//openGLのための宣言・定義
//---変数宣言---
int FormWidth = KINECT_WIDTH;
int FormHeight = KINECT_HEIGHT;
int mButton;
float twist, elevation, azimuth;
float cameraDistance = 0,cameraX = 0,cameraY = 0;
int xBegin, yBegin;

extern cv::Mat image;         //RGB画像
extern cv::Mat pointCloud_XYZ; //ポイントクラウド
extern cv::Point3f center3d;
extern cv::Point3f predict3d;
extern XnPlane3D plane;
extern void finish();

void polarview();

#endif /* defined(__OrbitYosoku__graphic__) */
