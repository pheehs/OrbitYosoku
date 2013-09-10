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

/*
 // 視点操作のための変数
 float  camera_yaw = -30.0;    // Ｙ軸を中心とする回転角度
 float  camera_pitch = -30.0;  // Ｘ軸を中心とする回転角度
 float  camera_distance = 15.0;// 中心からカメラの距離
 */

// マウスのドラッグのための変数
int  drag_mouse_r = 0;  // 右ボタンをドラッグ中かどうかのフラグ（0:非ドラッグ中,1:ドラッグ中）
int  drag_mouse_l = 0;  // 左ボタンをドラッグ中かどうかのフラグ（0:非ドラッグ中,1:ドラッグ中）
int  last_mouse_x;      // 最後に記録されたマウスカーソルのＸ座標
int  last_mouse_y;      // 最後に記録されたマウスカーソルのＹ座標


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
extern cv::Mat Xaxis,Yaxis,Zaxis;
extern XnPlane3D plane;
extern bool status;
extern void finish();
extern float at_float(cv::Mat,int,int);

void polarview();

#endif /* defined(__OrbitYosoku__graphic__) */
