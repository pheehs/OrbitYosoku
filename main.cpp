//
//  main.cpp
//  OrbitYosoku
//
//  Created by pheehs on 13/05/28.
//  Copyright (c) 2013年 Kph-lab. All rights reserved.
//
#include <stdexcept>
#include <algorithm>
#include <iostream>
#include <XnCppWrapper.h>
#include <ctype.h>
#include <math.h>
#include <GL/glut.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//#include <opencv/cv.h>
//#include <opencv/highgui.h>
//#include <opencv2/legacy/legacy.hpp>
#include "main.h"


// 設定ファイルのパス
const char* CONFIG_XML_PATH = "config.xml";

void init()
{
  image = cv::Mat(KINECT_HEIGHT,KINECT_WIDTH,CV_8UC3);         //RGB画像
  hsvimage = cv::Mat(KINECT_HEIGHT,KINECT_WIDTH,CV_8UC3);         //HSV画像
  depth = cv::Mat(KINECT_HEIGHT,KINECT_WIDTH,CV_16UC1);        //深度画像
  
  //OpenNIの初期化
  XnStatus status = context.InitFromXmlFile(CONFIG_XML_PATH, 0);
  if(status != XN_STATUS_OK){
    std::cout << xnGetStatusString(status) << std::endl;
    exit(0);
  }
  
  context.FindExistingNode(XN_NODE_TYPE_DEPTH, depthGenerator);
  context.FindExistingNode(XN_NODE_TYPE_IMAGE, imageGenerator);
  
  //RGB画像と振動画像のズレを補正
  depthGenerator.GetAlternativeViewPointCap().SetViewPoint(imageGenerator);
  cv::namedWindow ("RGBImage", CV_WINDOW_AUTOSIZE);
  cv::namedWindow ("DepthImage", CV_WINDOW_AUTOSIZE);
  
  //cv::createTrackbar("Hue", "Condensation", &pf->Hue, 180);
  //cv::createTrackbar("LIKELIHOOD_SIGMA", "Condensation", &pf->LIKELIHOOD_SIGMA, 100);
  //cv::CcreateTrackbar("Depth", "Condensation", &pf->Depth, 255);
 
}

void end()
{
  cv::destroyWindow("RGBImage");
  cv::destroyWindow("DepthImage");
  context.Release();
}
//ポイントクラウド描画
void drawPointCloud(cv::Mat &rgbImage,cv::Mat &pointCloud_XYZ){
    int SEQ = 0;//配列番号
    int channel = rgbImage.channels();
    int step = rgbImage.step;
    glPointSize(2);   //点の大きさ
    glBegin(GL_POINTS);  //今から点を描画しますよっと
    for(int y = 0;y < KINECT_HEIGHT;y++){
        for(int x = 0;x < KINECT_WIDTH;x++){
            //Point3f &point = &pointCloud_XYZ.at<Point3f>(y,x);
            cv::Point3f &point = ((cv::Point3f*)(pointCloud_XYZ.data + pointCloud_XYZ.step.p[0]*y))[x];
            if(point.z == 0)  //奥行きがとれてなければ描画しない
                continue;
            SEQ = y * step + x * channel;
            glColor3f(rgbImage.data[SEQ + 2] / 255.0f,rgbImage.data[SEQ + 1] / 255.0f,rgbImage.data[SEQ + 0] / 255.0f);
            glVertex3f(point.x,point.y,point.z);
        }
    }
    glEnd();  //描画終了
}
//3次元ポイントクラウドのための座標変換
void retrievePointCloudMap(cv::Mat &depth,cv::Mat &pointCloud_XYZ){
    static XnPoint3D proj[KINECT_HEIGHT * KINECT_WIDTH] = {0};
    int SEQ = 0;    //配列番号
    for(int y = 0; y < KINECT_HEIGHT; y++ ){
        for(int x = 0; x < KINECT_WIDTH; x++ ){
            proj[SEQ].X = (XnFloat)x;
            proj[SEQ].Y = (XnFloat)y;
            proj[SEQ].Z = ((unsigned short*)(depth.data + depth.step.p[0]*y))[x] * 0.001f; // from mm to meters
            SEQ++;
        }
    }
    //現実座標に変換
    depthGenerator.ConvertProjectiveToRealWorld(KINECT_HEIGHT*KINECT_WIDTH, proj, (XnPoint3D*)pointCloud_XYZ.data);
}

void display()
{
    // clear screen and depth buffer
    glClear ( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    // Reset the coordinate system before modifying
    glLoadIdentity();
    glEnable(GL_DEPTH_TEST); //「Zバッファ」を有効
    gluLookAt(0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0);   //視点の向き設定
    context.WaitAndUpdateAll();
    
    imageGenerator.GetMetaData(imageMD);
    depthGenerator.GetMetaData(depthMD);
    
    memcpy(image.data,imageMD.Data(),image.step * image.rows);    //イメージデータを格納
    memcpy(depth.data,depthMD.Data(),depth.step * depth.rows);    //深度データを格納
    
    cv::cvtColor(image, hsvimage, CV_BGR2HSV_FULL);	//HSV画像作成
    cv::cvtColor(image, image, CV_BGR2RGB);     //BGRをRGBに
    
    //3次元ポイントクラウドのための座標変換
    retrievePointCloudMap(depth,pointCloud_XYZ);

    
    //視点の変更
    polarview();
    //ポイントクラウド
    drawPointCloud(image,pointCloud_XYZ);
    
    //パーティクルの描画
    glPointSize(2);   //点の大きさ
    glBegin(GL_POINTS);  //今から点を描画しますよっと
    glColor3i(255, 0, 0);


    glEnd();  //描画終了
    
    // FPS表示
    nowTime = cv::getTickCount();
    diffTime = (int)((nowTime- startTime)*f);
    
    if (diffTime >= 1000) {
        startTime = nowTime;
        fps = cnt - oldcnt;
        oldcnt = cnt;
    }
    
    std::ostringstream os;
    os << fps;
    std::string number = os.str();
    
    cv::putText(image, number, cvPoint(2, 28), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,0,200), 2, CV_AA);
    cnt++;
    
    cv::imshow("RGBImage", image);
    cv::imshow("DepthImage", depth);
    glFlush();
    glutSwapBuffers();
}
// アイドル時のコールバック
void idle(){
    //再描画要求
    glutPostRedisplay();
}
//ウィンドウのサイズ変更
void reshape (int width, int height){
    FormWidth = width;
    FormHeight = height;
    glViewport (0, 0, (GLsizei)width, (GLsizei)height);
    glMatrixMode (GL_PROJECTION);
    glLoadIdentity ();
    //射影変換行列の指定
    gluPerspective (glFovy, (GLfloat)width / (GLfloat)height,glZNear,glZFar);
    glMatrixMode (GL_MODELVIEW);
}
//マウスの動き
void motion(int x, int y){
    int xDisp, yDisp;
    xDisp = x - xBegin;
    yDisp = y - yBegin;
    switch (mButton) {
        case GLUT_LEFT_BUTTON:
            azimuth += (float) xDisp/2.0;
            elevation -= (float) yDisp/2.0;
            break;
        case GLUT_MIDDLE_BUTTON:
        case GLUT_RIGHT_BUTTON:
            cameraX -= (float) xDisp/40.0;
            cameraY += (float) yDisp/40.0;
            break;
    }
    xBegin = x;
    yBegin = y;
}
//マウスの操作
void mouse(int button, int state, int x, int y){
    if (state == GLUT_DOWN) {
        switch(button) {
            case GLUT_RIGHT_BUTTON:
            case GLUT_MIDDLE_BUTTON:
            case GLUT_LEFT_BUTTON:
                mButton = button;
                break;
        }
        xBegin = x;
        yBegin = y;
    }
}

void keyboard( unsigned char key, int x, int y )
{
    switch ( key )
    {
        case 27: // Escape key
            glutDestroyWindow ( glutGetWindow() );
            end();
            exit (0);
            break;
    }
    //glutPostRedisplay();
}

//視点変更
void polarview(){
    glTranslatef( cameraX, cameraY, cameraDistance);
    glRotatef( -twist, 0.0, 0.0, 1.0);
    glRotatef( -elevation, 1.0, 0.0, 0.0);
    glRotatef( -azimuth, 0.0, 1.0, 0.0);
}

int main(int argc, char *argv[])
{
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGB);
    glutInitWindowSize(FormWidth, FormHeight);
    glutCreateWindow(argv[0]);
    //コールバック
    glutReshapeFunc (reshape);
    glutDisplayFunc(display);
    glutIdleFunc(idle);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);
    glutKeyboardFunc(keyboard);
    init();
    glutMainLoop();
    return 0;
}

