//
//  graphic.cpp
//  OrbitYosoku
//
//  Created by pheehs on 13/08/27.
//  Copyright (c) 2013年 Kph-lab. All rights reserved.
//

#include <iostream>
#include <GLUT/glut.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "graphic.h"
#include "const.h"

using namespace cv;
using namespace std;


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

void display()
{
    // clear screen and depth buffer
    glClear ( GL_COLOR_BUFFER_BIT );
    // Reset the coordinate system before modifying
    glLoadIdentity();
    gluLookAt(0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0);   //視点の向き設定
    
    //視点の変更
    polarview();
    //ポイントクラウド
    drawPointCloud(image,pointCloud_XYZ);
    
    //重心の描画
    glPointSize(40);   //点の大きさ
    glBegin(GL_POINTS);  //今から点を描画しますよっと
    glColor3i(255, 0, 0);
    glVertex3f(center3d.x,center3d.y,center3d.z);
    glColor3i(0,0,255);
    glVertex3f(predict3d.x,predict3d.y,predict3d.z);
    glEnd();  //描画終了
    
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
            finish();
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

void graphic_main(int argc, char* argv[])
{
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
    glutInitWindowSize(FormWidth, FormHeight);
    glutCreateWindow("3D view");
    //コールバック
    glutReshapeFunc (reshape);
    glutDisplayFunc(display);
    glutIdleFunc(idle);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);
    glutKeyboardFunc(keyboard);
    glutMainLoop();
}
