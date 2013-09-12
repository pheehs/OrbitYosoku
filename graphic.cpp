//
//  graphic.cpp
//  OrbitYosoku
//
//  Created by pheehs on 13/08/27.
//  Copyright (c) 2013年 Kph-lab. All rights reserved.
//

#include <iostream>
#include <GLUT/glut.h>
#include <XnCppWrapper.h>
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

//床平面描画
void drawFloor(){
    //cout << "point(" << plane.ptPoint.X << "," << plane.ptPoint.Y << "," << plane.ptPoint.Z << "), normal(" << plane.vNormal.X << "," << plane.vNormal.Y << "," << plane.vNormal.Z << ")" << endl ;
    if (plane.vNormal.X == 0 && plane.vNormal.Y == 0 && plane.vNormal.Z == 0){
        return;
    }
    float floor_d = plane.vNormal.X*plane.ptPoint.X + plane.vNormal.Y*plane.ptPoint.Y + plane.vNormal.Z*plane.ptPoint.Z;
    //printf("floor: %f x + %f y + %f z = %f\n", plane.vNormal.X, plane.vNormal.Y, plane.vNormal.Z, floor_d);
    
    //glPointSize(10);
    glBegin(GL_POLYGON);
    glColor3f(0.0f,1.0f,0.0f);
    glVertex3f(plane.ptPoint.X,plane.ptPoint.Y,plane.ptPoint.Z);
    //glVertex3f(0, 0, 0);
    //printf("(%f, %f, %f)", plane.ptPoint.X,plane.ptPoint.Y,plane.ptPoint.Z);
    for (int xi = 0; xi < 20; xi++) {
        for (int yi = 0; yi < 20; yi++) {
            glVertex3f(plane.ptPoint.X + xi, plane.ptPoint.Y + yi, (floor_d - plane.vNormal.X*(plane.ptPoint.X+xi) - plane.vNormal.Y*(plane.ptPoint.Y+yi)) / plane.vNormal.Z);
            //printf("(%f,%f,%f)", plane.ptPoint.X + xi, plane.ptPoint.Y + yi, (floor_d - plane.vNormal.X*(plane.ptPoint.X+xi) - plane.vNormal.Y*(plane.ptPoint.Y+yi)) / plane.vNormal.Z);
        }
    }
    glEnd();
}

void display()
{
    // clear screen and depth buffer
    glClear ( GL_COLOR_BUFFER_BIT );
    /*// Reset the coordinate system before modifying
    glLoadIdentity();
    gluLookAt(0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0);   //視点の向き設定
    
    //視点の変更
    polarview();*/
    //ポイントクラウド
    drawPointCloud(image,pointCloud_XYZ);
    //床平面の描画
    drawFloor();
    
    //重心の描画
    glBegin(GL_POLYGON);
    
    glPushMatrix(); //重心
    glColor3f(1.0, 0.0, 0.0);//red
    glTranslatef(center3d_graphic.x,center3d_graphic.y,center3d_graphic.z);
    glutSolidSphere(0.00,10,10);
    glPopMatrix();
    
    glPushMatrix(); //重心
    glColor3f(1.0, 0.0, 0.0);//red
    glTranslatef(center3d_graphic.x,center3d_graphic.y,center3d_graphic.z);
    glutSolidSphere(0.05,10,10);
    glPopMatrix();

    glPushMatrix(); //的１
    glColor3f(0.0,1.0,1.0); //skyblue
    glTranslatef(target[0].x,target[0].y,target[0].z);
    glutSolidSphere(0.03,10,10);
    glPopMatrix();

    glPushMatrix(); //的2
    glColor3f(0.0,1.0,1.0); //skyblue
    glTranslatef(target[1].x,target[1].y,target[1].z);
    glutSolidSphere(0.03,10,10);
    glPopMatrix();
    
    glPushMatrix(); //予測点
    glColor3f(0.0,0.0,1.0); //blue
    glTranslatef(predict3d.x,predict3d.y,predict3d.z);
    glutSolidSphere(0.03,10,10);
    glPopMatrix();
    
    for (int i=0; i < 100; i++) { //予測軌道
        glPushMatrix();
        switch (crashing[i]) {
            case 0:
                glColor3f(1.0,0.0,1.0); //purple
                break;
            case 1:
                glColor3f(1.0,0.0,1.0); //purple
                break;
            case -1:
            default:
                glColor3f(0.0,0.0,1.0); //blue
                break;
        }
        glTranslatef(orbit3d_list[i].x,orbit3d_list[i].y,orbit3d_list[i].z);
        glutSolidSphere(0.02,10,10);
        glPopMatrix();
        
    }
    glEnd();  //描画終了
    
    //RealWorld軸
    glBegin(GL_LINE_STRIP);//x
    //glPushMatrix();
    glColor3f(1.0,0.0,0.0);
    glVertex3f(0, 0, 0);
    glVertex3f(1, 0, 0);
    glPopMatrix();
    glEnd();
    glBegin(GL_LINE_STRIP);//y
    //glPushMatrix();
    glColor3f(0.0,1.0,0.0);
    glVertex3f(0, 0, 0);
    glVertex3f(0, 1, 0);
    //glPopMatrix();
    glEnd();
    glBegin(GL_LINE_STRIP);//z
    //glPushMatrix();
    glColor3f(0.0,0.0,1.0);
    glVertex3f(0, 0, 0);
    glVertex3f(0, 0, 1);
    //glPopMatrix();
    glEnd();
    
    //Parabolic軸
    glBegin(GL_LINE_STRIP);//X
    //glPushMatrix();
    glColor3f(1.0,0.0,0.0);
    glVertex3f(0, 0, 0);
    glVertex3f(at_float(Xaxis, 0, 0), at_float(Xaxis, 1, 0), at_float(Xaxis, 2, 0));
    glPopMatrix();
    glEnd();
    glBegin(GL_LINE_STRIP);//Y
    //glPushMatrix();
    glColor3f(0.0,1.0,0.0);
    glVertex3f(0, 0, 0);
    glVertex3f(at_float(Yaxis, 0, 0), at_float(Yaxis, 1, 0), at_float(Yaxis, 2, 0));
    //glPopMatrix();
    glEnd();
    glBegin(GL_LINE_STRIP);//Z
    //glPushMatrix();
    glColor3f(0.0,0.0,1.0);
    glVertex3f(0, 0, 0);
    glVertex3f(at_float(Zaxis, 0, 0), at_float(Zaxis, 1, 0), at_float(Zaxis, 2, 0));
    //glPopMatrix();
    glEnd();
        
    if (running == false) {
        glutDestroyWindow ( glutGetWindow() );
        finish();
    }
    
    glutSwapBuffers();
}
// アイドル時のコールバック
void idle(){
    //再描画要求
    glutPostRedisplay();
}
/*//ウィンドウのサイズ変更
void reshape (int width, int height){
    FormWidth = width;
    FormHeight = height;
    glViewport (0, 0, (GLsizei)width, (GLsizei)height);
    glMatrixMode (GL_PROJECTION);
    glLoadIdentity ();
    //射影変換行列の指定
    gluPerspective (glFovy, (GLfloat)width / (GLfloat)height,glZNear,glZFar);
    glMatrixMode (GL_MODELVIEW);
}*/
//
//  ウィンドウサイズ変更時に呼ばれるコールバック関数
//
void  reshape( int w, int h )
{
	// ウィンドウ内の描画を行う範囲を設定（ウィンドウ全体に描画するように設定）
	glViewport(0, 0, w, h);
    
	// カメラ座標系→スクリーン座標系への変換行列を設定
	glMatrixMode( GL_PROJECTION );
	glLoadIdentity();
	gluPerspective( 45, (double)w/h, 1, 500 );
}
//
//  マウスクリック時に呼ばれるコールバック関数
//
void  mouse( int button, int state, int mx, int my )
{
	// 右ボタンが押されたらドラッグ開始のフラグを設定
	if ( ( button == GLUT_LEFT_BUTTON ) && ( state == GLUT_DOWN ) )
		drag_mouse_l = 1;
	// 右ボタンが離されたらドラッグ終了のフラグを設定
	else if ( ( button == GLUT_LEFT_BUTTON ) && ( state == GLUT_UP ) )
		drag_mouse_l = 0;
    
	// 右ボタンが押されたらドラッグ開始のフラグを設定
	if ( ( button == GLUT_RIGHT_BUTTON ) && ( state == GLUT_DOWN ) )
		drag_mouse_r = 1;
	// 右ボタンが離されたらドラッグ終了のフラグを設定
	else if ( ( button == GLUT_RIGHT_BUTTON ) && ( state == GLUT_UP ) )
		drag_mouse_r = 0;
    
	// 現在のマウス座標を記録
	last_mouse_x = mx;
	last_mouse_y = my;
}
//
//  マウスドラッグ時に呼ばれるコールバック関数
//
void  motion( int mx, int my )
{
	// 左ボタンのドラッグ中であれば、マウスの移動量に応じて視点を移動する
	if ( drag_mouse_l == 1 )
	{
        /*		// マウスの縦移動に応じて距離を移動
         camera_distance += ( my - last_mouse_y ) * 0.2;
         if ( camera_distance < 5.0 )
         camera_distance = 5.0;
         */
		// マウスの縦移動に応じてカメラの距離を変化
		float  delta_dist = ( my - last_mouse_y ) * 0.5;
        
		// 現在の変換行列（カメラの向き）を取得
		float  m[ 16 ];
		glGetFloatv( GL_MODELVIEW_MATRIX, m );
        
		// 変換行列を初期化して、カメラ移動分の平行移動行列を設定
		glMatrixMode( GL_MODELVIEW );
		glLoadIdentity();
		glTranslatef( 0.0,  0.0, - delta_dist );
        
		// 右からこれまでの変換行列をかける
		glMultMatrixf( m );
	}
	
	// 右ボタンのドラッグ中であれば、マウスの移動量に応じて視点を回転する
	if ( drag_mouse_r == 1 )
	{
        /*		// マウスの横移動に応じてＹ軸を中心に回転
         camera_yaw -= ( mx - last_mouse_x ) * 1.0;
         if ( camera_yaw < 0.0 )
         camera_yaw += 360.0;
         else if ( camera_yaw > 360.0 )
         camera_yaw -= 360.0;
         */
		// マウスの横移動に応じてＹ軸を中心に回転
		float  delta_yaw = ( mx - last_mouse_x ) * 0.5;
        
		// 現在の変換行列の右側に、今回の回転変換をかける
		glMatrixMode( GL_MODELVIEW );
		glRotatef( delta_yaw, 0.0, 1.0, 0.0 );
	}
    
	// 右ボタンのドラッグ中であれば、マウスの移動量に応じて視点を回転する
	if ( drag_mouse_r == 1 )
	{
        /*		// マウスの縦移動に応じてＸ軸を中心に回転
         camera_pitch -= ( my - last_mouse_y ) * 1.0;
         if ( camera_pitch < -90.0 )
         camera_pitch = -90.0;
         else if ( camera_pitch > 0.0 )
         camera_pitch = 0.0;
         */
		// マウスの縦移動に応じてＸ軸を中心に回転
		float  delta_pitch = ( my - last_mouse_y ) * 0.5;
        
		// 現在の変換行列を取得
		float  m[ 16 ];
		float  tx, ty, tz;
		glGetFloatv( GL_MODELVIEW_MATRIX, m );
        
		// 現在の変換行列の平行移動成分を記録
		tx = m[ 12 ];
		ty = m[ 13 ];
		tz = m[ 14 ];
        
		// 現在の変換行列の平行移動成分を０にする
		m[ 12 ] = 0.0f;
		m[ 13 ] = 0.0f;
		m[ 14 ] = 0.0f;
        
		// 変換行列を初期化
		glMatrixMode( GL_MODELVIEW );
		glLoadIdentity();
        
		// カメラの平行移動行列を設定
		glTranslatef( tx, ty, tz );
        
		// 右側に、今回の回転変換をかける
		glRotatef( delta_pitch, 1.0, 0.0, 0.0 );
        
		// さらに、右側に、もとの変換行列から平行移動成分をとり除いたものをかける
		glMultMatrixf( m );
	}
    
	// 今回のマウス座標を記録
	last_mouse_x = mx;
	last_mouse_y = my;
    
	// 再描画の指示を出す（この後で再描画のコールバック関数が呼ばれる）
	glutPostRedisplay();
}

/*//マウスの動き
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
}*/
/*//マウスの操作
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
}*/

void keyboard( unsigned char key, int x, int y )
{
    switch ( key )
    {
        case 27: // Escape key
            status = false;
            break;
    }
    glutPostRedisplay();
}

/*//視点変更
void polarview(){
    glTranslatef( cameraX, cameraY, cameraDistance);
    glRotatef( -twist, 0.0, 0.0, 1.0);
    glRotatef( -elevation, 1.0, 0.0, 0.0);
    glRotatef( -azimuth, 0.0, 1.0, 0.0);
}*/

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
    
    // 変換行列を初期化
	glMatrixMode( GL_MODELVIEW );
	glLoadIdentity();
	glTranslatef( 0.0,  0.0, -2.0 );
	glRotatef( 10.0, 1.0, 0.0, 0.0 );
	glRotatef( 180.0, 0.0, 1.0, 0.0 );
    
    glutMainLoop();
}
