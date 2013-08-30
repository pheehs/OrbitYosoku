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
#include <ctype.h>
#include <math.h>
#include <XnCppWrapper.h>
#include <pthread.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>

#include "bgs.h"
#include "lsm.h"
#include "main.h"

using namespace cv;
using namespace std;

// 設定ファイルのパス
const char* CONFIG_XML_PATH = "/Users/shuhei/workspace/kinect/OrbitYosoku/OrbitYosoku/config.xml";

void init()
{
    image = Mat(KINECT_HEIGHT,KINECT_WIDTH,CV_8UC3);         //RGB画像
    hsvimage = Mat(KINECT_HEIGHT,KINECT_WIDTH,CV_8UC3);         //HSV画像
    depth = Mat(KINECT_HEIGHT,KINECT_WIDTH,CV_16UC1);        //深度画像16bit
    depth_8UC1 = Mat(KINECT_HEIGHT,KINECT_WIDTH,CV_8UC1);        //深度画像8bit
    pointCloud_XYZ = Mat(KINECT_HEIGHT,KINECT_WIDTH,CV_32FC3,cv::Scalar::all(0)); //ポイントクラウド
    
    //OpenNIの初期化
    XnStatus r = context.InitFromXmlFile(CONFIG_XML_PATH, 0);
    if(r != XN_STATUS_OK){
        cout << xnGetStatusString(r) << endl;
        exit(0);
    }
    
    context.FindExistingNode(XN_NODE_TYPE_DEPTH, depthGenerator);
    context.FindExistingNode(XN_NODE_TYPE_IMAGE, imageGenerator);
    context.FindExistingNode(XN_NODE_TYPE_SCENE, sceneAnalyzer);
    
    //RGB画像と振動画像のズレを補正
    depthGenerator.GetAlternativeViewPointCap().SetViewPoint(imageGenerator);
    //namedWindow ("RGBImage", CV_WINDOW_AUTOSIZE);
    //namedWindow ("DepthImage", CV_WINDOW_AUTOSIZE);
    namedWindow ("RGBMask", CV_WINDOW_AUTOSIZE);
    namedWindow ("DepthMask", CV_WINDOW_AUTOSIZE);
    namedWindow ("Mask", CV_WINDOW_AUTOSIZE);
    namedWindow ("Centroid", CV_WINDOW_AUTOSIZE);
    
    bgs_rgb = new BGS;
    bgs_depth = new BGS;
    createTrackbar("rgb_thresh", "Mask", &bgs_rgb->threshold, 100);
    createTrackbar("depth_thresh", "Mask", &bgs_depth->threshold, 100);
    
    lsm_x = new LSM;
    lsm_y = new LSM;
    lsm_z = new LSM;
    lsm_X = new LSM;
    lsm_Y = new LSM;
}

void end()
{
    destroyAllWindows();
    context.Release();
    delete bgs_rgb;
    delete bgs_depth;
    delete lsm_x;
    delete lsm_y;
    delete lsm_z;
    delete lsm_X;
    delete lsm_Y;
}
string getFPS()
{
    // FPS表示
    nowTime = getTickCount();
    diffTime = (int)((nowTime- startTime)*f);
    
    if (diffTime >= 1000) {
        startTime = nowTime;
        fps = cnt - oldcnt;
        oldcnt = cnt;
    }
    
    ostringstream os;
    os << fps;
    
    cnt++;
    return os.str();
}
Point3f getCentroid(Mat mask, Mat pointCloud)
{
    float sumX = 0, sumY = 0, sumZ = 0;
    int counter = 0;
    Point3f point;
    
    for (int y = 0; y < KINECT_HEIGHT; y++) {
        for (int x = 0; x < KINECT_WIDTH; x++) {
            if (mask.at<int>(y,x)) {
                point = pointCloud.at<Point3f>(y, x);
                sumX += point.x;
                sumY += point.y;
                sumZ += point.z;
                counter++;
            }
            
        }
    }
    return Point3f(sumX/counter, sumY/counter, sumZ/counter);
}
//3次元ポイントクラウドのための座標変換
void retrievePointCloudMap(Mat &depth,Mat &pointCloud_XYZ){
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
void update()
{
    Mat image_mask;
    Mat depth_mask;
    Mat mask;
        
    context.WaitAndUpdateAll();
    
    imageGenerator.GetMetaData(imageMD);
    depthGenerator.GetMetaData(depthMD);
    
    memcpy(image.data,imageMD.Data(),image.step * image.rows);    //イメージデータを格納
    memcpy(depth.data,depthMD.Data(),depth.step * depth.rows);    //深度データを格納
    
    cvtColor(image, hsvimage, CV_BGR2HSV_FULL);	//HSV画像作成
    cvtColor(image, image, CV_BGR2RGB);     //BGRをRGBに
    
    //3次元ポイントクラウドのための座標変換
    retrievePointCloudMap(depth,pointCloud_XYZ);
    
    //RGB画像と深度画像でそれぞれ背景差分
    depth.convertTo(depth_8UC1, CV_8UC1); //深度画像を16bit->8bitに
    bgs_rgb->process(image, image_mask);
    bgs_depth->process(depth_8UC1, depth_mask);
    
    //imshow("RGBImage", image);
    //imshow("DepthImage", depth);
    if (!image_mask.empty())
        imshow("RGBMask", image_mask);
    if (!depth_mask.empty())
        imshow("DepthMask", depth_mask);
    // RGBと深度の両方で検出された領域を抽出
    bitwise_and(image_mask, depth_mask, mask);
    
    if (!mask.empty()) {
        // 収縮と膨張でノイズな領域を削除。「重要な処理」
        erode(mask, mask, Mat());
        dilate(mask, mask, Mat());
        dilate(mask, mask, Mat());
        erode(mask, mask, Mat());
        
        //　画面上での重心を計算
        Moments m = moments(mask, true);
        Point2f center2d(m.m10/m.m00, m.m01/m.m00);
        Point2f predict2d(lsm_X->predict()/1000, lsm_Y->predict()/1000);
        
        /// 重心（赤）と予想した重心（青）の描画
        Mat centroid = image.clone();
        circle( centroid, center2d, 8, Scalar(0,0,200), -1, 4, 0 );
        circle( centroid, predict2d, 8, Scalar(200,0,0), -1, 4, 0 );
        
        // 画面上での予測軌道を描画
        int i = 1;
        Point2f orbit;
        do {
            //0.01sec毎の軌跡
            orbit = Point2f(lsm_X->predict(lsm_X->x+10*i)/1000, lsm_Y->predict(lsm_Y->x+10*i)/1000);
            circle( centroid, orbit, 5, Scalar(0,200,0), -1, 4, 0 );
            i++;
        } while (0 < orbit.x < KINECT_WIDTH && 0 < orbit.y < KINECT_HEIGHT && i < 1000);
        //画面外か、10sec以上経過で終了
        
        putText(centroid, getFPS(), cvPoint(2, 28), FONT_HERSHEY_SIMPLEX, 1, Scalar(0,0,200), 2, CV_AA);
        imshow("Mask", mask);
        imshow("Centroid", centroid);
        
        //現実座標での重心を計算
        center3d = getCentroid(mask, pointCloud_XYZ);
        predict3d = Point3f(lsm_x->predict()/1000, lsm_y->predict()/1000, lsm_z->predict()/1000);
    
        if (center3d.x!=0 && center3d.y!=0 && center3d.z!=0) {
            // <-- 2D -->
            printf("center2d: %f, %f\n", center2d.x, center2d.y);
            printf("predict2d: %f, %f\n", predict2d.x, predict2d.y);
            //最小二乗法
            lsm_X->process((int)(center2d.x*1000)); //m to mm
            lsm_Y->process((int)(center2d.y*1000));
            printf("X = %f t^2+ %f t+ %f\n", lsm_X->a, lsm_X->b, lsm_X->c);
            printf("Y = %f t^2+ %f t+ %f\n", lsm_Y->a, lsm_Y->b, lsm_Y->c);

            // <-- 3D -->
            printf("center3d: %f, %f, %f\n", center3d.x, center3d.y, center3d.z);
            printf("predict3d: %f, %f, %f\n", predict3d.x, predict3d.y, predict3d.z);
            //最小二乗法
            lsm_x->process((int)(center3d.x*1000)); //m to mm
            lsm_y->process((int)(center3d.y*1000));
            lsm_z->process((int)(center3d.z*1000));
            printf("x = %f t^2+ %f t+ %f\n", lsm_x->a, lsm_x->b, lsm_x->c);
            printf("y = %f t^2+ %f t+ %f\n", lsm_y->a, lsm_y->b, lsm_y->c);
            printf("z = %f t^2+ %f t+ %f\n", lsm_z->a, lsm_z->b, lsm_z->c);
            cout << endl;
        }else{
            lsm_X->tick();
            lsm_Y->tick();
            lsm_x->tick();
            lsm_y->tick();
            lsm_z->tick();
        }
        
    }
    
    
    sceneAnalyzer.GetFloor(plane);
    //cout << "point(" << plane.ptPoint.X << "," << plane.ptPoint.Y << "," << plane.ptPoint.Z << "), normal(" << plane.vNormal.X << "," << plane.vNormal.Y << "," << plane.vNormal.Z << ")" << endl ;
}

void* mainloop(void* args)
{
    init();
    while (waitKey(1) != 27 && status != false) {
        update();
    }
    status = false;
    cout << "ending @process thread" << endl;
    end();
    return (void*)NULL;
}

void finish()
{
    // called from graphic(main) thread
    status = false;
    
    int r = pthread_join(process_th, NULL);
    if (r != 0) {
        cout << "failed to join" << endl;
        exit(1);
    }
    cout << "ending @graphic thread" << endl;
    exit(0);
}

int main(int argc, char *argv[])
{
    int r;
    
    r = pthread_create(&process_th, NULL, mainloop, (void *)NULL);
    if (r!=0) {
        cout << "failed to create thread" << endl;
        exit(1);
    }
    //mainloop(NULL);
    graphic_main(argc, argv);
    
    return 0;
}

