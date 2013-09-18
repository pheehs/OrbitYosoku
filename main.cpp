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
    namedWindow ("Target", CV_WINDOW_AUTOSIZE);

    camera_test();
    bgs_rgb = new BGS(getImage, 1.0/200.0, 1.0/1000.0, 50.0);
    bgs_depth = new BGS(getDepth, 1.0/10000.0, 1.0/50000.0, 70.0);
    //createTrackbar("rgb_thresh", "Mask", &bgs_rgb->B_PARAM, 100);
    //createTrackbar("depth_thresh", "Mask", &bgs_depth->T_PARAM, 100);
    
    lsm_XY = new LSM;
    register_target();
}

void end()
{
    destroyAllWindows();
    context.Release();
    delete bgs_rgb;
    delete bgs_depth;
    delete lsm_XY;
}
Mat getImage(){
    imageGenerator.WaitAndUpdateData();
    imageGenerator.GetMetaData(imageMD);
    memcpy(image.data,imageMD.Data(),image.step * image.rows);    //イメージデータを格納
    cvtColor(image, image, CV_BGR2RGB);     //BGRをRGBに
    return image;
}

Mat getDepth(){
    cv::vector<cv::Mat> channels;
    cv::Mat zeroch; //深度画像3ch
    zeroch = Mat::zeros(KINECT_HEIGHT, KINECT_WIDTH, CV_8UC1);
    Mat depth_3ch;
    
    depthGenerator.WaitAndUpdateData();
    depthGenerator.GetMetaData(depthMD);
    memcpy(depth.data,depthMD.Data(),depth.step * depth.rows);    //イメージデータを格納
    depth.convertTo(depth_8UC1, CV_8UC1); //深度画像を16bit->8bitに
    
    channels.push_back(zeroch);
    channels.push_back(zeroch);
    channels.push_back(depth_8UC1);
    merge(channels, depth_3ch);

    return depth_3ch;
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
float at_float(Mat mat, int y, int x) {
    return ((float*)(mat.data + mat.step.p[0]*y))[x];
}
void convertRealToProjective(Point3f &src, Point2f &dest) {
    static XnPoint3D proj;
    proj.X = (XnFloat)src.x;
    proj.Y = (XnFloat)src.y;
    proj.Z = (XnFloat)src.z;
    //printf("real(%f,%f,%f)  ->  ",proj.X,proj.Y,proj.Z);
    depthGenerator.ConvertRealWorldToProjective(1, &proj, &proj);
    //printf("proj(%f,%f,%f)\n",proj.X,proj.Y,proj.Z);
    dest.x = proj.X;
    dest.y = proj.Y;
}
//放物線計算用の座標変換
bool retrieveForParabola(){
    const float AXIS_THRESH = 10.0;
    Xaxis_prev = Xaxis.clone();
    Yaxis_prev = Yaxis.clone();
    Zaxis_prev = Zaxis.clone();
    
    cout << "axis_prev: " << Xaxis_prev << Yaxis_prev << Zaxis_prev << endl;
    cout << "axis: " << Xaxis << Yaxis << Zaxis << endl;
    // Y軸は床の法線ベクトルと平行に（鉛直方向)
    printf("plane.vNormal: (%f, %f, %f)\n", plane.vNormal.X,plane.vNormal.Y,plane.vNormal.Z);
    if (plane.vNormal.X == 0 && plane.vNormal.Y == 0 && plane.vNormal.Z == 0) { //床を検出できてない→鉛直方向(Y軸)がわからない
        cout << "<!> Floor plane is not recognized!" << endl;
        Yaxis.at<float>(0, 0) = 0;  //とりあえずデフォルトと同じ方向にしておく(errorにしてもよい)
        Yaxis.at<float>(1, 0) = 1;
        Yaxis.at<float>(2, 0) = 0;
    }else {
        float floor_len = sqrt(plane.vNormal.X*plane.vNormal.X + plane.vNormal.Y*plane.vNormal.Y + plane.vNormal.Z*plane.vNormal.Z);
        Yaxis.at<float>(0,0) = plane.vNormal.X / floor_len;
        Yaxis.at<float>(1,0) = plane.vNormal.Y / floor_len;
        Yaxis.at<float>(2,0) = plane.vNormal.Z / floor_len;
    }
    // Z軸は速度ベクトル２つに垂直なベクトルと平行（放物線を含む平面に垂直）
    //cout << "center3d: " << center3d << center3d_prev <<endl;
    if (center3d_prev.x == 0 && center3d_prev.y == 0 && center3d_prev.z == 0) { //この瞬間の重心は取得してるがいっこまえのフレームでは取得してない
        return true; //放物運動はこれから
    }
    cout << "vvector: " << vvector << endl;
    vvector.at<float>(0, 0) = (center3d.x - center3d_prev.x)*1000;
    vvector.at<float>(1, 0) = (center3d.y - center3d_prev.y)*1000;
    vvector.at<float>(2, 0) = (center3d.z - center3d_prev.z)*1000;
    if (Xaxis_prev.dot(vvector) < 0) { //YZ平面に対してX軸（今までの進行方向）と逆向きの速度ベクトルの場合（X軸が↑0のときを含まない）
        cout << "[*] opposite vvector!" << endl<<endl;
        return false; //放物線の運動は終了している
    }
    if (at_float(vvector,0,0)==0 && at_float(vvector, 1, 0)==0 && at_float(vvector, 2, 0)==0) //速度ベクトルがゼロベクトル
        return false; //物体は静止している
    Zaxis = Yaxis.cross(vvector);
    if (at_float(Zaxis, 0, 0) == 0 && at_float(Zaxis, 1, 0) == 0 && at_float(Zaxis, 2, 0)) { //２つの速度ベクトルが平行
        return true; //放物線の運動は続いている可能性がある(放物線の一部を微視的に見れば直線っぽくなる)
    }
    Zaxis /= sqrt( at_float(Zaxis,0,0)*at_float(Zaxis,0,0) + at_float(Zaxis,1,0)*at_float(Zaxis,1,0) + at_float(Zaxis,2,0)*at_float(Zaxis,2,0));
    
    // X軸はY,Z軸に垂直で進行方向と同じ向き
    Xaxis = Zaxis.cross(Yaxis);
    Xaxis /= sqrt( at_float(Xaxis,0,0)*at_float(Xaxis,0,0) + at_float(Xaxis,1,0)*at_float(Xaxis,1,0) + at_float(Xaxis,2,0)*at_float(Xaxis,2,0));
    if (Xaxis.dot(vvector) < 0)
        Xaxis *= -1;
    // 軸のズレを確認
    Mat Xabsdiff, Yabsdiff,Zabsdiff,Xmaxdiff, Ymaxdiff, Zmaxdiff;
    float Xmax,Ymax,Zmax;
    absdiff(Xaxis, Xaxis_prev, Xabsdiff);
    absdiff(Yaxis, Yaxis_prev, Yabsdiff);
    absdiff(Zaxis, Zaxis_prev, Zabsdiff);
    reduce(Xabsdiff, Xmaxdiff, 0,CV_REDUCE_MAX);
    reduce(Yabsdiff, Ymaxdiff, 0,CV_REDUCE_MAX);
    reduce(Zabsdiff, Zmaxdiff, 0,CV_REDUCE_MAX);
    Xmax = at_float(Xmaxdiff,0,0);
    Ymax = at_float(Ymaxdiff,0,0);
    Zmax = at_float(Zmaxdiff,0,0);
    cout << "axis_prev: " << Xaxis_prev << Yaxis_prev << Zaxis_prev << endl;
    cout << "axis: " << Xaxis << Yaxis << Zaxis << endl;
    cout << "absdiff: " << Xabsdiff << Yabsdiff << Zabsdiff << endl;
    cout << "maxdiff: " << Xmaxdiff << Ymaxdiff << Zmaxdiff << endl;
    if (Xmax < AXIS_THRESH && Ymax < AXIS_THRESH && Zmax < AXIS_THRESH) {
        //座標変換用の行列
        Rotate = (Mat_<float>(3,3) <<
                  at_float(Xaxis,0,0),at_float(Xaxis,1,0),at_float(Xaxis,2,0),
                  at_float(Yaxis,0,0),at_float(Yaxis,1,0),at_float(Yaxis,2,0),
                  at_float(Zaxis,0,0),at_float(Zaxis,1,0),at_float(Zaxis,2,0) );
        Rotate_inv = Rotate.inv();
    }else {
        //軸が変動し過ぎ→放物線の運動は終わっている
        cout << "axis changes too much!" << endl;
        return false;
    }
    return true;
}
Point3f convertRealToParabolic(Point3f real) {
    Mat realmat = (Mat_<float>(3,1) << real.x,real.y,real.z);
    Mat convertedmat = Rotate * realmat;
    Point3f converted(convertedmat.at<float>(0,0)*1000,convertedmat.at<float>(1, 0)*1000,convertedmat.at<float>(2, 0)*1000); //m to mm
    /*if (converted.x/1000 != Rotate.at<float>(0,0)*real.x+Rotate.at<float>(0,1)*real.y+Rotate.at<float>(0,2)*real.z)
        cout << "r->p invalid x" <<endl;
    if (converted.y/1000 != Rotate.at<float>(1,0)*real.x+Rotate.at<float>(1,1)*real.y+Rotate.at<float>(1,2)*real.z)
        cout << "r->p invalid y" <<endl;
    if (converted.z/1000 != Rotate.at<float>(2,0)*real.x+Rotate.at<float>(2,1)*real.y+Rotate.at<float>(2,2)*real.z)
        cout << "r->p invalid z" <<endl;*/
    return converted;
}
Point3f convertParabolicToReal(Point3f para) {
    Mat paramat = (Mat_<float>(3,1) << para.x,para.y,para.z);
    Mat convertedmat = Rotate_inv * paramat;
    Point3f converted(convertedmat.at<float>(0,0)/1000,convertedmat.at<float>(1, 0)/1000,convertedmat.at<float>(2, 0)/1000); //mm to m
    /*if (converted.x != (Rotate_inv.at<float>(0,0)*para.x+Rotate_inv.at<float>(0,1)*para.y+Rotate_inv.at<float>(0,2)*para.z)/1000)
        cout << "p->r invalid x" <<endl;
    if (converted.y != (Rotate_inv.at<float>(1,0)*para.x+Rotate_inv.at<float>(1,1)*para.y+Rotate_inv.at<float>(1,2)*para.z)/1000)
        cout << "p->r invalid y" <<endl;
    if (converted.z != (Rotate_inv.at<float>(2,0)*para.x+Rotate_inv.at<float>(2,1)*para.y+Rotate_inv.at<float>(2,2)*para.z)/1000)
        cout << "p->r invalid z" <<endl;*/
    return converted;
}
bool all_zero(Mat mat) {
    for (int i = 0; i < mat.rows; i++) {
        const float* mati = mat.ptr<float>(i);
        for (int j=0; j < mat.cols; j++){
            if (mati[j] != 0)
                return false;
        }
    }
    return true;
}

Mat getMask() {
    Mat image_mask;
    Mat depth_mask;
    Mat mask;
    
    //RGB画像と深度画像でそれぞれ背景差分
    vector<Mat> channels;
    Mat zeroch = Mat::zeros(KINECT_HEIGHT, KINECT_WIDTH, CV_8UC1);
    Mat depth_3ch; //深度画像3ch
    depth.convertTo(depth_8UC1, CV_8UC1); //深度画像を16bit->8bitに
    channels.push_back(zeroch);
    channels.push_back(zeroch);
    channels.push_back(depth_8UC1);
    merge(channels, depth_3ch);
    
    bgs_rgb->process(image, image_mask);
    bgs_depth->process(depth_3ch, depth_mask);
    
    //imshow("RGBImage", image);
    //imshow("DepthImage", depth);

    // RGBと深度の両方で検出された領域を抽出
    bitwise_or(image_mask, depth_mask, mask);
    // 収縮と膨張でノイズな領域を削除。「重要な処理」
    erode(mask, mask, Mat());
    dilate(mask, mask, Mat());
    dilate(mask, mask, Mat());
    erode(mask, mask, Mat());
    bitwise_and(image_mask, mask, mask);
    // 収縮と膨張でノイズな領域を削除。「重要な処理」
    erode(mask, mask, Mat());
    dilate(mask, mask, Mat());
    dilate(mask, mask, Mat());
    erode(mask, mask, Mat());
    
    if (!image_mask.empty()) {
        flip(image_mask, image_mask, 1);
        imshow("RGBMask", image_mask);
    }
    if (!depth_mask.empty()) {
        flip(depth_mask, depth_mask, 1);
        imshow("DepthMask", depth_mask);
    }
    
    return mask;
}
void camera_test() {
    Mat centroid;
    cout << "camera test" << endl;
    while (status != false) {
        context.WaitAndUpdateAll();
        
        imageGenerator.GetMetaData(imageMD);
        
        memcpy(image.data,imageMD.Data(),image.step * image.rows);    //イメージデータを格納
        
        cvtColor(image, hsvimage, CV_BGR2HSV_FULL);	//HSV画像作成
        cvtColor(image, image, CV_BGR2RGB);     //BGRをRGBに
        
        flip(image, centroid, 1);
        putText(centroid, getFPS(), cvPoint(2, 28), FONT_HERSHEY_SIMPLEX, 1, Scalar(0,0,200), 2, CV_AA);
        imshow("Centroid", centroid);
        
    }
    status = true;
}
Point2f flip_point2f(Point2f point) {
    Point2f result(KINECT_WIDTH - point.x, point.y);
    return result;
}
void register_target() {
    Mat centroid;
    cout << "register_target" <<endl;
    
    for (int i = 0; i < 2; i++){
        while (status != false) {
            context.WaitAndUpdateAll();
            
            imageGenerator.GetMetaData(imageMD);
            depthGenerator.GetMetaData(depthMD);
            sceneAnalyzer.GetFloor(plane);
            
            memcpy(image.data,imageMD.Data(),image.step * image.rows);    //イメージデータを格納
            memcpy(depth.data,depthMD.Data(),depth.step * depth.rows);    //深度データを格納
            
            cvtColor(image, hsvimage, CV_BGR2HSV_FULL);	//HSV画像作成
            cvtColor(image, image, CV_BGR2RGB);     //BGRをRGBに
            
            //3次元ポイントクラウドのための座標変換
            retrievePointCloudMap(depth,pointCloud_XYZ);
            flip(image, centroid, 1);
            putText(centroid, getFPS(), cvPoint(2, 28), FONT_HERSHEY_SIMPLEX, 1, Scalar(0,0,200), 2, CV_AA);
            
            Mat mask = getMask();
            if(!mask.empty()) {
                //　画面上での重心を計算
                Moments m = moments(mask, true);
                Point2f center2d(m.m10/m.m00, m.m01/m.m00);
                circle( centroid, flip_point2f(center2d), 8, Scalar(0,0,200), -1, 4, 0 );
                //現実座標での重心を計算
                center3d_prev = center3d;
                center3d = pointCloud_XYZ.at<Point3f>((int)center2d.y,(int)center2d.x);
                flip(mask, mask, 1);
                imshow("Mask", mask);
            }
            
            imshow("Centroid", centroid);
            
        }
        status = true;
        cout << "target" << i << ": " << center3d <<endl;
        target[i] = center3d;
    }
}
int is_crashing(Point3f objpoint) {
    const float RADIUS = 0.10;
    
    if (sqrt((target[0].x - objpoint.x)*(target[0].x - objpoint.x) + (target[0].y - objpoint.y)*(target[0].y - objpoint.y)+(target[0].z - objpoint.z)*(target[0].z - objpoint.z)) < RADIUS*2)
        return 0;
    if (sqrt((target[1].x - objpoint.x)*(target[1].x - objpoint.x) + (target[1].y - objpoint.y)*(target[1].y - objpoint.y)+(target[1].z - objpoint.z)*(target[1].z - objpoint.z)) < RADIUS*2)
        return 1;
    return -1;
}
void estimateDestination() {
    const float MIN_MOVE = 200;
    cout << "<<<<<---estimateDestination--->>>>>" <<endl;
    if (at_float(vvector, 0, 0)*at_float(vvector, 0, 0)+at_float(vvector, 1, 0)*at_float(vvector, 1, 0)+at_float(vvector, 2, 0)*at_float(vvector, 2, 0) < MIN_MOVE) { //物体が動いていないとき
        //-1
        imshow("Target", white_img);
        return;
    }
    Point3f paratarget, paracenter;
    Point3f paravvec(at_float(vvector,0,0), at_float(vvector, 1, 0), at_float(vvector, 2, 0));
    paravvec = convertRealToParabolic(paravvec);
    Mat totarget_xz = (Mat_<float>(2, 1));
    Mat vvec_xz = (Mat_<float>(2, 1) << paravvec.x,paravvec.z);
    cout << "vvec_xz: " << vvec_xz <<endl;
    paracenter = convertRealToParabolic(center3d);
    float vector_angle[2] = {0};
    
    for (int i=0; i < 2; i++) {
        paratarget = convertRealToParabolic(target[i]);
        totarget_xz.at<float>(0,0) = paratarget.x - paracenter.x;
        totarget_xz.at<float>(1,0) = paratarget.z - paracenter.z;
        cout << "totarget_xz" << i << ": " <<totarget_xz <<endl;

        vector_angle[i] = totarget_xz.dot(vvec_xz)
        / sqrt(at_float(totarget_xz,0,0)*at_float(totarget_xz,0,0)+at_float(totarget_xz,1,0)*at_float(totarget_xz,1,0)) //length of totarget_xz
        / sqrt(at_float(vvec_xz, 0, 0)*at_float(vvec_xz, 0, 0)+at_float(vvec_xz, 1, 0)*at_float(vvec_xz, 1, 0)); //length of vvec_xz
        cout << "vector_angle" <<i<< ": " <<vector_angle[i] <<endl;
    }
    if (vector_angle[0] > vector_angle[1]) {
        //0
        imshow("Target", yellow_img);
        //lightLED(0);
    } else if (vector_angle[0] < vector_angle[1]) {
        //1
        imshow("Target", blue_img);
        // lightLED(1);
    } else{
        //-1
        imshow("Target", white_img);
    }
}
void update()
{
    int target_n;
    Mat centroid;
    context.WaitAndUpdateAll();
    
    imageGenerator.GetMetaData(imageMD);
    depthGenerator.GetMetaData(depthMD);
    sceneAnalyzer.GetFloor(plane);
    
    memcpy(image.data,imageMD.Data(),image.step * image.rows);    //イメージデータを格納
    memcpy(depth.data,depthMD.Data(),depth.step * depth.rows);    //深度データを格納
    
    cvtColor(image, hsvimage, CV_BGR2HSV_FULL);	//HSV画像作成
    cvtColor(image, image, CV_BGR2RGB);     //BGRをRGBに
    
    //3次元ポイントクラウドのための座標変換
    retrievePointCloudMap(depth,pointCloud_XYZ);
    
    Mat mask = getMask();
    
    flip(image, centroid, 1);

    if (!mask.empty()) {
        
        //　画面上での重心を計算
        Moments m = moments(mask, true);
        Point2f center2d(m.m10/m.m00, m.m01/m.m00);
        //現実座標での重心を計算
        center3d_prev = center3d;
        center3d = pointCloud_XYZ.at<Point3f>((int)center2d.y,(int)center2d.x);
            
        if (center3d.x!=0 && center3d.y!=0 && center3d.z!=0 //おそらくありえない座標なのでリセット用に使う
            && retrieveForParabola()) { //放物運動が続いてるか？　→放物線座標系の変換行列を求める
            center3d_graphic = center3d;
            if (!all_zero(Rotate)) { //放物運動が続いていても、始まったばかりで変換行列がまだの時はスルー
                estimateDestination();
                predict3d = convertParabolicToReal(lsm_XY->predict());
                
                /// 重心（赤）と予想した重心（青）の描画
                Point2f predict2d;
                convertRealToProjective(predict3d, predict2d);
                circle( centroid, flip_point2f(center2d), 8, Scalar(0,0,200), -1, 4, 0 );
                //circle( centroid, flip_point2f(predict2d), 8, Scalar(200,0,0), -1, 4, 0 );
                
                // 画面上での予測軌道を描画
                Point3f orbit3d;
                Point2f orbit2d;
                //cout << "orbit::" <<endl;
                
                /*for (int i=1; i <= 100; i++) {
                    //0.01sec毎の軌跡
                    orbit3d = convertParabolicToReal(lsm_XY->predict(lsm_XY->t+1*i));
                    if (lsm_XY->a == 0 && lsm_XY->c == 0 && lsm_XY->d == 0) { //データが１点しか取れてない
                        break;
                    }
                    orbit3d_list[i-1] = orbit3d;
                    if ((target_n = is_crashing(orbit3d)) != -1) {
                        crashing[i-1] = target_n;
                    } else {
                        crashing[i-1] = -1;
                    }
                    ostringstream os;
                    os << target_n;
                    
                    putText(centroid, os.str(), cvPoint(2, KINECT_WIDTH-20), FONT_HERSHEY_SIMPLEX, 1, Scalar(200,0,200), 2, CV_AA);
                    
                    convertRealToProjective(orbit3d, orbit2d);
                    circle( centroid, flip_point2f(orbit2d), 5, Scalar(0,200,0), -1, 4, 0 );
                    
                }*/
                //画面外か、1sec以上経過で終了
                
                printf("center3d: %f, %f, %f\n", center3d.x, center3d.y, center3d.z);
                printf("predict3d: %f, %f, %f\n", predict3d.x, predict3d.y, predict3d.z);
                //最小二乗法
                lsm_XY->process(convertRealToParabolic(center3d));
                cout << endl;
            }
        }else{ //物体を見失ったとき
            //LSMをリセット
            lsm_XY->tick();
            //さっきなで追跡してた物体に関する情報をリセット
            center3d = Point3f(0,0,0);
            vvector.zeros(3, 1, CV_32FC1);
            imshow("Target", white_img);
        }
        flip(mask, mask, 1);
        imshow("Mask", mask);
    }else{ //マスクが何故か空のとき
        center3d = Point3f(0,0,0);
        vvector.zeros(3, 1, CV_32FC1);
    }
    putText(centroid, getFPS(), cvPoint(2, 28), FONT_HERSHEY_SIMPLEX, 1, Scalar(0,0,200), 2, CV_AA);
    imshow("Centroid", centroid);
    
}

void* mainloop(void* args)
{
    init();
    while (status != false) {
        update();
    }
    status = false;
    running = false;
    cout << "ending @process thread" << endl;
    end();
    return (void*)NULL;
}

void finish()
{
    // called from graphic(main) thread
    status = false;
    running = false;
    
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
    graphic_main(argc, argv);
    
    return 0;
}

