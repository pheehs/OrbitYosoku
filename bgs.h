//
//  bgs.h
//  OrbitYosoku
//
//  Created by pheehs on 13/08/23.
//  Copyright (c) 2013年 Kph-lab. All rights reserved.
//
//ref: http://opencv.jp/sample/accumulation_of_background.html

#ifndef __OrbitYosoku__bgs__
#define __OrbitYosoku__bgs__

class BGS
{
private:
    int INIT_TIME;
    cv::Mat av_img, sgm_img, tmp_img, lower_img, upper_img;
    cv::Mat dst_img, msk_img;
public:
    double B_PARAM;
    double T_PARAM;
    double Zeta;

    BGS(cv::Mat (*getFrame)(),float, float, float);

    void process(const cv::Mat &img_input, cv::Mat &img_output);
};

#endif /* defined(__OrbitYosoku__bgs__) */
