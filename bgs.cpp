//
//  bgs.cpp
//  OrbitYosoku
//
//  Created by pheehs on 13/08/23.
//  Copyright (c) 2013年 Kph-lab. All rights reserved.
//
//ref: https://code.google.com/p/bgslibrary/
// This code is for Background Subtraction

#include "bgs.h"

BGS::BGS()
:threshold(50){} //閾値の初期値

void BGS::process(const cv::Mat &img_input, cv::Mat &img_output)
{
    if(img_input.empty())
        return;
    
    if(img_input_prev.empty()) {
        img_input.copyTo(img_input_prev);
        return;
    }
    
    cv::absdiff(img_input_prev, img_input, img_diff);
    
    if(img_diff.channels() == 3)
        cv::cvtColor(img_diff, img_diff, CV_RGB2GRAY);
    cv::threshold(img_diff, img_diff, threshold, 255, cv::THRESH_BINARY);
    
    img_diff.copyTo(img_output);
    img_input.copyTo(img_input_prev);
    
    return;
}
