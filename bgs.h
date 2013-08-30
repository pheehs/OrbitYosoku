//
//  bgs.h
//  OrbitYosoku
//
//  Created by pheehs on 13/08/23.
//  Copyright (c) 2013年 Kph-lab. All rights reserved.
//
//ref: https://code.google.com/p/bgslibrary/

#ifndef __OrbitYosoku__bgs__
#define __OrbitYosoku__bgs__

#include <opencv2/opencv.hpp>

class BGS
{
private:
    cv::Mat img_input_prev;
    cv::Mat img_diff;
public:
    int threshold;
    BGS();
    void process(const cv::Mat &img_input, cv::Mat &img_output);
};

#endif /* defined(__OrbitYosoku__bgs__) */
