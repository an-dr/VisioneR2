// Copyright (c) 2024 Andrei Gramakov. All rights reserved.

#include <stdio.h>
#include "ulog.h"
#include <linux/videodev2.h>
#include <opencv2/opencv.hpp>
#include <unistd.h>
#include "Remotion.hpp"
#include <string.h>

static void write_img(char *buffer, int width, int height,
                      std::string filename)
{
    cv::Mat img(cv::Size(width, height), CV_8UC2, buffer);
    cv::cvtColor(img, img, cv::COLOR_YUV2BGR_YUYV);
    cv::imwrite(filename, img);
}

int main(int argc, char *argv[])
{
    // Set parameters
    const int width = 1280;
    const int height = 720;
    const int fps = 30;
    const unsigned int format = V4L2_PIX_FMT_YUYV;
    const std::string video_dev = "/dev/video0";
    const std::string port_name = "/dev/ttyUSB0";

    // Create and init Remotion object
    Remotion r;
    r.start(port_name, video_dev, format, width, height, fps);
    auto buffer_size = r.getVideoCaptureBufferSize();
    char *buffer = new char[buffer_size];

    // Set different expressions
    r.setExpression(RemotionExpression::CONFUSED);
    sleep(1);
    r.setExpression(RemotionExpression::CALM);
    sleep(1);
    r.setExpression(RemotionExpression::HAPPY);
    sleep(1);

    log_info("main() started");
    r.readVideoFrame(buffer, buffer_size);
    write_img(buffer, width, height, "img.jpg");
    sleep(1 / fps);
    if (argc > 1)
    {
        printf("Hello, %s!\n", argv[1]);
    }
    else
    {
        printf("Hello!\n");
    }
    return 0;
}
