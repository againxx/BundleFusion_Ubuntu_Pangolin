/**
 * BundleFusionKinectV2
 * This is a adaptation of BundleFusion to work with live data from the Kinect V2
 *   on Linux machines.
 * @file: BundleFusionKinectV2.cpp
 * @author: Alan Cheng (aldcheng)
 * 
 * Notes: Original BundleFusion Linux modification: https://github.com/FangGet/BundleFusion_Ubuntu_Pangolin
 */

#include <iostream>
#include <string>
#include <fstream>
#include <BundleFusion.h>
#include <dirent.h>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <GlobalAppState.h>
#include <unistd.h>
#include <sys/time.h>

// LibFreenect2 Interface
#include "imageStream.h"
#include "image_stream_kinectv1.h"

int main ( int argc, char** argv )
{
    if ( argc != 3 )
    {
        std::cout<<"usage: ./bundle_fusion_example /path/to/zParametersDefault.txt /path/to/zParametersBundlingDefault.txt"<<std::endl;
        return -1;
    }

    std::string app_config_file = "";
    std::string bundle_config_file = "";

    app_config_file = std::string ( argv[1] );
    bundle_config_file = std::string ( argv[2] );

    // step 1: init all resources
    if ( !initBundleFusion ( app_config_file, bundle_config_file ) )
    {
        std::cerr<<"BundleFusion init failed, exit." << std::endl;
        return -1;
    }

    // ImageStream streamer;
    Freenect::Freenect freenect;
    ImageStreamKinectV1& streamer = freenect.createDevice<ImageStreamKinectV1>(0);

    streamer.startVideo();
    streamer.startDepth();

    cv::Mat rgb, depth;

    // Main loop using live data
    while(true) {
        if (!streamer.new_depth_frame_ || !streamer.new_rgb_frame_) {
            std::cout << "Run out of frames" << '\n';
            continue;
        } else {
            cv::imshow("RGB", streamer.getVideo(rgb));
            cv::imshow("Depth", streamer.getDepth(depth));

            if ( processInputRGBDFrame ( rgb, depth ) )
            {
                std::cout<<"\tSuccess! frame added into BundleFusion." << std::endl;
            }
            else
            {
                std::cout<<"\tFailed! frame not added into BundleFusion." << std::endl;
            }
        
        if (cv::waitKey(1) == 27) break;
        }
    }

    deinitBundleFusion();

    return 0;
}
