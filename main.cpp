/*
 * Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
 */
#include <iostream>
#include <alproxies/alvideodeviceproxy.h>
#include <alproxies/almotionproxy.h>
#include <alvision/alvisiondefinitions.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <qi/os.hpp>


#define MINPITCH -0.671951
#define MAXPITCH 0.515047
#define TILT_JOINT "HeadPitch"
#define PAN_JOINT "HeadYaw"


void printErrorMsgAndExit();
void setTiltHead(char, bool&, double&, AL::ALMotionProxy&);
void updateTilt(int, double&);


int main(int argc, char** argv)
{
    std::string pip = "127.0.0.1";
    std::string video_path;
    std::string cameraType = "top";
    AL::ALValue ALimg;
    cv::Mat cvImg;
    cv::Size imgSize;
    cv::VideoWriter vw;
    int cameraFlag;
    char key;
    bool headset = false;
    double tilt_cmd = 0.0;
    double fps = 30.0;
    video_path = get_current_dir_name();
    video_path += "/capturedVideo.avi";


    // Check correctness of input arguments
    if(argc != 3 && argc != 4){
        printErrorMsgAndExit();
    }

    if(argc == 3){
        if(std::string(argv[1]) == "--pip"){
            pip = std::string(argv[2]);
        }
        else{
            printErrorMsgAndExit();
        }
    }

    if(argc == 4){
        if((std::string(argv[1]) == "--pip") &&
           (std::string(argv[3]) == "bottom" || std::string(argv[3]) == "top" )){
                pip = std::string(argv[2]);
                cameraType = std::string(argv[3]);
                cameraFlag = (cameraType == "top") ? (0) : (1);
                std::cout << "Camera chosen: " << cameraType << std::endl;
        }
        else{
            printErrorMsgAndExit();
        }
    }

    // Create NAO proxies for the required functionalities
    // ALMotion: set the stiffness of the body joints
    AL::ALMotionProxy motionProxy(pip);
    motionProxy.stiffnessInterpolation("Body", 1.0, 1.0); //// DOES THE STIFFNESS HAVE TO BE  1.0?????
    qi::os::sleep(2);

    // ALVideoDevice: instantiate structures for NAO camera image capturing
    AL::ALVideoDeviceProxy videoProxy(pip);
    std::string cameraName = videoProxy.subscribeCamera(cameraType,cameraFlag,AL::kQVGA, AL::kYuvColorSpace, fps);
    ALimg = videoProxy.getImageRemote(cameraName);
    imgSize = cv::Size((int)ALimg[0],(int)ALimg[1]);
    cvImg.create(imgSize,CV_8UC1);
    cvImg.data = (uchar*)ALimg[6].GetBinary();

    // Open the OpenCV video writer, to save all acquired images on a output video file
    vw.open(video_path,CV_FOURCC('M','J','P','G'),fps,imgSize,false);
    if(!vw.isOpened()){
        std::cout << "ERROR: Input video not found or failure in opening." << std::endl;
        std::exit(1);
    }


    // Run the main loop: first the pitch of the head is set, to find the best view for the camera in use.
    while(true){
        key = cv::waitKey(30);
        if((int)key == 27){
            break;
        }
        ALimg = videoProxy.getImageRemote(cameraName);
        if(!headset){
            cv::imshow("Image",cvImg);
            setTiltHead(key,headset,tilt_cmd,motionProxy);
            std::cout << "tilt_cmd: " << tilt_cmd << std::endl;
        }
        else{
            vw << cvImg;
            cv::imshow("Recording Image",cvImg);
        }

    }

    // Release all the open activities before closing the application
    std::cout << "Releasing camera ..." << std::endl;
    videoProxy.unsubscribe(cameraName);
    cv::destroyAllWindows();
    std::cout << "Releasing the Video Writer ... " << std::endl;
    vw.release();

    return 0;
}


void setTiltHead(char key, bool& headset, double& tilt_cmd, AL::ALMotionProxy& motionProxy){
    int dtilt = 0;

    switch(key){
    case('u'):
        dtilt = 1;
        break;
    case('d'):
        dtilt = -1;
        break;
    case('f'):
        headset = true;
        cv::destroyAllWindows();
        break;
    default:
        dtilt = 0;
        break;
    }

    updateTilt(dtilt,tilt_cmd);

    double pan = 0.0;

    AL::ALValue names = AL::ALValue::array(PAN_JOINT,TILT_JOINT);
    AL::ALValue angles = AL::ALValue::array(pan,tilt_cmd);
    double fracSpeed = 0.2;

    motionProxy.setAngles(names,angles,fracSpeed);
}

void updateTilt(int dtilt, double& tilt_cmd){
    double delta = 0.1;

    tilt_cmd += (double)dtilt*delta;

    tilt_cmd = (tilt_cmd < MAXPITCH) ? ((tilt_cmd > MINPITCH) ? (tilt_cmd) : (MINPITCH)) : (MAXPITCH) ;

}

void printErrorMsgAndExit(){
    std::cout << "ERROR: wrong number of arguments.\n"
                 "\tUsage: cameraVideoCapture --pip [nao_ip] (default camera is top)"
                 "\n\tUsage: cameraVideoCapture --pip [nao_ip] [bottom/top]" << std::endl ;

    std::exit(1);
}
