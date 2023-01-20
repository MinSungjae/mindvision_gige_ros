#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

#include "CameraApi.h" //相机SDK的API头文件

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdio.h>

using namespace cv;

#define NUM_CAMERAS 2

unsigned char *g_pRgbBuffer[NUM_CAMERAS];     //处理后数据缓存区

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mindvision_gige_node");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Publisher cam1_img_pub = it.advertise("mindvision1/image", 1);
    image_transport::Publisher cam2_img_pub = it.advertise("mindvision2/image", 1);
    ros::Publisher cam1_info_pub = nh.advertise<sensor_msgs::CameraInfo>("mindvision1/camera_info", 1);
    ros::Publisher cam2_info_pub = nh.advertise<sensor_msgs::CameraInfo>("mindvision2/camera_info", 1);

    int                     iCameraCounts = NUM_CAMERAS;
    int                     iStatus=-1;
    tSdkCameraDevInfo       tCameraEnumList[NUM_CAMERAS];
    int                     hCamera[NUM_CAMERAS];
    tSdkCameraCapbility     tCapability[NUM_CAMERAS];      //设备描述信息
    tSdkFrameHead           sFrameInfo[NUM_CAMERAS];
    BYTE*			        pbyBuffer[NUM_CAMERAS];
    int                     iDisplayFrames = 10000;
    // IplImage *iplImage = NULL;
    int                     channel=3;

    CameraSdkInit(1);

    //枚举设备，并建立设备列表
    iStatus = CameraEnumerateDevice(tCameraEnumList,&iCameraCounts);
	printf("state = %d\n", iStatus);

	printf("count = %d\n", iCameraCounts);
    //没有连接设备
    if(iCameraCounts==0){
        return -1;
    }

    //相机初始化。初始化成功后，才能调用任何其他相机相关的操作接口
    for(int cam = 0; cam < iCameraCounts; cam++)
    {
        iStatus = CameraInit(&tCameraEnumList[cam],-1,-1,&hCamera[cam]);

        //初始化失败
        printf("state = %d\n", iStatus);
        if(iStatus!=CAMERA_STATUS_SUCCESS){
            return -1;
        }

        //获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数
        CameraGetCapability(hCamera[cam],&tCapability[cam]);   

        //
        g_pRgbBuffer[cam] = (unsigned char*)malloc(tCapability[cam].sResolutionRange.iHeightMax*tCapability[cam].sResolutionRange.iWidthMax*3);
        //g_readBuf = (unsigned char*)malloc(tCapability.sResolutionRange.iHeightMax*tCapability.sResolutionRange.iWidthMax*3);

        /*让SDK进入工作模式，开始接收来自相机发送的图像
            数据。如果当前相机是触发模式，则需要接收到
            触发帧以后才会更新图像。    */
        CameraPlay(hCamera[cam]);
        CameraSetOnceWB(hCamera[cam]);

        /*其他的相机参数设置
            例如 CameraSetExposureTime   CameraGetExposureTime  设置/读取曝光时间
            CameraSetImageResolution  CameraGetImageResolution 设置/读取分辨率
            CameraSetGamma、CameraSetConrast、CameraSetGain等设置图像伽马、对比度、RGB数字增益等等。
            更多的参数的设置方法，，清参考MindVision_Demo。本例程只是为了演示如何将SDK中获取的图像，转成OpenCV的图像格式,以便调用OpenCV的图像处理函数进行后续开发
        */
        double	m_fExpLineTime=0;//当前的行曝光时间，单位为us

        if(CameraSetAeState(hCamera[cam], FALSE) != 0)
            ROS_INFO("Manual exposure set failed...");
        CameraSetAnalogGain(hCamera[cam], 0.75);
        CameraGetExposureLineTime(hCamera[cam], &m_fExpLineTime);
        CameraSetExposureTime(hCamera[cam], 60000);
        
        // CameraSet

        if(tCapability[cam].sIspCapacity.bMonoSensor){
            channel=1;
            CameraSetIspOutFormat(hCamera[cam],CAMERA_MEDIA_TYPE_MONO8);
        }else{
            channel=3;
            CameraSetIspOutFormat(hCamera[cam],CAMERA_MEDIA_TYPE_BGR8);
        }
    }

    sensor_msgs::CameraInfo cam1_info, cam2_info;
    tSdkImageResolution info1, info2;
    CameraGetImageResolution(hCamera[0], &info1);
    CameraGetImageResolution(hCamera[1], &info2);

    cam1_info.header.seq = info1.iIndex;
    cam1_info.width = info1.iWidth;
    cam1_info.height = info1.iHeight;

    cam2_info.header.seq = info2.iIndex;
    cam2_info.width = info2.iWidth;
    cam2_info.height = info2.iHeight;

    // for(int d = 0; d < 5; d++)
    // {
    //     cam1_info.D.at(d) = cam1_dist_coff[d];
    //     cam2_info.D.at(d) = cam2_dist_coff[d];
    // }

    

    sensor_msgs::ImagePtr msg[NUM_CAMERAS];
    while (ros::ok())
    {
        for (int cam = 0; cam < iCameraCounts; cam++)
        {
            if (CameraGetImageBuffer(hCamera[cam], &sFrameInfo[cam], &pbyBuffer[cam], 1000) == CAMERA_STATUS_SUCCESS)
            {
                CameraImageProcess(hCamera[cam], pbyBuffer[cam], g_pRgbBuffer[cam], &sFrameInfo[cam]);

                cv::Mat matImage(
                    cvSize(sFrameInfo[cam].iWidth, sFrameInfo[cam].iHeight),
                    sFrameInfo[cam].uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3,
                    g_pRgbBuffer[cam]
                );

                msg[cam] = cv_bridge::CvImage(std_msgs::Header(), "bgr8", matImage).toImageMsg();
                // imshow("Opencv Demo", matImage);

                // waitKey(5);

                // 在成功调用CameraGetImageBuffer后，必须调用CameraReleaseImageBuffer来释放获得的buffer。
                // 否则再次调用CameraGetImageBuffer时，程序将被挂起一直阻塞，直到其他线程中调用CameraReleaseImageBuffer来释放了buffer
                CameraReleaseImageBuffer(hCamera[cam], pbyBuffer[cam]);
            }
        }
        msg[0]->header.frame_id = cam1_info.header.frame_id = "mindvision1";
        cam1_info.header.stamp = cam1_info.header.stamp;
        msg[1]->header.frame_id = cam2_info.header.frame_id = "mindvision2";
        cam2_info.header.stamp = cam2_info.header.stamp;

        int cam1_width, cam1_height, cam2_width, cam2_height;
        double cam1_matrix[4], cam2_matrix[4];
        double cam1_dist_coff[5], cam2_dist_coff[5];
        CameraGetUndistortParams(hCamera[0], &cam1_width, &cam1_height, cam1_matrix, cam1_dist_coff);
        CameraGetUndistortParams(hCamera[1], &cam2_width, &cam2_height, cam2_matrix, cam2_dist_coff);

        cam1_matrix[0] = 2630.8;
        cam1_matrix[1] = 2632.5;
        cam1_matrix[2] = 2108.1;
        cam1_matrix[3] = 1524.7;

        cam2_matrix[0] = 200;
        cam2_matrix[1] = 100;
        cam2_matrix[2] = cam2_info.width/2;
        cam2_matrix[3] = cam2_info.height/2;

        cam1_info.K.at(0) = cam1_matrix[0];
        cam1_info.K.at(2) = cam1_matrix[2];
        cam1_info.K.at(4) = cam1_matrix[1];
        cam1_info.K.at(5) = cam1_matrix[3];

        cam2_info.K.at(0) = cam2_matrix[0];
        cam2_info.K.at(2) = cam2_matrix[2];
        cam2_info.K.at(4) = cam2_matrix[1];
        cam2_info.K.at(5) = cam2_matrix[3];

        cam1_info.P.at(0) = cam1_matrix[0];
        cam1_info.P.at(2) = cam1_matrix[2];
        cam1_info.P.at(5) = cam1_matrix[1];
        cam1_info.P.at(6) = cam1_matrix[3];

        cam2_info.P.at(0) = cam2_matrix[0];
        cam2_info.P.at(2) = cam2_matrix[2];
        cam2_info.P.at(5) = cam2_matrix[1];
        cam2_info.P.at(6) = cam2_matrix[3];
        
        cam1_img_pub.publish(msg[0]);
        cam2_img_pub.publish(msg[1]);
        cam1_info_pub.publish(cam1_info);
        cam2_info_pub.publish(cam2_info);
    }

    for(int cam = 0; cam < iCameraCounts; cam++)
        CameraUnInit(hCamera[cam]);
    
    //注意，现反初始化后再free
    for(int cam = 0; cam < NUM_CAMERAS; cam++)
        free(g_pRgbBuffer[cam]);

    return 0;
}

