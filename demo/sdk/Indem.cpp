#include <iostream>
#define _CRT_SECURE_NO_WARNING
#include "imrsdk.h"
#include <thread>
#ifdef WIN32
#include <Windows.h>
#endif
#include <mutex>
#include <future>
#include <stdio.h>

struct ImrDepthImageTarget
{
    float _cubesize;
    int _image_w;
    int _image_h;
    float* _deepptr;
};

void PoseCallback(int, void* pData, void* pParam) {
	ImrModulePose* pHeadPose = (ImrModulePose*)pData;
	//检查两次之间的间隔距离平方应当小于5cm
	static double sdX = 0, sdY = 0, sdZ = 0;
	double dx = (pHeadPose->_pose._position[0] - sdX);
	double dy = (pHeadPose->_pose._position[1] - sdY);
	double dz = (pHeadPose->_pose._position[2] - sdZ);
	double ds2 = dx * dx + dy * dy + dz * dz;
	sdX = pHeadPose->_pose._position[0];
	sdY = pHeadPose->_pose._position[1];
	sdZ = pHeadPose->_pose._position[2];
}

void  SdkCameraCallBack(double time, unsigned char* pLeft, unsigned char* pRight, int width, int height, int channel, void* pParam)
{
	std::cout << "----------------- IMG Come: " << time<<" ----------------"<< std::endl;
}

void sdkImuCallBack(double time, float accX, float accY, float accZ, float gyrX, float gyrY, float gyrZ, void* pParam)
{
    std::cout << "IMU: " << time << "\t"
        << accX << "\t" << accY << "\t" << accZ << "\t"
        << gyrX << "\t" << gyrY << "\t" << gyrZ << "\t";
}

void sdkSLAMResult(int, void* pData, void* pParam)
{
    ImrModulePose* pose = (ImrModulePose*)pData;
	std::cout << "SLAM: "<<pose->_pose._position[0] << std::endl;
}

void DepthImageCallback(int ret, void* pData, void* pParam) {
    ImrDepthImageTarget* Depth = reinterpret_cast<ImrDepthImageTarget*>(pData);
	float maxp = 0;
	for (int mi = 0; mi < Depth->_image_w * Depth->_image_h; mi++)
	{
		//cout << elas.D1[mi] << endl;
		if (Depth->_deepptr[mi] > maxp)
			maxp = Depth->_deepptr[mi];
	}
	FILE* fp;
	char header[20];

	fp = fopen("Depth.dat", "wb");

	unsigned char* pImage = new unsigned char[Depth->_image_h*Depth->_image_w];
	for (int mv = 0; mv < Depth->_image_h; mv++)
	{
		for (int mu = 0; mu < Depth->_image_w; mu++)
		{
			//	cout << elas.D1[mv*img_W + mu] << endl;
			pImage[mv*Depth->_image_w + mu] = (unsigned char)fmax(Depth->_deepptr[mv*Depth->_image_w + mu] * 255.f / maxp, 0.f);
			//	cout << dep.at<uchar>(mv, mu) << endl;
			fprintf(fp,"%d " ,pImage[mv*Depth->_image_w + mu]);

		}
		fprintf(fp, "\n ");
	}


	fclose(fp);
	delete[] pImage;
}

void main(){
    using namespace indem;
    CIMRSDK* pSDK = new CIMRSDK();
    MRCONFIG config = { 0 };
    strcpy(config.slamPath, "slam.dll");
    //不开启slam
    config.bSlam = false;
    //SDK初始化
    pSDK->Init(config);
    //SDK图像数据回调
	pSDK->RegistModuleCameraCallback(SdkCameraCallBack,NULL);
   //SDK模组IMU数据回调
	pSDK->RegistModuleIMUCallback(sdkImuCallBack,NULL);
   //SDK的slam结算结果回调
	pSDK->RegistModulePoseCallback(sdkSLAMResult,NULL);
    //将DepthImageCallback设置为深度图回调
    pSDK->AddPluginCallback("depthimage", "depth", DepthImageCallback, NULL);
    std::this_thread::sleep_for(std::chrono::seconds(60 ));
    pSDK->Release();
    delete pSDK;
}
