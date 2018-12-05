#include <iostream>
#define _CRT_SECURE_NO_WARNING
#include "imrsdk.h"
#include <thread>
#ifdef WIN32
#include <Windows.h>
#endif
#include <mutex>
#include <future>
#include <cmath>
#include <cstring>
#include <stdio.h>
#include "map2D.h"

#include <opencv2\opencv.hpp>
struct ImrDepthImageTarget
{
    float _cubesize;
    int _image_w;
    int _image_h;
    float* _deepptr;
};

//void PrintModuleInfo(CIMRSDK* pSDK)
//{
//	ImrModuleDevice Info info = pSDK->GetModuleInfo();
//	std::cout << "Module Detail Info: \n"
//		<< "  ID: " << info._id << std::endl
//		<< "  Designed By: " << info._designer << std::endl
//		<< "  BaseLine: " << info._baseline << std::endl
//		<< "  Firmware Version: " << info._firmware_version << std::endl
//		<< "  Hardware Version: " << info._hardware_version << std::endl
//		<< "  IMU: " << info._imu << std::endl
//		<< "  Lens: " << info._lens << std::endl
//		<< "  View Angle: " << info._viewing_angle << std::endl;
//}

void PrintEach(int row, int col, double* ptr)
{
	for (int r = 0; r < row; ++r)
	{
		for (int c = 0; c<col; ++c)
		{
			std::cout << ptr[r*col + c] << "\t";
		}
		std::cout << std::endl;
	}
}

void PrintModuleParameters(CIMRSDK* pSDK)
{
	CameraCalibrationParameter params = pSDK->GetModuleParams();
	std::cout << "ACC: " << std::endl;
	PrintEach(3, 4, params._Acc);
	std::cout << "Gyr: " << std::endl;
	PrintEach(3, 4, params._Gyr);
	std::cout << "Dl: " << std::endl;
	PrintEach(4, 1, params._Dl);
	std::cout << "Dr: " << std::endl;
	PrintEach(4, 1, params._Dr);
	std::cout << "Kl: " << std::endl;
	PrintEach(3, 3, params._Kl);
	std::cout << "Kr: " << std::endl;
	PrintEach(3, 3, params._Kr);
	std::cout << "Pl: " << std::endl;
	PrintEach(3, 4, params._Pl);
	std::cout << "Pr: " << std::endl;
	PrintEach(3, 4, params._Pr);
	std::cout << "Rl: " << std::endl;
	PrintEach(3, 3, params._Rl);
	std::cout << "Rr: " << std::endl;
	PrintEach(3, 3, params._Rr);
	std::cout << "TSCl: " << std::endl;
	PrintEach(4, 4, params._TSCl);
	std::cout << "TSCr: " << std::endl;
	PrintEach(4, 4, params._TSCr);
	std::cout << "Baseline: " << params._baseline << " m" << std::endl;
	std::cout << "AMax: " << params._AMax << std::endl;
	std::cout << "SigmaAC: " << params._SigmaAC << std::endl;
	std::cout << "SigmaBa: " << params._SigmaBa << std::endl;
	std::cout << "GMax: " << params._GMax << std::endl;
	std::cout << "SigmaAwC: " << params._SigmaAwC << std::endl;
	std::cout << "SigmaBg: " << params._SigmaBg << std::endl;
	std::cout << "SigmaGC: " << params._SigmaGC << std::endl;
	std::cout << "SigmaGwC: " << params._SigmaGwC << std::endl;
}
void PoseCallback(int, void* pData, void* pParam) {
	ImrModulePose* pHeadPose = (ImrModulePose*)pData;
	//妫�鏌ヤ袱娆′箣闂寸殑闂撮殧璺濈骞虫柟搴斿綋灏忎簬5cm
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
		<< gyrX << "\t" << gyrY << "\t" << gyrZ << "\t" << std::endl;;
}

void sdkSLAMResult(int, void* pData, void* pParam)
{
    ImrModulePose* pose = (ImrModulePose*)pData;
	std::cout << "SLAM Pose: " << pose->_pose._position[0]<<" " \
		<< pose->_pose._position[1] << " "\
		<< pose->_pose._position[2] << " " << std::ends;
    std::cout << "SLAM Rotation: " << pose->_pose._oula[0] << " "\
		<<pose->_pose._oula[1] << " " \
		<<pose->_pose._oula[2] <<std::endl;
}

void DepthImageCallback(int ret, void* pData, void* pParam) {
    ImrDepthImageTarget* Depth = reinterpret_cast<ImrDepthImageTarget*>(pData);
	//float maxp = 0;
	//for (int mi = 0; mi < Depth->_image_w * Depth->_image_h; mi++)
	//{
	//	//cout << elas.D1[mi] << endl;
	//	if (Depth->_deepptr[mi] > maxp)
	//		maxp = Depth->_deepptr[mi];
	//}
	//FILE* fp;
	//char header[20];

	//fp = fopen("Depth.dat", "wb");

	//unsigned char* pImage = new unsigned char[Depth->_image_h*Depth->_image_w];
	//for (int mv = 0; mv < Depth->_image_h; mv++)
	//{
	//	for (int mu = 0; mu < Depth->_image_w; mu++)
	//	{
	//		//	cout << elas.D1[mv*img_W + mu] << endl;
	//		pImage[mv*Depth->_image_w + mu] = (unsigned char)fmax(Depth->_deepptr[mv*Depth->_image_w + mu] * 255.f / maxp, 0.f);
	//		//	cout << dep.at<uchar>(mv, mu) << endl;
	//		fprintf(fp,"%d " ,pImage[mv*Depth->_image_w + mu]);

	//	}
	//	fprintf(fp, "\n ");
	//}
	float maxp = 0;
	for (int mi = 0; mi < Depth->_image_w * Depth->_image_h; mi++)
	{
		//cout << elas.D1[mi] << endl;
		if (Depth->_deepptr[mi] > maxp)
			maxp = Depth->_deepptr[mi];
	}
	
	cv::Mat dep(Depth->_image_h, Depth->_image_w, CV_8UC1);
	for (int mv = 0; mv < Depth->_image_h; mv++)
	{
		for (int mu = 0; mu < Depth->_image_w; mu++)
		{ 
			//	cout << elas.D1[mv*img_W + mu] << endl;
			dep.at<uchar>(mv, mu) = (uchar)fmax(Depth->_deepptr[mv*Depth->_image_w + mu] * 255.f / maxp, 0.f);
			//	cout << dep.at<uchar>(mv, mu) << endl;
		}
	}
	cv::imshow("DepthImage.jpg", dep);
	cv::waitKey(1);


	//fclose(fp);
	//delete[] pImage;
}
CMap2D* pCMap;
int main(){
    using namespace indem;
    CIMRSDK* pSDK = new CIMRSDK();
    MRCONFIG config = { 0 };

    strcpy(config.slamPath, "slam.dll");
	//不开启slam    
	config.bSlam = true;    
	//SDK初始化    
	pSDK->Init(config);     


	//PrintModuleInfo(pSDK);
	//PrintModuleParameters(pSDK);
	//SDK图像数据回调	
	//pSDK->RegistModuleCameraCallback(SdkCameraCallBack,NULL);   
	//SDK模组IMU数据回调	
	//pSDK->RegistModuleIMUCallback(sdkImuCallBack,NULL);   
	////SDK的slam结算结果回调	
	pSDK->RegistModulePoseCallback(sdkSLAMResult,NULL);    
	//将DepthImageCallback设置为深度图回调
    //pSDK->AddPluginCallback("depthimage", "depth", DepthImageCallback, NULL);
	
    std::this_thread::sleep_for(std::chrono::seconds(60 *60 *24));
    pSDK->Release();
    delete pSDK;

    return 0;

}
