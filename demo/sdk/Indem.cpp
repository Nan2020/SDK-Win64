#include <iostream>
#include "imrsdk.h"
#include <thread>
#ifdef WIN32
#include <Windows.h>
#endif
#include <mutex>
#include <future>
// 写ppm图像到文件
void ppm_save(char* filename, unsigned char* data, int w, int h)
{
    FILE* fp;
    char header[20];

    fp = fopen(filename, "wb");

    // 写图片格式、宽高、最大像素值
    fprintf(fp, "P6\n%d %d\n255\n", w, h);

    // 写RGB数据
    fwrite(data, w*h * 3, 1, fp);

    fclose(fp);
}

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
	//std::cout << "D: " << ds2 << std::endl;
	//EXPECT_TRUE(ds2 < 0.005);
}

void  SdkCameraCallBack(double time, unsigned char* pLeft, unsigned char* pRight, int width, int height, int channel, void* pParam)
{
	std::cout << "IMG: " << time<< std::endl;
}

void sdkImuCallBack(double time, float accX, float accY, float accZ, float gyrX, float gyrY, float gyrZ, void* pParam)
{
		//IMU temp;
		//temp.imu_time = time;
		//temp.acc[0] = accX;
		//temp.acc[1] = accY;
		//temp.acc[2] = accZ;
		//temp.gyp[0] = gyrX;
		//temp.gyp[1] = gyrY;
		//temp.gyp[2] = gyrZ;
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

	UCHAR* pImage = new UCHAR[Depth->_image_h*Depth->_image_w];
	for (int mv = 0; mv < Depth->_image_h; mv++)
	{
		for (int mu = 0; mu < Depth->_image_w; mu++)
		{
			//	cout << elas.D1[mv*img_W + mu] << endl;
			pImage[mv*Depth->_image_w + mu] = (UCHAR)fmax(Depth->_deepptr[mv*Depth->_image_w + mu] * 255.f / maxp, 0.f);
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
    config.bSlam = true;
    //初始化
    pSDK->Init(config);
    //SDK回调图像数据
	pSDK->RegistModuleCameraCallback(SdkCameraCallBack,NULL);
   //SDK回调IMU数据
	pSDK->RegistModuleIMUCallback(sdkImuCallBack,NULL);
   //SDK回调slam结算结果
	pSDK->RegistModulePoseCallback(sdkSLAMResult,NULL);
    //深度解算，DepthImageCallback为深度解算回调函数
    pSDK->AddPluginCallback("depthimage", "depth", DepthImageCallback, NULL);
    std::this_thread::sleep_for(std::chrono::seconds(60 ));
    pSDK->Release();
    delete pSDK;
}
