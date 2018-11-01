#include <iostream>
#include <gtest/gtest.h>
#include "imrsdk.h"
#include <opencv2/opencv.hpp>
#include <thread>
#include "imr_driver.h"
#ifdef WIN32
#include <Windows.h>
#endif
#include <g3log/g3log.hpp>
#include <g3log/logworker.hpp>
#include <queue>
#include <mutex>
#include <fstream>
#include <future>
#include <boost/shared_array.hpp>

int g_iImageWidth = 0, g_iImageHeight = 0,g_r_num=0,g_o_num=0,g_num=0;

double imgTime = 0.0;
std::atomic<bool> g_stop(false);
std::atomic<bool> g_save(false);   //是否存储数据到文件

struct IMG
{
	cv::Mat image;
	double img_time = 0.0;
};
struct IMU
{
	double imu_time = 0.0;
	double current_imgTime = 0.0;
	float acc[3];
	float gyp[3];
};
struct ImrDepthImageTarget
{
	float _cubesize;
	int _image_w;
	int _image_h;
	cv::Mat _depth_image;
	float* _deepptr;
};
std::queue<IMG> Img_Queue;
std::queue<IMG> Img_Queue_w;
std::queue<IMU> Imu_Queue;
std::queue<IMU> Imu_Queue_w;
//std::queue<ImrModulePose> Slam_Queue;
std::mutex im;
std::mutex imu;
std::mutex ti;
std::mutex slamtx;
std::queue<void*> Slam_Queue; 
std::queue<void*> Slam_Queue_w;
//std::queue<void*> Slam_Queue;
//std::queue<void*> Slam_Queue_w; 
void HotPlugCallback(int iType, void* pParam) {
	if (iType) {
		std::cout << "Device Connect Success!";
	}
	else {
		std::cout << "Device Connect Fail!";
	}
}

void SeeThrougthCallback(int ret, void* pData, void* pParam) {
	ImrImage* pImage = reinterpret_cast<ImrImage*>(pData);
	//cv::Mat img(pImage->_height, pImage->_width, CV_8UC1, pImage->_image);
	//imwrite("D:/indemTest.jpg", img);
}
void DepthImageCallback(int ret, void* pData, void* pParam) {
	//std::cout << "---------iiiiiiiiiiiiiii-----------" << std::endl;
	ImrDepthImageTarget* Depth = reinterpret_cast<ImrDepthImageTarget*>(pData);
	if (g_save)
	{
#ifdef WIN32
		//cv::imwrite("D:/TEST/TEst.png", Depth->_depth_image);
#else
		//static int picnumber = 0;
		//++picnumber;
		//std::string name = "imgs/" + std::to_string(picnumber)+ ".png";
		//cv::imwrite(name, Depth->_depth_image);
#endif // WIN32
	}

	cv::imshow("depthimage.png", Depth->_depth_image);
	cv::waitKey(1);
}
void GetCameraCallback(double, ImrImage pData, void*) {
	//EXPECT_TRUE(pData._height == g_iImageHeight);
	//EXPECT_TRUE(pData._width == g_iImageWidth);
	EXPECT_TRUE(pData._image != NULL);
	//cv::Mat img(pData._height, pData._width, CV_8UC1, pData._image);
	//imwrite("D:/indemTest.jpg", img);
}

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

void PlaneCallback(ImrRecogPlaneTarget* res, void* pParam) {
	if (res->_planecount) {
		for (int idx = 0; idx < res->_planecount; ++idx) {
			//EXPECT_TRUE(res->_targets[idx]._pointNum > 0);
		}
	}
}

void SenmiticCallback(int ret, void* pData, void* pParam) {
	ImrRecognizeResult* res = (ImrRecognizeResult*)pData;
	if (res->_num > 0) {
		std::cout << "SenmiticMap: " << res->_num << std::endl;
	}
}

struct ImrSize {
	int32_t _width;
	int32_t _height;
};

void GetIMUCallback(double time, float accX, float accY, float accZ, float gyrX, float gyrY, float gyrZ, void* pParam)
{
	if (g_save)
	{
		++g_r_num;
		std::lock_guard<std::mutex> lck(imu);
		IMU temp;
		temp.imu_time = time;
		temp.acc[0] = accX;
		temp.acc[1] = accY;
		temp.acc[2] = accZ;
		temp.gyp[0] = gyrX;
		temp.gyp[1] = gyrY;
		temp.gyp[2] = gyrZ;

		Imu_Queue.push(temp);
	}
	//	printf("%-18.2f   %-18f   %-18f   %-18f   %-18f   %-18f   %-18f\r\n", time, accX, accY, accZ, gyrX, gyrY, gyrZ);
}

void GetModuleCameraCallback(double time, unsigned char* pLeft, unsigned char* pRight, int width, int height, int channel, void* pParam)
{
	//std::string filename = "imgs/" + std::to_string(time) + ".jpg";
	//cv::Mat img(height, width, CV_8UC1, pLeft);
	//imwrite(filename, img);
	if (g_save)
	{
		boost::shared_array<unsigned char> _data;
		_data = boost::shared_array<unsigned char>(new unsigned char[800 * 2560]);
		memcpy(_data.get(), pLeft, height * width * sizeof(unsigned char));
		cv::Mat img(height, width, CV_8UC1, _data.get());
		IMG temp;
		cv::Mat aa;
		temp.image = img.clone();
		temp.img_time = time;
		{
			std::lock_guard<std::mutex> lkk(im);
			Img_Queue.push(temp);
		}
		img.release();
	}
}

indem::CIMRSDK* CreateImrSDK(bool bOpenSlam, bool bDataSet = false)
{
	indem::CIMRSDK* pSDK = new indem::CIMRSDK();
	indem::MRCONFIG config = { 0 };
		//Init SDK
		config.bSlam = bOpenSlam;
		bool flag = pSDK->Init(config);
		if (flag) {
			std::cout << "Init SDK success!" << std::endl;
			return pSDK;
		}
		else
		{
			std::cout << "Init SDK failed!" << std::endl;
			pSDK->Release();
		}
		return NULL;
	}

void IMUDataCallbackFunction(imrIMUData* data) {
	if (g_save)
	{
		++g_r_num;
		IMU temp;
		temp.imu_time = data->_timeStamp;
		temp.acc[0] = data->_acc[0];
		temp.acc[1] = data->_acc[1];
		temp.acc[2] = data->_acc[2];
		temp.gyp[0] = data->_gyr[0];
		temp.gyp[1] = data->_gyr[1];
		temp.gyp[2] = data->_gyr[2];
		std::lock_guard<std::mutex> lck(imu);
		Imu_Queue.push(temp);
	}
}

std::string savefilename = "imu.csv";
std::ofstream outfile(savefilename, std::ios::app);
double Lastimu_time = 0.0;
void SaveImu()
{
	if (!g_save){
		outfile.close();
		return;
	}
	if (!outfile){
		return;
	}

	outfile << "time ;    gyp    ;   acc    " << std::endl;
	while (!g_stop)
	{
		if (Imu_Queue_w.empty()) {
			std::lock_guard<std::mutex> lck(imu);
			if (Imu_Queue.size() >= 2000) {
				Imu_Queue.swap(Imu_Queue_w);
				std::cout << "num=" << ++g_num << ",Imu_Queue_w.size=" << Imu_Queue_w.size() << ",Imu_Queue.size=" << Imu_Queue.size() << ",g_r_num=" << g_r_num << ",g_o_num=" << g_o_num << std::endl;
			}
		}
		else
		{
			IMU temp;
			temp = Imu_Queue_w.front();
			Imu_Queue_w.pop();
			{
				std::lock_guard<std::mutex> l(ti);
				temp.current_imgTime = imgTime;
			}

			//if (Lastimu_time < temp.imu_time)
			{
				//Lastimu_time = temp.imu_time;
				outfile << std::setiosflags(std::ios::fixed) << std::setprecision(14) << std::setiosflags(std::ios::showpoint)
					<< temp.imu_time << " ,"
					<< temp.gyp[0] << " ," << temp.gyp[1] << " ," << temp.gyp[2] << " ,"
					<< temp.acc[0] << " ," << temp.acc[1] << " ," << temp.acc[2] << " ,"
					<< temp.current_imgTime << " ,"

					<< std::endl;
			}
			++g_o_num;
		}
	}
	outfile.close();
}

std::string SDKSaveSlam = "Slam.csv";
std::ofstream outfileSlam(SDKSaveSlam, std::ios::app);
void SaveSlam()
{
	if (!g_save) {
		outfileSlam.close();
		return;
	}
	if (!outfileSlam){
		return;
	}

	while (!g_stop)
	{
		if (Slam_Queue_w.empty()) {
			std::lock_guard<std::mutex> lck(slamtx);
			if (Slam_Queue.size() >= 2000) {
				Slam_Queue.swap(Slam_Queue_w);
			}
		}
		else
		{
			ImrModulePose* temp = NULL;
			temp = (ImrModulePose*)Slam_Queue_w.front();
			Slam_Queue_w.pop();

			if (temp != NULL){
				outfileSlam << std::setiosflags(std::ios::fixed) << std::setprecision(14) << std::setiosflags(std::ios::showpoint)
					<< temp->_score << " ,"
					<< temp->_pose.time << " , "
					<< temp->_pose._position[0] << " ," << temp->_pose._position[1] << " ," << temp->_pose._position[2] << " ,"
					<< temp->_pose._rotation[0] << " ," << temp->_pose._rotation[1] << " ," << temp->_pose._rotation[2] << " ," << temp->_pose._rotation[3] << " ,"
					<< std::endl;
			}
		}
	}
	outfileSlam.close();
}

void CameraCallbackFunction(imrCameraData* data)
{
	if (g_save)
	{
		if (data == nullptr)
		{
			assert(data != nullptr);
			std::cout << "img_data is NULL " << std::endl;
		}
		else
		{
			/*uchar *image_uchar = data->_image;
			cv::Mat img = cv::Mat(800, 2560, CV_8UC3, data->_image);*/
			boost::shared_array<unsigned char> _data;
			_data = boost::shared_array<unsigned char>(new unsigned char[800 * 2560]);
			memcpy(_data.get(), data->_image, 800 * 2560 * sizeof(unsigned char));
			cv::Mat img(800, 2560, CV_8UC1, _data.get());
			IMG temp;
			cv::Mat aa;
			temp.image = img.clone();
			temp.img_time = data->_timeStamp;
			{
				std::lock_guard<std::mutex> l(ti);
				imgTime = temp.img_time;
			}
			{
				std::lock_guard<std::mutex> lck(im);
				Img_Queue.push(temp);
			}
			img.release();
		}
	}
}

std::string savefil = "img.csv";
std::ofstream outfiless(savefil, std::ios::app);
void SaveImg()
{
	if (!g_save) {
		outfiless.close();
		return;
	}
	if (!outfiless){
		return;
	}
    outfiless << "time ;  pic    " << std::endl;
	while (!g_stop)
	{
		if (Img_Queue_w.empty()) {
			std::lock_guard<std::mutex> lck(im);
			if (Img_Queue.size() >= 1000) {
				Img_Queue.swap(Img_Queue_w);
			}
		}
		else
		{
			IMG temp;
			temp = Img_Queue_w.front();
			Img_Queue_w.pop();

			std::stringstream ss;
			ss << std::dec << std::setiosflags(std::ios::fixed) << std::setprecision(14) << std::setiosflags(std::ios::showpoint)
				<< temp.img_time << ".png";
			outfiless.setf(std::ios::fixed, std::ios::floatfield);
			outfiless << temp.img_time << "," << temp.img_time << ".png" << std::endl;
		}
	}
	outfiless.close();
}


std::string saveslam = "slam.csv";
std::ofstream outfileslam(saveslam, std::ios::app);
//void SaveSlam()
//{
//
//	if (!outfileslam)
//	{
//		return;
//	}
//	outfileslam << "time ;    q    ;   p    " << std::endl;
//	while (!g_stop)
//	{
//		//int Size = 0;
//		if (Slam_Queue.size() > 0)
//		{
//			ImrModulePose* temp;
//			{
//				std::lock_guard<std::mutex> lck(slamtx);
//				temp = (ImrModulePose*)Slam_Queue.front();
//				Slam_Queue.pop();
//			}
//			{
//				outfileslam << std::setiosflags(std::ios::fixed) << std::setprecision(14) << std::setiosflags(std::ios::showpoint)
//					<< temp->_pose.time << " ,"
//					<< temp->_pose._rotation[0] << " ," << temp->_pose._rotation[1] << " ," << temp->_pose._rotation[2] << " ," << temp->_pose._rotation[3] << " , "
//					<< temp->_pose._position[0] << " ," << temp->_pose._position[1] << " ," << temp->_pose._position[2] << " ,"
//					<< std::endl;
//			}
//		}
//	}
//}


void  SdkCameraCallBack(double time, unsigned char* pLeft, unsigned char* pRight, int width, int height, int channel, void* pParam)
{
	//std::cout << "-------------IMG" << std::endl;
	if (g_save)
	{
		if (pLeft == nullptr)
		{
			assert(pLeft != nullptr);
			std::cout << "img_data is NULL " << std::endl;
		}
		else
		{
			/*uchar *image_uchar = data->_image;
			cv::Mat img = cv::Mat(800, 2560, CV_8UC3, data->_image);*/
			boost::shared_array<unsigned char> _data_L;
			//boost::shared_array<unsigned char> _data_R;
			_data_L = boost::shared_array<unsigned char>(new unsigned char[800 * 1280]);
			//_data_R = boost::shared_array<unsigned char>(new unsigned char[800 * 1280]);
			memcpy(_data_L.get(), pLeft, 800 * 1280 * sizeof(unsigned char));
			//memcpy(_data_R.get(), pRight, 800 * 2560 * sizeof(unsigned char));
			cv::Mat img(800, 1280, CV_8UC1, _data_L.get());
			IMG temp;
			cv::Mat aa;
			temp.image = img.clone();
			temp.img_time = time;
			{
				std::lock_guard<std::mutex> l(ti);
				imgTime = temp.img_time;
			}
			std::lock_guard<std::mutex> lck(im);
			Img_Queue.push(temp);
			img.release();
			//cv::Mat  R;
			//cv::resize(temp.image, R, cv::Size(temp.image.cols / 2, temp.image.rows /2), 0, 0, cv::INTER_LINEAR);

			//cv::imshow("left", R);

			//cv::waitKey(1);
			//cv::imshow("pathimage.png", temp.image);
			//cv::waitKey(1);
		}
	}
}
double last_imutime = 0.0;
void sdkImuCallBack(double time, float accX, float accY, float accZ, float gyrX, float gyrY, float gyrZ, void* pParam)
{
	//std::cout << "==================IMU" << std::endl;
	if (g_save)
	{
		IMU temp;
		temp.imu_time = time;
		temp.acc[0] = accX;
		temp.acc[1] = accY;
		temp.acc[2] = accZ;
		temp.gyp[0] = gyrX;
		temp.gyp[1] = gyrY;
		temp.gyp[2] = gyrZ;
		std::lock_guard<std::mutex> lck(imu);
		Imu_Queue.push(temp);
	}
	else
	{
		LOG(INFO) << std::setprecision(10) << time;
	}
}

void sdkSLAMResult(int, void* pData, void* pParam)
{
	//std::cout << "==================SLAm" << std::endl;
	if (g_save)
	{
		std::lock_guard<std::mutex> lck(slamtx);
		Slam_Queue.push(pData);
	}
}

#if 0
/*Test: IMU data callback*/
TEST(Indem, ModuleIMUTest) {
	indem::CIMRSDK* pSDK = CreateImrSDK(true);
	if (pSDK != NULL) {
		g_stop = false;
		std::thread savefie_thd(std::bind(SaveImu));
		pSDK->RegistModuleIMUCallback(GetIMUCallback, NULL);

		std::this_thread::sleep_for(std::chrono::seconds(12*60*60));

		std::cout << "-----------------------END1--------------------" << std::endl;
		pSDK->RegistModuleIMUCallback(NULL, NULL);
		pSDK->Release();
		std::cout << "-----------------------END2--------------------" << std::endl;

		g_stop = true;

		std::cout << "g_r_num=" << g_r_num << ",g_o_num=" << g_o_num << std::endl;
	}
	std::cout << "-------------------ModuleIMUTest END----------------" << std::endl;
}
#endif

#if 0
/*Test:  camera data callback*/
TEST(Indem, ModuleCameraTest) {
    indem::CIMRSDK* pSDK = CreateImrSDK(true);
    std::cout << "-----------------------CreateImrSDK--------------------" << std::endl;
    if (pSDK != NULL) {
        std::thread threadSaveImage(std::bind(SaveImg));
        pSDK->RegistModuleCameraCallback(GetModuleCameraCallback, NULL);

        std::this_thread::sleep_for(std::chrono::seconds(60));

        std::cout << "-----------------------END1--------------------" << std::endl;
        //pSDK->RegistModuleCameraCallback(NULL, NULL);
        pSDK->Release();
        std::cout << "-----------------------END2--------------------" << std::endl;
        delete pSDK;

        g_stop = true;
        threadSaveImage.join();
    }
}
#endif

#if 0
/*Test: DepthImage*/
TEST(Indem, ModuleDepthImageTest) {
	indem::CIMRSDK* pSDK = CreateImrSDK(true);
	std::cout << "-----------------------CreateImrSDK--------------------" << std::endl;
	if (pSDK != NULL) {
		pSDK->AddPluginCallback("depthimage", "depth", DepthImageCallback, NULL);

		std::this_thread::sleep_for(std::chrono::seconds(60));

		std::cout << "-----------------------END1--------------------" << std::endl;
		pSDK->AddPluginCallback("depthimage", "depth", NULL, NULL);
		pSDK->Release();
		std::cout << "-----------------------END2--------------------" << std::endl;
	}
	std::cout << "-------------------ModuleCameraTest END----------------" << std::endl;
}
#endif

#if 0
TEST(Indem, driver) {//存储图像数据的Main函数
	std::cout << "driver" << std::endl;
	g_stop = false;
	IMRDEVICE_HANDLE handlfd = imrOpenIMR();
	DevicesInfo m_idiInfo = imrSearchDevices(handlfd);
	std::cout << "Find " << m_idiInfo.devNum << " Devices " << std::endl;

	CameraConfig cc;
	cc._cb = CameraCallbackFunction;
	std::thread savefie_thd(std::bind(SaveImu));
	//std::thread saveimg_thd(std::bind(SaveImg));
	//	imrConnectDevice(handlfd, m_idiInfo._devInfo[0]._id, 0, IMUDataCallbackFunction);
	imrInitCamera(handlfd, cc);

	imrStartCamera(handlfd);
	imrConnectDevice(handlfd, NULL, 0, IMUDataCallbackFunction);

#ifdef WIN32
	Sleep(10 * 60);
#else
	sleep(3 * 60);
#endif

	std::cout << "g_r_num=" << g_r_num << ",g_o_num=" << g_o_num << std::endl;
	g_stop = true;

	imrCloseIMR(NULL);
	std::cout << "TestDriverFinish" << std::endl;
}
#endif

#if 1
TEST(Indem, Initailize) {
    using namespace g3;
    std::unique_ptr<LogWorker> m_pLogger = LogWorker::createLogWorker();
    m_pLogger->addDefaultLogger("SDK", ".");
    initializeLogging(m_pLogger.get());

    g_stop = false;
    using namespace indem;
    //RegistDisconnectCallback(HotPlugCallback, NULL);
    CIMRSDK* pSDK = new CIMRSDK();
    MRCONFIG config = { 0 };
    //strcpy(config.slamPath, "E:/Program Files/indemind/Software Develop Kit/slam.dll");

    //pSDK->RegistModulePoseCallback(PoseCallback, NULL);
    config.bSlam = true;
    EXPECT_TRUE(pSDK->Init(config));
	g_save = true;
	std::thread savefie_thd(std::bind(SaveImu));
	std::thread saveimg_thd(std::bind(SaveImg));
	std::thread saveslam_thd(std::bind(SaveSlam));
	pSDK->RegistModuleCameraCallback(SdkCameraCallBack,NULL);
	pSDK->RegistModuleIMUCallback(sdkImuCallBack,NULL);
	pSDK->RegistModulePoseCallback(sdkSLAMResult,NULL);

	//pSDK->AddPluginCallback("depthimage", "depth", DepthImageCallback, this);

    //int plgNum = 0;
    //char** names=new char*[8];
    //for (int i=0;i<8;++i)
    //{
    //    names[i] = new char[32];
    //}
    //pSDK->ListPluginsInfo(&plgNum, names);
    //for (int i = 0; i < 8; ++i)
    //{
    //    delete[] names[i];
    //}
    //delete[] names;
    //注册摄像头画面回调函数
    //pSDK->AddPluginCallback("seethrough", "image", SeeThrougthCallback, NULL);
    //std::cout << "Start Motionless Test" << std::endl;
    //std::this_thread::sleep_for(std::chrono::seconds(15));
    //关闭摄像头画面回调函数
	//pSDK->AddPluginCallback("depthimage", "depth", NULL, NULL);
    //Sleep(5 * 1000);
    std::this_thread::sleep_for(std::chrono::seconds(60 ));
    //Sleep(5 * 1000);
    std::cout << "-----------------------END1--------------------" << std::endl;
	outfiless.close();
	outfile.close();
	outfileslam.close();
    pSDK->Release();
    std::cout << "-----------------------END2--------------------" << std::endl;
    delete pSDK;

    g_stop = true;
    saveimg_thd.join();
    savefie_thd.join();
	saveslam_thd.join();
}
#endif


//深度解算   不跑Slam
#if 0
TEST(Testidnem,Initialize){
	g_stop = false;
	using namespace indem;

	CIMRSDK* pSDK = new CIMRSDK();
	MRCONFIG m_config = { 0 };

	m_config.imgFreq = 60;
	m_config.bSlam = true;
	EXPECT_TRUE(pSDK->Init(m_config));
	pSDK->RegistModuleCameraCallback(SdkCameraCallBack, NULL);
	pSDK->RegistModuleIMUCallback(sdkImuCallBack, NULL);
	pSDK->RegistModulePoseCallback(sdkSLAMResult, NULL);
	//pSDK->AddPluginCallback("depthimage", "depth", DepthImageCallback, this);
	std::this_thread::sleep_for(std::chrono::seconds(60 * 60));
	std::cout << "END!" << std::endl;

	pSDK->Release();
	delete pSDK;
	g_stop = true;

    //saveimg_thd.join();
    //savefie_thd.join();
}
#endif 