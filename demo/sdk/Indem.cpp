#include <mutex>
#include <queue>
#include <thread>
#include <iomanip>
#include <string>
#include <iostream>
#include <cmath>
#include "imrsdk.h"

//通过Cmake打开DISPLAY_POINT_CLOUD选项以启用显示点云功能,依赖opencv_viz模块显示
#ifdef DISPLAY_POINT_CLOUD
#include <opencv2/viz.hpp>
#include <opencv2/highgui.hpp>
#endif

using namespace indem;

void PrintModuleInfo(CIMRSDK* pSDK)
{
    ImrModuleDeviceInfo info = pSDK->GetModuleInfo();
    std::cout << "Module Detail Info: \n"
        << "  ID: " << info._id << std::endl
        << "  Designed By: " << info._designer << std::endl
        << "  BaseLine: " << info._baseline << std::endl
        << "  Firmware Version: " << info._firmware_version << std::endl
        << "  Hardware Version: " << info._hardware_version << std::endl
        << "  IMU: " << info._imu << std::endl
        << "  Lens: " << info._lens << std::endl
        << "  View Angle: " << info._viewing_angle << std::endl;
}

void PrintEach(int row, int col, double* ptr)
{
    for (int r = 0; r < row; ++r)
    {
        for (int c=0;c<col;++c)
        {
            std::cout << ptr[r*col+c]<<"\t";
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
#ifdef DISPLAY_POINT_CLOUD
struct point_xyz {
    float x;
    float y;
    float z;
    float a;
};
struct DepthData {
    double _time;
    unsigned char* _depthImage;
    size_t _number;
    point_xyz* _points;
};
struct CommandParams {
    int16_t width;
    int16_t height;
    char distortion_model[16];
    double P[12];
};
CommandParams g_params = { 0 };
std::mutex global_mutex;
cv::viz::WCloud* global_cloud_data = NULL;
void CloudDataCallback(int ret, void* pData, void* pParam) {
    DepthData* pCloudData = (DepthData*)pData;
    cv::Mat cloudPoint(1, pCloudData->_number, CV_32FC3);
    cv::Mat cloudColor(1, pCloudData->_number, CV_8UC1);
    auto* pt = cloudPoint.ptr<cv::Point3f>();
    auto* pColor = cloudColor.ptr<unsigned char>();
    for (int cnt = 0; cnt < pCloudData->_number; ++cnt)
    {
        auto& point = pCloudData->_points[cnt];
        pt[cnt].x = point.x;
        pt[cnt].y = point.y;
        pt[cnt].z = point.z;
        pColor[cnt] = point.a;
    }
    if (global_cloud_data == nullptr) {
        global_cloud_data = new cv::viz::WCloud(cloudPoint, cloudColor);
    }
    else {
        cv::viz::WCloud* pT = global_cloud_data;
        {
            std::unique_lock<std::mutex> cloudLock(global_mutex);
            global_cloud_data = nullptr;
        }
        global_cloud_data = new cv::viz::WCloud(cloudPoint, cloudColor);
        delete pT;
    }
}
#endif

void  SdkCameraCallBack(double time, unsigned char* pLeft, unsigned char* pRight, int width, int height, int channel, void* pParam)
{
	//std::cout << "SdkCameraCallBack==" << std::setprecision(10) << time << std::endl;
}

void sdkImuCallBack(double time, float accX, float accY, float accZ, float gyrX, float gyrY, float gyrZ, void* pParam)
{
//	std::cout << "sdkImuCallBack==" << time << " " << accX << "  " << accY << "  " << accZ  << "  " << gyrX << "  " << gyrY << "  " << gyrZ  << std::endl;
}

void sdkSLAMResult(int ret, void* pData, void* pParam)
{
    ImrModulePose* pose = (ImrModulePose*)pData;
    std::cout << "SLAM: "<<pose->_pose._position[0] << " " << pose->_pose._position[1]  << " " << pose->_pose._position[2] << " "<< pose->_pose._oula[0] << " " << pose->_pose._oula[1]  << " " << pose->_pose._oula[2]  << std::endl;
}

int main()
{
//    using namespace indem;
    CIMRSDK* pSDK = new CIMRSDK();
    MRCONFIG config = { 0 };

    config.bSlam = true;   //true: open slam
    pSDK->Init(config);

    PrintModuleInfo(pSDK);
    PrintModuleParameters(pSDK);

    pSDK->RegistModuleCameraCallback(SdkCameraCallBack,NULL);
    pSDK->RegistModuleIMUCallback(sdkImuCallBack,NULL);
    pSDK->RegistModulePoseCallback(sdkSLAMResult,NULL);
 


    
#ifdef DISPLAY_POINT_CLOUD
	pSDK->InvokePluginMethod("pointcloud", "getParams", NULL, &g_params);
    if (pSDK->AddPluginCallback("pointcloud", "depth", CloudDataCallback, NULL) == PLG_NOT_EXIST) {
        std::cout << "Load pointcloud fail" << std::endl;
    }
    //获取点云数据并三维呈现，点云坐标以模组为原点，因此为了看到画面，需要设置3D场景中camera的位姿
    cv::viz::Viz3d window("point_cloud");
    window.showWidget("Coordinate", cv::viz::WCoordinateSystem());
    cv::Vec3f cam_position(-1.0f, 0.0f, .0f), cam_focal_point(0.f, 0.f, 0.0f), cam_y_direc(.0f, 0.0f, 1.0f);
    cv::Affine3f cam_pose = cv::viz::makeCameraPose(cam_position, cam_focal_point, cam_y_direc);
    window.setViewerPose(cam_pose);
    window.setRenderingProperty("Coordinate", cv::viz::LIGHTING, 0);
    while (!window.wasStopped()) {
        {
            std::unique_lock<std::mutex> cloudLock(global_mutex);
            if (global_cloud_data) {
                window.showWidget("Cloud", *global_cloud_data);
                try {
                    window.setRenderingProperty("Cloud", cv::viz::LIGHTING, 0);
                }
                catch (cv::Exception& err) {
                    std::cout << err.what() << std::endl;
                }
            }
        }
        window.spinOnce(1, true);
    }
#endif
    std::this_thread::sleep_for(std::chrono::seconds(60 * 60 * 24));
    pSDK->Release();
    delete pSDK;
    std::cout << "-----------------------END--------------------" << std::endl;
     
    return 0;
}