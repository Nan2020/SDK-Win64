#include <mutex>
#include <queue>
#include <thread>
#include <iomanip>
#include <string>
#include <iostream>
#include <cmath>
#include "imrsdk.h"

//ͨ��Cmake��DISPLAY_POINT_CLOUDѡ����������ʾ���ƹ���,����opencv_vizģ����ʾ
#ifdef DISPLAY_POINT_CLOUD
#include <opencv2/viz.hpp>
#include <opencv2/highgui.hpp>
#endif

using namespace indem;

struct ImrDepthImageTarget
{
    double _time;
	float _cubesize;
	int _image_w;
	int _image_h;
	float* _deepptr;
};

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

void DepthImageCallback(int ret, void* pData, void* pParam) {
	ImrDepthImageTarget* Depth = reinterpret_cast<ImrDepthImageTarget*>(pData);
        //std::cout << "DepthImageCallback==" << std::setprecision(10) << Depth->_image_w << ", _image_h" << Depth->_image_h << std::endl;
#ifdef DISPLAY_POINT_CLOUD
    static float maxp = 0;
    if (maxp == 0) {
        for (int mi = 0; mi < Depth->_image_w * Depth->_image_h; mi++)
        {
            if (Depth->_deepptr[mi] > maxp)
                maxp = Depth->_deepptr[mi];
        }
    }

    cv::Mat img = cv::Mat(Depth->_image_h, Depth->_image_w, CV_8UC1, Depth->_deepptr).clone();
    for (int mv = 0; mv < Depth->_image_h; mv++)
    {
        for (int mu = 0; mu < Depth->_image_w; mu++)
        {
            img.at<uchar>(mv, mu) = (uchar)fmax(Depth->_deepptr[mv*Depth->_image_w + mu] * 255.f / maxp, 0.f);
        }
    }

    cv::namedWindow("DepthDisplay");
    cv::imshow("DepthDisplay", img);
    cv::waitKey(1);
#endif
}

#ifdef DISPLAY_POINT_CLOUD
struct PointCloudData {
    int _image_w;
    int _image_h;
    float* _xyz;
};

std::mutex global_mutex;
cv::viz::WCloud* global_cloud_data=NULL;
void PointCloudCallback(int ret, void* pData, void* pParam) {
    PointCloudData* pPoints = (PointCloudData*)pData;
    static cv::Mat cloudPoint(pPoints->_image_h, pPoints->_image_w, CV_32FC3);
    for (int row = 0; row < pPoints->_image_h; ++row)
    {
        for (int col = 0; col < pPoints->_image_w; ++col)
        {
            size_t idx = row * pPoints->_image_w + col;
            auto* pt = cloudPoint.ptr<cv::Point3f>(row, col);
            pt->x = pPoints->_xyz[3 * idx + 0];
            pt->y = pPoints->_xyz[3 * idx + 1];
            pt->z = pPoints->_xyz[3 * idx + 2];
        }
    }
    //cv::imwrite("point.png", cloudPoint);
    if (global_cloud_data == nullptr) {
        global_cloud_data = new cv::viz::WCloud(cloudPoint, cv::viz::Color::green());
    }
    else {
        cv::viz::WCloud* pT = global_cloud_data;
        {
            std::unique_lock<std::mutex> cloudLock(global_mutex);
            global_cloud_data = nullptr;
        }
        global_cloud_data = new cv::viz::WCloud(cloudPoint, cv::viz::Color::green());
        delete pT;
        //cv::Affine3d aff;
        //global_cloud_data->updatePose(aff);
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

struct CommandParams {
    int16_t width;
    int16_t height;
    char distortion_model[16];
    double P[12];
};

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
    CommandParams params={0};
    //pSDK->InvokePluginMethod("depthimage","getParams",NULL,&params);
    pSDK->AddPluginCallback("depthimage", "depth", DepthImageCallback, NULL);


#ifdef DISPLAY_POINT_CLOUD
    //��ȡ�������ݲ���ά���֣�����������ģ��Ϊԭ�㣬���Ϊ�˿������棬��Ҫ����3D������camera��λ��
    pSDK->AddPluginCallback("depthimage", "point_cloud", PointCloudCallback, NULL);
    cv::viz::Viz3d window("point_cloud");
    window.showWidget("Coordinate", cv::viz::WCoordinateSystem());
    cv::Vec3f cam_position(0.0f,0.0f, -8.0f), cam_focal_point(0.f, 0.f, 0.0f), cam_head_direc(0.0f, 1.0f, 0.0f);
    cv::Affine3f cam_pose = cv::viz::makeCameraPose(cam_position, cam_focal_point, cam_head_direc);
    window.setViewerPose(cam_pose);
    window.setRenderingProperty("Coordinate", cv::viz::LIGHTING, 0);
    while (!window.wasStopped()) {
        if (global_cloud_data) {
            window.showWidget("Cloud", *global_cloud_data);
            try {
                window.setRenderingProperty("Cloud", cv::viz::LIGHTING, 0);
            }
            catch (cv::Exception& err) {
                std::cout << err.what() << std::endl;
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