# indemind-sdk-win64
Binocular Vision Inertial Module

#### 简介  

INDEMIND双目视觉惯性模组采用全局快门的2X1280X800@50FPS高清摄像头，可提供水平120°、垂向75°视场角，结合高帧率6轴IMU传感器，为SLAM等算法提供强有力前端数据采集能力。
本SDK提供了双目视觉惯性模组的开发接口及依赖环境。  
已在以下环境下测试过：  
Windows 10 64位, VS2015, VS2017

#### 硬件要求  
双目视觉惯性模组要求支持USB3.0接口。深度解算以插件形式存在，该插件依赖CUDA9.0，建议使用Geforce GTX 1050以上的显卡

#### 使用方式  
创建SDK对象  
~~~
    CIMRSDK* pSDK = new CIMRSDK();  
~~~
设置使用的SLAM  
~~~
    MRCONFIG config = { 0 };
    strcpy(config.slamPath, "slam.dll");
    config.bSlam = true;
~~~
获取模组图像数据
~~~
    pSDK->RegistModuleCameraCallback(SdkCameraCallBack,NULL);
~~~
获取IMU数据
~~~
    pSDK->RegistModuleIMUCallback(sdkImuCallBack,NULL);
~~~
获取SLAM结果
~~~
    pSDK->RegistModulePoseCallback(sdkSLAMResult,NULL);
~~~
获取深度图
~~~
    pSDK->AddPluginCallback("depthimage", "depth", DepthImageCallback, NULL);
~~~
释放资源
~~~
    pSDK->Release();
    delete pSDK;
~~~

详细信息参考《双目惯性模组用户使用说明》。