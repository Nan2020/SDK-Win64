#pragma once
//该文件存放SDK对外的数据接口
//坐标系定义：x轴右，y轴上,z轴后
struct ImrPose {
	//[x,y,z]
	float _position[3];
	//[w,x,y,z]
	float _rotation[4];
};
//头显位姿
struct ImrModulePose {
	ImrPose _pose;
	bool _isLoop;   //闭环标志
    int  _score;    //-1和0是有效数据,其它无效数据
};

//头显摄像头图像数据
struct ImrImage {
    double _time;
    int _width;
    int _height;
	unsigned char* _image;
};

//头显摄像头双目原始图像数据
struct ImrImages {
    double _time;
    int _width;
    int _height;
    unsigned char* _image[2];
};

//模组信息
struct ImrModuleDeviceInfo {
    char _id[32];               //模组ID
    char _designer[32];         //模组开发商
    char _fireware_version[32]; //固件版本
    char _hardware_version[32];
    char _lens[32];
    char _imu[32];
    char _viewing_angle[32];
    char _baseline[32];
};
