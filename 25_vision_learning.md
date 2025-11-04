# 25赛季视觉代码学习

## 1 自瞄需要实现的任务：
- 目标装甲板、Buff符位姿的解算以及运动状态的预测
- 接管发射机构控制权，判断合适开火时机
![自瞄模块划分](https://github.com/user-attachments/assets/0e3e960e-0bf0-430a-95f5-d0803d1a6ade)
（打符会相应少一个分类模块）
### DeepWiki:https://deepwiki.com/shuiqinhh/sp_vision_25/1-overview
## 2 wsl前备知识
- 打开Ubuntu中的VSCode
``` 
 code .
 ``` 
- [x] 安装支持USB接口的msi程序
  
## 3 代码文件结构
```
sp_vision_25 
├── assets         // 包含demo素材、网络权重等
│   └── ...
├── calibration    // 标定相关程序
│   ├── calibrate_camera.cpp             // 相机内参标定程序
│   ├── calibrate_handeye.cpp            // 手眼标定程序
│   ├── calibrate_robotworld_handeye.cpp // 手眼标定程序（同时计算标定板位置）
│   └── capture.cpp                      // 相机标定数据采集程序
├── CMakeLists.txt // CMake配置文件
├── configs        // 每台机器人的YAML配置文件
│   └── ...
├── io             // 硬件抽象层，见3.4软件架构
│   └── ...
├── src            // 应用层，见3.4软件架构
│   └── ...
├── tasks          // 功能层，见3.4软件架构
│   ├── auto_aim       // 自瞄相关算法实现
│   │   └── ...
│   ├── auto_buff      // 打符相关算法实现
│   │   └── ...
│   └── omniperception // 全向感知相关算法实现
│   │   └── ...
├── tests
│   ├── auto_aim_test.cpp         // 自瞄录制视频测试程序
│   ├── auto_buff_test.cpp        // 打符录制视频测试程序
│   ├── camera_detect_test.cpp    // 识别器测试程序（工业相机）
│   ├── camera_test.cpp           // 相机测试程序
│   ├── camera_thread_test.cpp    // 相机线程测试程序
│   ├── cboard_test.cpp           // C板测试程序
│   ├── detector_video_test.cpp   // 识别器测试程序（视频）
│   ├── dm_test.cpp               // 达妙IMU测试程序
│   ├── fire_test.cpp             // 开火测试程序
│   ├── gimbal_response_test.cpp  // 云台响应测试程序
│   ├── gimbal_test.cpp           // 云台通信测试程序
│   ├── handeye_test.cpp          // 手眼标定测试程序
│   ├── minimum_vision_system.cpp // 最小视觉系统测试程序
│   ├── multi_usbcamera_test.cpp  // 多USB摄像头测试程序
│   ├── planner_test_offline.cpp  // 规划器测试程序（离线）
│   ├── planner_test.cpp          // 规划器测试程序（实车）
│   ├── publish_test.cpp          // ROS发送测试程序
│   ├── subscribe_test.cpp        // ROS接收测试程序
│   ├── topic_loop_test.cpp       // ROS话题循环测试程序
│   ├── usbcamera_detect_test.cpp // 识别器测试程序（USB相机）
│   ├── usbcamera_test.cpp        // USB相机测试程序
│   └── ...
└── tools          // 工具层，见3.4软件架构
    ├── crc.hpp                    // CRC校验
    ├── exiter.hpp                 // 退出检测
    ├── extended_kalman_filter.hpp // 扩展卡尔曼滤波器
    ├── img_tools.hpp              // 图像处理工具
    ├── logger.hpp                 // 日志记录器
    ├── math_tools.hpp             // 数学工具
    ├── plotter.hpp                // 曲线图绘制工具
    ├── recorder.hpp               // 视频录制器
    ├── thread_safe_queue.hpp      // 线程安全队列
    ├── trajectory.hpp             // 弹道解算
    ├── yaml.hpp                   // YAML配置文件解析器
    └── ...
```    

## 4 代码学习ing
### .vscode :  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;针对Linux系统x64架构的VScode中一些编译环境的配置。
```c
#if __INTELLISENSE__       //Intellisense:VScode的智能编译系统（C/C++扩展定义的宏）
#undef __ARM_NEON          //如果是Intellisense分析，取消定义arm_neon宏(默认当前linux系统为x64架构)
#undef __ARM_NEON__
#endif
```
### assets:
<p>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;内含测试demo以及一些训练好的神经网络模型参数文件(yolov5\yolov8\yolo11 and 打符&自瞄分离)。

### calibration:
#####  calibrate_camara.cpp: 相机内参标定程序，通过拍摄标定板照片（物理世界中的各参数确定），再通过逻辑计算得到相机内参。

  ```c
  <cv::Point3f> //表三维空间中一个点的数据类型，内含float x,float y,float z
  <cv::Point> //二维空间点，数据类型int
  <cv::Ponit2f>//二维空间点，数据类型float
  <cv:；Point3d>//三维空间点，数据类型double
  ```
  ```c
  std::vector<cv::Point3f> centers_3d(const cv::Size & pattern_size, const float center_distance)              //两个参数分别为图像尺寸和格子（像素）中心距
  {
    std::vector<cv::Point3f> centers_3d;
    for (int i = 0; i < pattern_size.height; i++)
       for (int j = 0; j < pattern_size.width; j++)
            centers_3d.push_back({j * center_distance, i * center_distance, 0});
    return centers_3d;
  }       //遍历得到所有像素坐标，形成点云
  ```
  
  ```c
  void load(
  const std::string & input_folder, const std::string & config_path, cv::Size & img_size,
  std::vector<std::vector<cv::Point3f>> & obj_points,
  std::vector<std::vector<cv::Point2f>> & img_points)        //obj_points:实际点坐标阵（mm），img_points:图像坐标下像素点阵
  {
    // 读取yaml参数
    auto yaml = YAML::LoadFile(config_path);
    auto pattern_cols = yaml["pattern_cols"].as<int>();
    auto pattern_rows = yaml["pattern_rows"].as<int>();
    auto center_distance_mm = yaml["center_distance_mm"].as<double>();
    cv::Size pattern_size(pattern_cols, pattern_rows);
    for (int i = 1; true; i++) {
      // 读取图片
      auto img_path = fmt::format("{}/{}.jpg", input_folder, i);
      auto img = cv::imread(img_path);
      if (img.empty()) break;

    // 设置图片尺寸
      img_size = img.size();

    // 识别标定板
      std::vector<cv::Point2f> centers_2d;
      auto success = cv::findCirclesGrid(img, pattern_size, centers_2d, cv::CALIB_CB_SYMMETRIC_GRID);  //检测到完整的点网格返回true

    // 显示识别结果
      auto drawing = img.clone();
      cv::drawChessboardCorners(drawing, pattern_size, centers_2d, success);
      cv::resize(drawing, drawing, {}, 0.5, 0.5);  // 缩小图片尺寸便于显示完全
      cv::imshow("Press any to continue", drawing);
      cv::waitKey(0);

    // 输出识别结果
      fmt::print("[{}] {}\n", success ? "success" : "failure", img_path);
      if (!success) continue;

    // 记录所需的数据
      img_points.emplace_back(centers_2d);
      obj_points.emplace_back(centers_3d(pattern_size, center_distance_mm));  //emplace_back,在vector动态数组末尾加元素（比如这里在obj_points中添加新行，每行是一张图片像素点的的3d坐标）
    }
  }
  ```
  此程序导出参数：
  1.相机内参矩阵：
  $$
  K = \begin{bmatrix}
  f_x & 0 & c_x \\
  0 & f_y & c_y \\
  0 & 0 & 1
  \end{bmatrix}
  $$
  其中fx,fy为相机焦距，cx,cy为相机主点坐标（一般为中心点），最后一个1用于坐标系变换。
  3D坐标到2D坐标的变换公式：
  $$s
\begin{bmatrix}
u \\
v \\
1
\end{bmatrix}=K\cdot[R|t]\cdot
\begin{bmatrix}
X \\
Y \\
Z \\
1
\end{bmatrix}$$
其中u,v为二维坐标，s为缩放因子，[R|t]为外参矩阵（定义了世界坐标系到相机坐标系的转换），满足公式：
$$\begin{bmatrix}
X_{c} \\
Y_{c} \\
Z_{c}
\end{bmatrix}=\mathbf{R}
\begin{bmatrix}
X_{w} \\
Y_{w} \\
Z_{w}
\end{bmatrix}+\mathbf{t}$$
其中R为拍摄图片的3×3旋转向量（相机朝向），t为3×1的平移向量（相机位置）

2.畸变系数：包含径向畸变（直线向外或向内弯曲）和切向畸变（由于镜头和成像平面不平行产生偏）。

3.重投影误差：衡量标定的精度
$$\mathrm{error}=\frac1N\sum_{i=1}^N\|p_i-\hat{p}_i\|$$


$\cdot$ $p_i\to$实测点$\cdot$ $\hat{p} _i\rightarrow$投影点 $\cdot$ $N\to$所有点的总数

###### calibrate_handeye.cpp：手眼标定程序，求解“相机坐标系”→“云台坐标系”的旋转矩阵和平移矩阵。
- 前备知识1：旋转矩阵
  - 旋转矩阵相乘：如果一个旋转矩阵$R_{1}$表示坐标系A向坐标系B的转换，则A坐标系中的3×1的向量$v_{A}$,其在B坐标系中则表示为$R_{1}v_{A}$。
  另一个旋转矩阵$R_{2}$表示坐标系B向坐标系C的转换，则坐标系A向坐标系C的转换公式为：
  $$
  R_{AC}=R_{2}R_{1}
  $$
  - 旋转矩阵转置：旋转矩阵是正交矩阵，其逆矩阵与转置矩阵相等：$R^{-1}=R^{T}$
    向量$v_{B}$表示向量$v_{A}$在B坐标系中的表示，则$v_{A}$=$R_{1}^{T}v_{B}$

- 前备知识2：四元数
  四元数是一种能够表示三维空间里旋转的量，可以看作是一种扩展了复数的数：
  $$
  q=w+xi+yj+zk
  $$
  其中i,j,k是三个方向上的虚数单位($i^{2}=j^{2}=k^{2}=ijk=-1$)，也可以理解为本身就是一种旋转，i表示yOz平面内从z轴向y轴的旋转，k表示xOy平面内y轴向x轴的旋转，j表示xOz平面内x轴向z轴的旋转。四元数可看成旋转向量+旋转角度的组合：
  $$
  q=\cos(\theta/2)+\sin(\theta/2)(u_xi+u_yj+u_zk)
  $$
  其中$u_{x},u_{y},u_{z}$表示旋转轴的方向单位向量，$\theta$表示旋转的角度。
  关于四元数转换成3×3旋转矩阵的公式：
  $$
  \mathcal{M}(\hat{\mathbf{v}},\theta)=
\begin{bmatrix}
\cos\theta+(1-\cos\theta)u_{x}^2 & (1-\cos\theta)u_{x}u_{y}-(\sin\theta)u_{z} & (1-\cos\theta)u_{x}u_{z}+(\sin\theta)u_{y} \\
(1-\cos\theta)u_{y}u_{x}+(\sin\theta)zu_{z} & \cos\theta+(1-\cos\theta)u_{y}^2 & (1-\cos\theta)u_{y}u_{z}-(\sin\theta)u_{x} \\
(1-\cos\theta)u_{z}u_{x}-(\sin\theta)u_{y} & (1-\cos\theta)u_{z}u_{y}+(\sin\theta)u_{x} & \cos\theta+(1-\cos\theta)u_{z}^2
\end{bmatrix}
 $$
其中$\hat{\mathbf{v}}$为旋转中心单位向量。
- 前备知识3：欧拉角
  欧拉角是一种用三个角度描述空间旋转的方式，比如最常见的ZYX顺序欧拉角：
  $$R=R_z(\psi)R_y(\theta)R_x(\phi)$$
   - $\psi(yaw):$绕Z轴旋转的偏航角
   - $\theta(pitch):$绕Y轴旋转的俯仰角
   - $\phi(roll):$绕X轴旋转的滚转角
  欧拉角中的万向锁问题：当中间的角（比如在此中的pitch）旋转到正负$\pi/2$时会失去一个自由度（此时绕Z轴旋转失效）。
  
  
```c
Eigen::Quaterniond read_q(const std::string & q_path)
{
  std::ifstream q_file(q_path);
  double w, x, y, z;
  q_file >> w >> x >> y >> z;
  return {w, x, y, z};
} //读取文件中的四元数
```
```c
void load(
  const std::string & input_folder, const std::string & config_path,
  std::vector<double> & R_gimbal2imubody_data, std::vector<cv::Mat> & R_gimbal2world_list,
  std::vector<cv::Mat> & t_gimbal2world_list, std::vector<cv::Mat> & rvecs,
  std::vector<cv::Mat> & tvecs) //参数说明：云台到IMU机体的旋转矩阵（固定）；云台到世界坐标系的旋转矩阵；云台到世界坐标系的平移向量；标定板在相机坐标系下的旋转向量和平移向量
{
  // 读取yaml参数
  auto yaml = YAML::LoadFile(config_path);
  auto pattern_cols = yaml["pattern_cols"].as<int>();
  auto pattern_rows = yaml["pattern_rows"].as<int>();
  auto center_distance_mm = yaml["center_distance_mm"].as<double>();
  R_gimbal2imubody_data = yaml["R_gimbal2imubody"].as<std::vector<double>>();
  auto camera_matrix_data = yaml["camera_matrix"].as<std::vector<double>>();
  auto distort_coeffs_data = yaml["distort_coeffs"].as<std::vector<double>>();

  cv::Size pattern_size(pattern_cols, pattern_rows);
  Eigen::Matrix<double, 3, 3, Eigen::RowMajor> R_gimbal2imubody(R_gimbal2imubody_data.data());
  cv::Matx33d camera_matrix(camera_matrix_data.data());
  cv::Mat distort_coeffs(distort_coeffs_data);

  for (int i = 1; true; i++) {
    // 读取图片和对应四元数
    auto img_path = fmt::format("{}/{}.jpg", input_folder, i);
    auto q_path = fmt::format("{}/{}.txt", input_folder, i);
    auto img = cv::imread(img_path);
    Eigen::Quaterniond q = read_q(q_path);
    if (img.empty()) break;

    // 计算云台的欧拉角
    Eigen::Matrix3d R_imubody2imuabs = q.toRotationMatrix();  //四元数转化为imu机体坐标系向imu绝对坐标系的转化的旋转矩阵
    Eigen::Matrix3d R_gimbal2world =
      R_gimbal2imubody.transpose() * R_imubody2imuabs * R_gimbal2imubody; //注意gimbal到imubody的转换等于world到imuabs的转换
    Eigen::Vector3d ypr = tools::eulers(R_gimbal2world, 2, 1, 0) * 57.3;  // degree

    // 在图片上显示云台的欧拉角，用来检验R_gimbal2imubody是否正确
    auto drawing = img.clone();
    tools::draw_text(drawing, fmt::format("yaw   {:.2f}", ypr[0]), {40, 40}, {0, 0, 255});
    tools::draw_text(drawing, fmt::format("pitch {:.2f}", ypr[1]), {40, 80}, {0, 0, 255});
    tools::draw_text(drawing, fmt::format("roll  {:.2f}", ypr[2]), {40, 120}, {0, 0, 255});

    // 识别标定板
    std::vector<cv::Point2f> centers_2d;
    auto success = cv::findCirclesGrid(img, pattern_size, centers_2d);  // 默认是对称圆点图案

    // 显示识别结果
    cv::drawChessboardCorners(drawing, pattern_size, centers_2d, success);
    cv::resize(drawing, drawing, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
    cv::imshow("Press any to continue", drawing);
    cv::waitKey(0);

    // 输出识别结果
    fmt::print("[{}] {}\n", success ? "success" : "failure", img_path);
    if (!success) continue;

    // 计算所需的数据
    cv::Mat t_gimbal2world = (cv::Mat_<double>(3, 1) << 0, 0, 0);
    cv::Mat R_gimbal2world_cv;
    cv::eigen2cv(R_gimbal2world, R_gimbal2world_cv);
    cv::Mat rvec, tvec;
    auto centers_3d_ = centers_3d(pattern_size, center_distance_mm);
    cv::solvePnP(
      centers_3d_, centers_2d, camera_matrix, distort_coeffs, rvec, tvec, false, cv::SOLVEPNP_IPPE);

    // 记录所需的数据
    R_gimbal2world_list.emplace_back(R_gimbal2world_cv);
    t_gimbal2world_list.emplace_back(t_gimbal2world);
    rvecs.emplace_back(rvec);
    tvecs.emplace_back(tvec);
  }
}
```





  

  

