# Lidar_RTK_Calib_SDK
[TOC]



### 功能 

使用`手眼标定`法计算Lidar和INS(RTK or IMU)的相对姿态



### 手眼标定原理

https://github.com/jhu-lcsr/handeye_calib_camodocal

![            AX=XB Hand Eye Calibration Solution](assets/687474703a2f2f692e737461636b2e696d6775722e636f6d2f77644f79672e6a7067.jpeg) 

![1551086092459](assets/1551086092459.png)

[论文](doc/Optimal Hand-Eye Calibration.pdf)

### 姿态表示

geometry_msgs::PoseWithCovarianceStamped

http://docs.ros.org/lunar/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html

![1548401502593](assets/1548401502593.png)

### 编译

catkin_ws/src目录下

```shell
git clone http://192.168.10.192:8800/quchunlei/lidar_rtk_calib.git
cd ../
catkin build
source devel/setup.zsh 
```

### 数据采集

- 消息格式

  | topic | msg                     |
  | ----- | ----------------------- |
  | 点云  | sensor_msgs/PointCloud2 |
  | rtk   | sensor_msgs/NavSatFix   |
  | imu   | sensor_msgs/Imu         |

  

- 在室外开阔场地，周围有建筑物，如下图

  ![image](assets/image.png)

- 录制数据

  车辆先静止2-5秒，再以5公里每小时的速度行驶2个`8`字形

  ```shell
  rosbag record -a 
  ```

  ![image](assets/image-1549942026684.png)

### 运行

#### 测试数据

smb: /xxx/cali_8zixing_rtk.bag

md5： 00fa5625d8b3d6a8077ce73f654228a5



```shell
roslaunch lidar_rtk_calibration run_calib.launch

```

#### 标定计算 

run_calib.launch

|                 |                   | 备注                    |
| --------------- | ----------------- | ----------------------- |
| calib_bag_file  |                   | 使用绝对路径            |
| imu_topic       |                   | rtk自带的imu            |
| lidar_topic     |                   |                         |
| rtk_topic       |                   |                         |
| lidar_pose_file | lidar每一帧的位姿 | 从rs_mapping 工具包得到 |
| save_file       | 输出数据          | 使用绝对路径            |

![1549952183206](assets/1549952183206.png)

计算过程需要几十秒，计算结束



![1549965820362](assets/1549965820362.png)

```shell
Translation (x,y,z) : 2.00624, 0.396034, 13.3124（手动改为0）
Rotation (w,x,y,z): 0.999903, -0.0115794, -0.000901282, -0.00771969

```

标定结果z值需要手动改为0或测量值

####  验证

```shell
roslaunch rs_lidar_rtk_calibration sim_rtk_mapping.launch
```

修改launch文件的topic和标定参数，理想的标定结果下，点云在固定坐标系`odom`下保持静止。错误的标定结果会导致车辆掉头后点云上下和左右移动。IMU测量值的抖动或导致点云绕基准摇晃，这不是标定错误导致的。



![1547799190214](assets/1547799190214.png)

![1547799211628](assets/1547799211628.png)

![1547799304479](assets/1547799304479.png)

![1547799344618](assets/1547799344618.png)

![1547799556816](assets/1547799556816.png)

![1547799571517](assets/1547799571517.png)

![1547799692648](assets/1547799692648.png)

![1547799722043](assets/1547799722043.png)

![1547799848441](assets/1547799848441.png)

![1547799916615](assets/1547799916615.png)