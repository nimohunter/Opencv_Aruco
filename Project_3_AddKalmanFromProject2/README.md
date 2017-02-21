Reference:

1. [Real Time pose estimation of a textured object](http://docs.opencv.org/3.1.0/dc/d2c/tutorial_real_time_pose.html) and here is [Code](https://github.com/opencv/opencv/tree/master/samples/cpp/tutorial_code/calib3d/real_time_pose_estimation)

2. [Opencv 3.0 API](http://docs.opencv.org/3.1.0/index.html)

确实从这篇Document学到了很多，简单重构了一下Project2的代码，形成行的Project3.

---

### Usage
输入的是录像文件，需要摄像头的标定数据，以及Aruco对应的点的位置。即可输出SolvePnp和Kalman之后的数据。

### Conclusion
虽然私底下重构了这个版本，但是关于Cmake使用都不太会，应该重新看看参考中的Document，有很大帮助。

### Reference中的基本思路
[Real Time pose estimation of a textured object]，这篇文档的简单思路是：

1. 对原图的Box的的主平面做好ORB的特征点检测，并利用摄像头的标定信息，生成特征点的3D数据并存储起来。[main_registration.cpp]
2. 从摄像机取Frame，同样做ORB的特征点检测，使用Flann和原图进行Match，这样找到了Frame特征点和原图的特征点的对应关系，从而对应上第一步生成的特征点的3D数据。
3. 利用Frame的特征点的2D数据，和对应的特征点的3D数据进行SolvePnPRanse得到Camera Pose数据
4. 利用Kalman滤波，将Camera Pose数据进行平滑。



