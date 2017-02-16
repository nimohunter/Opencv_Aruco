Reference:

1. [Real Time pose estimation of a textured object](http://docs.opencv.org/3.1.0/dc/d2c/tutorial_real_time_pose.html) and here is [Code](https://github.com/opencv/opencv/tree/master/samples/cpp/tutorial_code/calib3d/real_time_pose_estimation)

2. [Opencv 3.0 API](http://docs.opencv.org/3.1.0/index.html)

确实从这篇Document学到了很多，简单重构了一下Project2的代码，形成行的Project3.

---

### Usage
输入的是录像文件，需要摄像头的标定数据，以及Aruco对应的点的位置。即可输出SolvePnp和Kalman之后的数据。

### Conclusion
虽然私底下重构了这个版本，但是关于Cmake使用都不太会，应该重新看看参考中的Document，有很大帮助。

