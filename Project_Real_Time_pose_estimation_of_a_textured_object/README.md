Reference:

> [Real Time pose estimation of a textured object](http://docs.opencv.org/3.1.0/dc/d2c/tutorial_real_time_pose.html) and here is [Code](https://github.com/opencv/opencv/tree/master/samples/cpp/tutorial_code/calib3d/real_time_pose_estimation)

Tips:

1. This Project is running in Debian with Qt and Opencv 3.
2. The code by Document cannot Run with QT, so I do something small change.

## 简单思路
想要使用此项目，首先需要明确一点，这里尝试定位的是一个盒子(BOX)，甚至在box的配置文件中(BOX.ply)中写入了mesh，所以暂时只是演示一个长方形，其他的空间体暂时没有经过测试。

整个QT工程，是首先通过一张图片进行标定（或者称之为注册），然后才能实现实时的定位。

### 1.注册（main_registration.cpp）

注册过程包含几个部分。

##### a. 手动输入八个顶点
首先，利用一张用来标定的初始图片，用户按照顺序，手动在图片点出8个点所在的位置。（这里使用长方形，所以指定了八个顶点作为关键点，其中这八个点三维的World Coordination的位置存在box.ply文件中，而摄像机坐标系的2D坐标就是手动给出的，其实通过这两份数据就可以利用SolvePnP计算出当前box在摄像头坐标系下的空间姿态）


然后，利用这两组数据进行estimatePose，甚至进行校准，如下图：

![Register_1_1.png](https://ws1.sinaimg.cn/mw690/bc1eaf88ly1ffe2lhbvwhj20v00jsqi5.jpg)

![register_1_2.jpg](https://ws1.sinaimg.cn/mw690/bc1eaf88ly1ffe2rylg24j20hs0dcwgr.jpg)

注意，其中红色的点是手动点击的，绿色的点是程序经过PnP估算重新校准的。还不清楚绿色点是否替换掉了红色的手动标记点。


##### b.ORB计算特征点

利用ORB特征选取整张图上的特征点，遍历所有特征点，如果在盒子的表面，这就是位于盒子上的关键点，存下来，准确来说是存储到cookies_ORB.yml文件中去，以便实时检测时使用。

![Register_2](https://ws1.sinaimg.cn/mw690/bc1eaf88ly1ffe30vhcvhj20hs0dctbe.jpg)

图中显示0个在盒子上，这种情况，待查。

### 2.实时检测（main_detection.cpp）
实时检测就是利用之前标定好的数据(cookies_ORB.yml)，从视频流中不停的计算盒子的空间姿态。

##### Step 1: Robust matching between model descriptors and scene descriptors
这里使用`cv::FlannBasedMatcher`的Robust matching，具体来说就是从视频流中取出图片，检测关键点和描述子，再与之前计算好的描述子进行比较，使用two Nearest Neighbou，并会移除非对称的描述子。 这样找到了当前视频图片中的特征点描述子，与之前注册过程计算好的描述子，两者之间的良好匹配。

##### Step 2: Find out the 2D/3D correspondences
在上一步找到的良好匹配中，保存当前视频帧的2D坐标，保存最好匹配的之前计算好的特征点的3D坐标，准备SolvePnP。


##### Step 3: Estimate the pose using RANSAC approach
利用上一步的摄像头坐标系下的2D坐标以及对应点的世界坐标系的3D坐标，计算SolvePnP

##### Step 4: Catch the inliers keypoints to draw
个人觉得这一步可以省略

##### Step 5: Kalman Filter
Kalman滤波，这个可以单独拿出来讲。在此不表，为了提高姿态的输出数据的稳定性。




