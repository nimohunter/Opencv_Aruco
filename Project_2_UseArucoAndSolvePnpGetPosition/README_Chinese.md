完成了第一次完整的流程。
1. 摄像头标定
2. 图像处理 （基本没处理，只是变成黑白）
3. 特征点抓取识别 （使用的Aruco）
4. 利用特征点执行POSIT和SolvePnP算法 （仅仅使用的SolvePnpRanse）
6. 数据平滑处理，并Unity显示出来

思路整理：

### 1. 摄像头标定
这里的代码放到Github上去了。 基本都是网上down下来的，没什么好说的。

### 2. 图像处理
只是变成黑白

### 3. 抓取识别
使用的是Aruco库，在Opencv_contrib库里。识别使用的代码很简单：

```
cv::Mat inputImage = imread("/home/nimo/Pictures/chessboard00.jpg");
cv::Ptr<aruco::DetectorParameters> parameters;
cv::Ptr<aruco::Dictionary> dictionary=aruco::getPredefinedDictionary(aruco::DICT_6X6_250);

std::vector<int> ids;
std::vector<std::vector<cv::Point2f> > corners, rejectedCandidates;
cv::aruco::detectMarkers(inputImage, dictionary, corners, ids);
```

这里的ids和corners就是检测到的结果。
ids 存着检测到的图片的ID，同时Corners里面存着Aruco图片的四个点在Image的位置。这里使用各个Aruco标记图的四个顶点作为特征点。

### 4. SolvePnP使用
既然Corners里面有特征点，然后手算了这些点的空间位置。
```
std::map<int ,std::vector<cv::Point3f> >  AurcoObjectPositionMap;
AurcoObjectPositionMap[31].push_back(cv::Point3f(2.7, 6.9, 6.2));
AurcoObjectPositionMap[31].push_back(cv::Point3f(7.9, 6.9, 6.2));
AurcoObjectPositionMap[31].push_back(cv::Point3f(7.9, 6.9, 1.0));
AurcoObjectPositionMap[31].push_back(cv::Point3f(2.7, 6.9, 1.0));
```
例如，这里做个map，将id和空间位置对应上。这样就可以直接算Solvepnp了，当然这里用的是
`solvePnPRansac`。

### 5. 数据平滑处理，并Unity显示出来

在算空间位置的时候输出到文件，将solvePnPRansac的postion和rotaion输出出来，用Unity读取文件。这里的数据不是很友好，抖动非常厉害，需要进行数据平滑处理。

这里数据平滑处理暂时处理比较简单，缓存三四组Postion，后来的Pos数据与前面差别太大将抛弃，视为错误数据。输出给Unity的数据是缓存数据的平均值。



    