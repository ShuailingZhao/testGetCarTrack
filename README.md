## Get X,Y DeSai

#### testGetCarTrack 车道
从工程getDisWithGeometry修改来的，不是什么基础的工具，是一个临时快速的演示程序，宿迁交警项目，所以程序没有写的很好，实现的主要功能是已知摄像头的经纬度，利用视觉测量的方法，测出车相对于摄像头的相对位置，再加上绝对的摄像头的经纬度，来得到车的绝对经纬度，进而得到车的运行轨迹，画在高精度图上
input:高精地图，车道的检测，车辆的检测，摄像头的经纬度位置，摄像头的安装高度
output:车的经纬度位置
```
./clean.sh
cmake .. && make -j4
./testGetCarTrackXY -config=../data/defaultConfigNan.yml
```
