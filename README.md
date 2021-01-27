# Pandar128_SDK

## About the project
Pandar128_SDK project is the software development kit for:
**Pandar128/Pandar64S/Pandar40S/Pandar80S**
LiDAR sensor manufactured by Hesai Technology.
## Environment and Dependencies
**System environment requirement: Linux + G++ 7.0 or above**
**Library Dependencies: libpcap-dev + libyaml-cpp-dev**
```
$ sudo apt install libpcap-dev libyaml-cpp-dev
```
## Clone
```
$ git clone https://github.com/HesaiTechnology/Pandar128_SDK.git
```
## Build
```
1.$ cd Pandar128_SDK
2.$ mkdir build
3.$ cd build
4.$ cmake ..
5.$ make
```
## Run

Set the ip and port of lidar in class Pandar128SDK in test.cc
```
Pandar128SDK(std::string("192.168.1.201"), \    \\The ip of the device
            2368, \                             \\The port number of lidar data
            10110, \                            \\The port number of gps data
            std::string("Pandar128"), \
            std::string("../params/correction.csv"), \
            std::string(""), \
            std::string(""), \                  \\The pcap flie path
            lidarCallback, rawcallback, gpsCallback, \
            std::string(""), \
            std::string(""), \
            std::string(""), \
            0, 0, std::string("both_point_raw")));

```
Set the pcap flie path only when you what to read a pcap
```
$ make 
$ ./pandar128sdkTest
```
## Add to your project
### Cmake
```
add_subdirectory(<path_to>pandar128sdk)

include_directories(
	<path_to>Pandar128_SDK/include
	<path_to>Pandar128_SDK/src
)

target_link_libraries(<Your project>
  pandar128sdk
)
```
