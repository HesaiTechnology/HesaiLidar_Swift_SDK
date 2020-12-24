# Pandar128_SDK

## About the project
Pandar128_SDK project is the software development kit for:
**Pandar128**
LiDAR sensor manufactured by Hesai Technology.
## Environment and Dependencies
**System environment requirement: Linux + G++ 7.0 or above**
**Library Dependencies: libpcap-dev + libyaml-cpp-dev**
```
$ sudo apt install libpcap-dev libyaml-cpp-dev
```
## Clone
```
$ git clone https://github.com/HesaiTechnology/HesaiLidar_General_SDK.git
```
## Build
```
$ cd Pandar128_SDK
$ mkdir build
$ cd build
$ cmake ..
$ make
```
## Run
```
$ ./pandar128sdkTest

```