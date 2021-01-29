# HesaiLidar_Swift_SDK

## About the project
HesaiLidar_Swift_SDK project is the software development kit for:
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
$ git clone https://github.com/HesaiTechnology/HesaiLidar_Swift_SDK.git
```

## Build
```
1.$ cd HesaiLidar_Swift_SDK
2.$ mkdir build
3.$ cd build
4.$ cmake ..
5.$ make
```

## Add to your project
### Cmake
```
add_subdirectory(<path_to>HesaiLidar_Swift_SDK)

include_directories(
	<path_to>HesaiLidar_Swift_SDK/include
	<path_to>HesaiLidar_Swift_SDK/src
)

target_link_libraries(<Your project>
  PandarSwiftSDK
)
```

## Run

Set the parameters of class Pandar128SDK in test.cc
```
// for Pandar128
Pandar128SDK(std::string("192.168.1.201"), 2368, 10110, std::string("Pandar128"), \
                                std::string("../params/correction.csv"), \
                                std::string(""), \
                                std::string(""), \
                                lidarCallback, rawcallback, gpsCallback, \
                                std::string(""), \
                                std::string(""), \
                                std::string(""), \
                                0, 0, std::string("both_point_raw")));

```
Parameter description
```
        deviceipaddr  	  The ip of the device
        lidarport 	      The port number of lidar data
        gpsport           The port number of gps data
        frameid           The id of the point cloud data published to ROS
        correctionfile    The correction file path
        firtimeflie       The firtime flie path
        pcapfile          The pcap flie path
        pclcallback       The callback of PCL data structure
        rawcallback       The callback of raw data structure
        gpscallback       The callback of GPS structure
        certFile          The path of the user's certificate
        privateKeyFile    The path of the user's private key
        caFile            The path of the root certificate
        start_angle       The start angle of every point cloud should be <real angle> * 100.
        timezone          The timezone of local
        publishmode       The mode of publish
        datatype          The model of input data

```
Set the pcap flie path only when you what to read a pcap
```
$ make 
$ ./PandarSwiftTest
```
