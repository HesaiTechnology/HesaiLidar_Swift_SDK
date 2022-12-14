# HesaiLidar_Swift_SDK
## About the project
This repository includes the software development kit for Pandar LiDAR sensor manufactured by Hesai Technology. Branches are included for different systems and UDP protocol versions:
* master: The software development kit for Ubuntu 20.04 and Ubuntu 18.04 supports Hesai lidar with UDP protocol v1.3 and v1.4
* ubuntu16.04: The software development kit for Ubuntu 16.04 supports Hesai lidar with UDP protocol v1.3 and v1.4 
* UDP4.3: The software development kit for Ubuntu 20.04, Ubuntu 18.04 and Ubuntu 16.04 supports Hesai lidar with UDP protocol v4.1 and v4.3

To get the UDP protocol version number of your device,  check the UDP package header field.

## Environment and Dependencies

**System environment requirement:Linux**
```
Recommanded
-Ubuntu 16.04
-Ubuntu 18.04
-Ubuntu 20.04
```

**Compiler vresion requirement**
```
Cmake version requirement:Cmake 3.8.0 or above
G++ version requirement:G++ 7.5 or above
```
**Library Dependencies: libpcl-dev + libpcap-dev + libyaml-cpp-dev + libboost-dev**
```
$ sudo apt install libpcl-dev libpcap-dev libyaml-cpp-dev libboost-dev
```

## Clone
```
$ git clone https://github.com/HesaiTechnology/HesaiLidar_Swift_SDK.git
$ git checkout UDP4.3
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

## C++

Set the parameters of class PandarSwiftSDK in test.cc
```
// for PandarAT128
PandarSwiftSDK(std::string("192.168.1.201"), 2368, 10110, std::string("PandarAT128"), \
                                std::string("../params/correction.csv"), \
                                std::string(""), \
                                std::string(""), \
                                lidarCallback, rawcallback, gpsCallback, faultMessageCallback\
                                std::string(""), \
                                std::string(""), \
                                std::string(""), \
                                0, 0, 1, std::string("both_point_raw"), "", threadPriority));

```
Parameter description
```
        deviceIpaddr  	     The ip of the device
        hostIpaddr  	       The ip of the host ip address
        lidarPort 	         The port number of lidar data
        gpsPort              The port number of gps data
        frameId              The id of the point cloud data published to ROS
        correctionFile       The correction file path
        firtimeFlie          The firtime flie path
        pcapFile             The pcap flie path
        pclCallback          The callback of PCL data structure
        rawCallback          The callback of raw data structure
        gpsCallback          The callback of GPS structure
        faultMessageCallback The callback 0f fault message structure
        certFile             The path of the user's certificate
        privateKeyFile       The path of the user's private key
        caFile               The path of the root certificate
        startAngle           The start angle of every point cloud should be <real angle> * 100.
        timeZone             The timezone of local
        viewMode             The different way to parser and  show the point cloud
        publishMode          The mode of publish
        threadPriority       The priority of thread 
        dataType             The model of input data
        multCastIp           The multicast IP address of connected Lidar, will be used to get udp packets from multicast ip address

```
Set the pcap flie path only when you what to read a pcap
```
$ make 
$ ./PandarSwiftTest
```
