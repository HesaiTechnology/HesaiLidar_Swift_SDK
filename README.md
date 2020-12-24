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
## Build
```
1.$ cd Pandar128_SDK
2.$ mkdir build
3.$ cd build
4.$ cmake ..
5.$ make
```
## Run

Set the parameters of class Pandar128SDK in test.cc
```
        lidarport 		  The port number of lidar data
        gpsport   		  The port number of gps data
        frameid           The id of the point cloud data published to ROS
        correctionfile    The correction file path
        firtimeflie       The firtime flie path
        pcapfile          The pcap flie path
        pclcallback       The callback of PCL data structure
        rawcallback       The callback of raw data structure
        gpscallback       The callback of GPS structure
        certFile          Represents the path of the user's certificate
        privateKeyFile    Represents the path of the user's private key
        caFile            Represents the path of the root certificate
        start_angle       The start angle of every point cloud
                          should be <real angle> * 100.
        timezone          The timezone of local
        publishmode       The mode of publish
        datatype          The model of input data

```
Set the pcap flie path only when you what to read a pcap
```
$ make 
$ ./pandar128sdkTest
```


