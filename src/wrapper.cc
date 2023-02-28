#include "wrapper.h"
#include "pandarSwiftSDK.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>
#ifdef __cplusplus
extern "C" {
#endif

#define PRINT_FLAG
int printFlag = 1; // 1:print, other number: don't print
int pcdFileWriteFlag = 0; // 1:write pcd, other number: don't write pcd
bool running = true;
int frameItem = 0;
int saveFrameIndex = 10;
boost::shared_ptr<PandarSwiftSDK> spPandarSwiftSDK;

void gpsCallback(double timestamp) {
    if(printFlag == 1)     
        printf("gps: %lf\n", timestamp);   
}

void lidarCallback(boost::shared_ptr<PPointCloud> cld, double timestamp) {
    if(printFlag == 1)       
        printf("timestamp: %lf,point_size: %ld\n", timestamp, cld->points.size());
    if(pcdFileWriteFlag == 1) {
        frameItem++;
        if(saveFrameIndex == frameItem) {
            int Num = cld->points.size();                           
            std::ofstream zos("./cloudpoints.csv");
            for (int i = 0; i < Num; i++)                             
            {
                zos <<  cld->points[i].x << "," << cld->points[i].y << "," << cld->points[i].z << "," << cld->points[i].intensity << "," << cld->points[i].timestamp << "," << cld->points[i].ring << std::endl;
            }
        }
    }         
}



void rawcallback(PandarPacketsArray *array) {
    // printf("array size: %d\n", array->size());
}

void RunPandarSwiftSDK(char* deviceipaddr, int lidarport, int gpsport, char* correctionfile, char* firtimeflie, char* pcapfile,
						char* certFile, char* privateKeyFile, char* caFile, int runTime) {
    spPandarSwiftSDK.reset(new PandarSwiftSDK(deviceipaddr, "", lidarport, gpsport, std::string("Pandar128"), \
                                correctionfile, \
                                firtimeflie, \
                                pcapfile, lidarCallback, rawcallback, gpsCallback, \
                                certFile, \
                                privateKeyFile, \
                                caFile, \
                                0, 0, std::string("both_point_raw"), false, ""));  
  
    sleep(runTime);
    spPandarSwiftSDK->stop();
    return;
}

void SetPcdFileWriteFlag(int flag, int frameNum){
    saveFrameIndex = frameNum;
    pcdFileWriteFlag = flag;
    return;
}

void SetPrintFlag(int flag){
    printFlag = flag;
    return;
}
#ifdef __cplusplus
};
#endif