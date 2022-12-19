/******************************************************************************
 * Copyright 2020 The Hesai Technology Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "pandarSwiftSDK.h"
#define PRINT_FLAG
// #define PCD_FILE_WRITE_FLAG

int frameItem = 0;

void gpsCallback(double timestamp) {
#ifdef PRINT_FLAG    
    printf("gps: %lf\n", timestamp);
#endif    
}

void lidarCallback(boost::shared_ptr<PPointCloud> cld, double timestamp) {
#ifdef PRINT_FLAG       
    printf("timestamp: %lf,point_size: %ld\n", timestamp, cld->points.size());
#endif

#ifdef PCD_FILE_WRITE_FLAG
    frameItem++;
    pcl::PCDWriter writer;
    std::string fileName = "PointCloudFrame" + std::to_string(frameItem) + ".pcd";
    writer.write(fileName, *cld);
    printf("save frame %d\n",frameItem);
#endif      
}

void rawcallback(PandarPacketsArray *array) {
    // printf("array size: %d\n", array->size());
}

int main(int argc, char** argv) {
    boost::shared_ptr<PandarSwiftSDK> spPandarSwiftSDK;
    spPandarSwiftSDK.reset(new PandarSwiftSDK(std::string("192.168.1.201"), "", 2368, 10110, std::string("Pandar128"), \
                                std::string("../params/Pandar128_Correction.csv"), \
                                std::string("../params/Pandar128_Firetimes.csv"), \
                                std::string(""), lidarCallback, rawcallback, gpsCallback, \
                                std::string(""), \
                                std::string(""), \
                                std::string(""), \
                                0, 0, std::string("both_point_raw"), false, \
                                std::string("../params/QT128C2X_Channel_Cofig.csv"), ""));
    while (true) {
        // PandarFunctionSafety functionSafety;
        // spPandarSwiftSDK->getPandarFunctionSafety(functionSafety);
        // functionSafety.Print();
        sleep(1);
    }
    return 0;
}