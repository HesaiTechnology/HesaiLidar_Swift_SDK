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

#define PRINT_FLAG (false)
#define PCD_FILE_WRITE_FLAG (false) //false: don't save point cloud data;
                                    //true : save a frame of point cloud data
int frameItem = 0;

void gpsCallback(double timestamp) {
    printf("gps: %lf\n", timestamp);
}

void lidarCallback(boost::shared_ptr<PPointCloud> cld, double timestamp) {
    if(PRINT_FLAG)
        printf("timestamp: %lf,point_size: %ld\n", timestamp, cld->points.size());
    //Debug code,save the tenth frame data to the local pcd file to verify the correctness of the data
    if(PCD_FILE_WRITE_FLAG) {
        frameItem++;
        if(10 == frameItem) {
            printf("write pcd file\n");
            pcl::PCDWriter writer;
            writer.write("PandarAT128Pcd.pcd", *cld);
        }
    }
}

void rawcallback(PandarPacketsArray *array) {
    // printf("array size: %d\n", array->size());
}

int main(int argc, char** argv) {
    boost::shared_ptr<PandarSwiftSDK> spPandarSwiftSDK;
    spPandarSwiftSDK.reset(new PandarSwiftSDK(std::string("192.168.1.201"), 2368, 10110, std::string("PandarAT128"), \
                                std::string("../params/default_PandarAT.dat"), \
                                std::string(""), \
                                std::string(""), lidarCallback, rawcallback, gpsCallback, \
                                std::string(""), \
                                std::string(""), \
                                std::string(""), \
                                0, 0, 1, \
                                std::string("both_point_raw")));
    while (true) {
        sleep(100);
    }
    return 0;
}