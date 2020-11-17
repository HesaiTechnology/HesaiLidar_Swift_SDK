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
#include "pandar128SDK.h"

void gpsCallback(double timestamp) {
    printf("gps: %lf\n", timestamp);
}

void lidarCallback(boost::shared_ptr<PPointCloud> cld, double timestamp) {
    printf("timestamp: %lf,point_size: %ld\n", timestamp, cld->points.size());
    
    // test code, save one frame for data analysis
    // static int item = 0;
    // item++;
    // if(10 == item) {
    //     printf("write pcd file\n");
    //     pcl::PCDWriter writer;
    //     writer.write("../P128Pcd.pcd", *cld);
    // }
}

void rawcallback(PandarPacketsArray *array) {
    // printf("array size: %d\n", array->size());
}

int main(int argc, char** argv) {
    boost::shared_ptr<Pandar128SDK> spPandar128SDK;
    spPandar128SDK.reset(new Pandar128SDK(std::string("192.168.1.201"), 2368, 10110, std::string("Pandar128"), \
                                std::string("../params/correction.csv"), \
                                std::string("../params/laser-ts.csv"), \
                                std::string(""), lidarCallback, rawcallback, gpsCallback, 0, 0, std::string("both_point_raw")));
    while (true) {
        sleep(100);
    }
    return 0;
}