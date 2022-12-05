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

// #define PRINT_FLAG 
// #define PCD_FILE_WRITE_FLAG  //false: don't save point cloud data; true : save a frame of point cloud data
// #define SET_LIDAR_STANDBY_MODE
// #define SET_LIDAAR_NORMAL_MODE 
// #define SET_LIDAR_RETURN_MODE
// #define GET_LIDAR_RETURN_MODE
// #define SET_LIDAR_SPIN_RATE
// #define GET_LIDAR_SPIN_RATE
// #define SET_LIDAR_LENS_HEAT_SWITCH
// #define GET_LIDAR_LENS_HEAT_SWITCH 
// #define SAVE_FAULT_MESSAGE
// #define SET_PTCS_MODE
// #define SET_PTC_MODE
// #define GET_PTCS_STATE // -1:get fail; 0:ptc mode; 1:ptcs mode
uint16_t lidarSpinRate = 200; // 200 300 400 500
uint8_t lidarReturnMode = 0;   // 0-last return, 1-strongest return, 2-dual return  
uint8_t lidarLensHeatSwitch = 0; //0-close, 1-open                            
int frameItem = 0;

void gpsCallback(double timestamp) {
#ifdef PRINT_FLAG     
    printf("gps: %lf\n", timestamp);
#endif    
}

void lidarCallback(boost::shared_ptr<PPointCloud> cld, double timestamp) {
#ifdef PRINT_FLAG 
    printf("timestamp: %lf,point_size: %ld\n", timestamp, cld->points.size());
    //Debug code,save the tenth frame data to the local pcd file to verify the correctness of the data
#endif
#ifdef PCD_FILE_WRITE_FLAG
    frameItem++;
    // uint32_t startTick = GetTickCount();
    pcl::PCDWriter writer;
    std::string name = "PandarAT128Pcd" + std::to_string(frameItem) + ".pcd";
    writer.write(name, *cld);
    // uint32_t endTick = GetTickCount();
    // printf("save pcd use time %ums\n", -startTick + endTick);
#endif
}

void rawcallback(PandarPacketsArray *array) {
    // printf("array size: %d\n", array->size());
}

void faultmessagecallback(AT128FaultMessageInfo &faultMessage) {
#ifdef SAVE_FAULT_MESSAGE    
    static int count = 0;
    if(count == 0){
        std::ofstream zos("./faultmessage.csv");
        zos <<  "Version " << "," << "Total_time" << "," << "Operate_State" << "," << "Fault_State" << "," << "Fault_code_type" << "," << "Rolling_Counter" \
        << "," << "Total_fault_code_number" << "," << "Fault_code_ID" << "," << "DTC_Num" << "," << "DTC_State" << "," << "TDM_Data_indicator" << "," << "Temperature" << "," << "软件ID" \
        << "," << "软件版本号" << "," << "硬件版本号" << "," << "BT版本号" << "," << "Heating_Sate"  << "," << "High_Temperture_Shutdown_State";
        for(int i = 0; i < 12; i++){
            for(int j = 0; j < 8; j++){
                zos << "," << "水平区域" << std::to_string(i) << " 垂直区域" << std::to_string(j);
            }
        }
        zos << std::endl;

    }
    count++;
    std::ofstream zos("./faultmessage.csv", std::ios::app);
    zos <<  int(faultMessage.m_u8Version) << "," << std::to_string(faultMessage.m_dTotalTime) << "," <<int (faultMessage.m_operateState) << "," << int(faultMessage.m_faultState) << "," << int(faultMessage.m_faultCodeType) << "," <<\
    int(faultMessage.m_u8RollingCounter) << "," << int(faultMessage.m_u8TotalFaultCodeNum) << "," << int(faultMessage.m_u8FaultCodeId) << "," << int(faultMessage.m_iDTCNum) << "," << int(faultMessage.m_DTCState) << "," << int(faultMessage.m_TDMDataIndicate) << "," <<\
    faultMessage.m_dTemperature << "," << faultMessage.m_u16SoftwareId << "," << faultMessage.m_u16SoftwareVersion << "," << faultMessage.m_u16HardwareVersion << "," << faultMessage.m_u16BTversion << "," << int(faultMessage.m_HeatingState) << "," << int(faultMessage.m_HighTempertureShutdownState);

    for(int i = 0; i < 12; i++){
        for(int j = 0; j < 8; j++){
            zos << "," << int(faultMessage.m_LensDirtyState[i][j]);
        }
    }
    zos << std::endl;
#endif    
}



int main(int argc, char** argv) {
    boost::shared_ptr<PandarSwiftSDK> spPandarSwiftSDK;
    std::map<std::string, int32_t> configMap;
    configMap["process_thread"] = 91;
    configMap["publish_thread"] = 90;
    configMap["read_thread"] = 99;
    configMap["timestamp_num"] = 0;
    configMap["without_pointcloud_udp_warning_time"] = 10000; // ms
    configMap["without_faultmessage_udp_warning_time"] = 10000; // ms
    configMap["untragger_pclcallback_warning_time"] = 10000; // ms
    spPandarSwiftSDK.reset(new PandarSwiftSDK(std::string("192.168.1.201"), std::string(""), 2368, 10110, std::string("PandarAT128"), \
                                std::string("../params/corrections1.5.dat"), \
                                std::string("../params/AT128E2X_Firetime Correction File.csv"), \
                                std::string(""), lidarCallback, rawcallback, gpsCallback, faultmessagecallback,\
                                std::string(""), \
                                std::string(""), \
                                std::string("../params/ca-chain.cert.pem"), \
                                0, 0, 1, \
                                std::string("both_point_raw"), "", configMap));
#ifdef SET_LIDAR_STANDBY_MODE 
    spPandarSwiftSDK->setStandbyLidarMode();  
#endif 

#ifdef SET_LIDAAR_NORMAL_MODE 
    spPandarSwiftSDK->setNormalLidarMode();  
#endif 

#ifdef SET_LIDAR_RETURN_MODE 
    spPandarSwiftSDK->setLidarReturnMode(lidarReturnMode); 
#endif 

#ifdef GET_LIDAR_RETURN_MODE 
    uint8_t returnMode = -1;
    spPandarSwiftSDK->getLidarReturnMode(returnMode); 
    printf("Lidar return mode is %d\n", returnMode);
#endif 

#ifdef SET_LIDAR_SPIN_RATE 
    spPandarSwiftSDK->setLidarSpinRate(lidarSpinRate);
#endif 

#ifdef GET_LIDAR_SPIN_RATE 
    uint16_t spinRate = 0;
    spPandarSwiftSDK->getLidarSpinRate(spinRate);
    printf("Lidar spin rate is %drpm\n",spinRate);
#endif 

#ifdef SET_LIDAR_LENS_HEAT_SWITCH 
    spPandarSwiftSDK->setLidarLensHeatSwitch(lidarLensHeatSwitch);
#endif 

#ifdef GET_LIDAR_LENS_HEAT_SWITCH 
    uint8_t lensHeatSwitch;
    spPandarSwiftSDK->getLidarLensHeatSwitch(lensHeatSwitch);
    printf("Lidar len heat switch status is %d\n", lensHeatSwitch);
#endif

#ifdef SET_PTCS_MODE 
    spPandarSwiftSDK->setPtcsLidarMode();
#endif 

#ifdef SET_PTC_MODE 
    spPandarSwiftSDK->setPtcLidarMode();
#endif 

#ifdef GET_PTCS_STATE 
    int result = spPandarSwiftSDK->getPtcsLidarMode();
    printf("Get ptcs state result = %d\n",result);
#endif 
    spPandarSwiftSDK->start();
    while (!spPandarSwiftSDK->GetIsReadPcapOver()) {
        usleep(100);
    }
    spPandarSwiftSDK->stop();
    return 0;
}