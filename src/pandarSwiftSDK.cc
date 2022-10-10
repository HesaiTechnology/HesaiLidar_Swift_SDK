/*
 *  Copyright (c) 2020 Hesai Photonics Technology, Lingwen Fang
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file
 *   This class PandarSwiftSDKs raw Pandar128 3D LIDAR packets to PointCloud2.
 */
#include "pandarSwiftSDK.h"

#include <sys/syscall.h>
#include <time.h>
#include <unistd.h>

#include <fstream>
#include <iostream>

#include "platUtil.h"
#include "taskflow.hpp"

#define ENABLE_FIRETIME_CORRECTION
// #define PRINT_TEMPERATURE

static tf::Executor executor;
float degreeToRadian(float degree) { return degree * M_PI / 180.0f; }

static const float elevAngle[] = {
    14.436f,  13.535f,  13.08f,   12.624f,  12.163f,  11.702f,  11.237f,
    10.771f,  10.301f,  9.83f,    9.355f,   8.88f,    8.401f,   7.921f,
    7.437f,   6.954f,   6.467f,   5.98f,    5.487f,   4.997f,   4.501f,
    4.009f,   3.509f,   3.014f,   2.512f,   2.014f,   1.885f,   1.761f,
    1.637f,   1.511f,   1.386f,   1.258f,   1.13f,    1.009f,   0.88f,
    0.756f,   0.63f,    0.505f,   0.379f,   0.251f,   0.124f,   0.0f,
    -0.129f,  -0.254f,  -0.38f,   -0.506f,  -0.632f,  -0.76f,   -0.887f,
    -1.012f,  -1.141f,  -1.266f,  -1.393f,  -1.519f,  -1.646f,  -1.773f,
    -1.901f,  -2.027f,  -2.155f,  -2.282f,  -2.409f,  -2.535f,  -2.662f,
    -2.789f,  -2.916f,  -3.044f,  -3.172f,  -3.299f,  -3.425f,  -3.552f,
    -3.680f,  -3.806f,  -3.933f,  -4.062f,  -4.190f,  -4.318f,  -4.444f,
    -4.571f,  -4.698f,  -4.824f,  -4.951f,  -5.081f,  -5.209f,  -5.336f,
    -5.463f,  -5.589f,  -5.717f,  -5.843f,  -5.968f,  -6.099f,  -6.607f,
    -7.118f,  -7.624f,  -8.135f,  -8.64f,   -9.149f,  -9.652f,  -10.16f,
    -10.664f, -11.17f,  -11.67f,  -12.174f, -12.672f, -13.173f, -13.668f,
    -14.166f, -14.658f, -15.154f, -15.643f, -16.135f, -16.62f,  -17.108f,
    -17.59f,  -18.073f, -18.548f, -19.031f, -19.501f, -19.981f, -20.445f,
    -20.92f,  -21.379f, -21.85f,  -22.304f, -22.77f,  -23.219f, -23.68f,
    -24.123f, -25.016f,
};

static const float azimuthOffset[] = {
    3.257f,  3.263f, -1.083f, 3.268f, -1.086f, 3.273f,  -1.089f, 3.278f,
    -1.092f, 3.283f, -1.094f, 3.288f, -1.097f, 3.291f,  -1.1f,   1.1f,
    -1.102f, 1.1f,   -3.306f, 1.102f, -3.311f, 1.103f,  -3.318f, 1.105f,
    -3.324f, 1.106f, 7.72f,   5.535f, 3.325f,  -3.33f,  -1.114f, -5.538f,
    -7.726f, 1.108f, 7.731f,  5.543f, 3.329f,  -3.336f, -1.116f, -5.547f,
    -7.738f, 1.108f, 7.743f,  5.551f, 3.335f,  -3.342f, -1.119f, -5.555f,
    -7.75f,  1.11f,  7.757f,  5.56f,  3.34f,   -3.347f, -1.121f, -5.564f,
    -7.762f, 1.111f, 7.768f,  5.569f, 3.345f,  -3.353f, -1.123f, -5.573f,
    -7.775f, 1.113f, 7.780f,  5.578f, 3.351f,  -3.358f, -1.125f, -5.582f,
    -7.787f, 1.115f, 7.792f,  5.586f, 3.356f,  -3.363f, -1.126f, -5.591f,
    -7.799f, 1.117f, 7.804f,  5.595f, 3.36f,   -3.369f, -1.128f, -5.599f,
    -7.811f, 1.119f, -3.374f, 1.12f,  -3.379f, 1.122f,  -3.383f, 3.381f,
    -3.388f, 3.386f, -1.135f, 3.39f,  -1.137f, 3.395f,  -1.138f, 3.401f,
    -1.139f, 3.406f, -1.14f,  3.41f,  -1.141f, 3.416f,  -1.142f, 1.14f,
    -1.143f, 1.143f, -3.426f, 1.146f, -3.429f, 1.147f,  -3.433f, 1.15f,
    -3.436f, 1.152f, -3.44f,  1.154f, -3.443f, 1.157f,  -3.446f, -3.449f,
};
const char *PandarSwiftSDK::m_sPtcsModeSetFilePath = "../params/test.bin";

/** @brief Constructor. */
PandarSwiftSDK::PandarSwiftSDK(
    std::string deviceipaddr, uint16_t lidarport, uint16_t gpsport,
    std::string frameid, std::string correctionfile, std::string firtimeflie,
    std::string pcapfile,
    boost::function<void(boost::shared_ptr<PPointCloud>, double)> pclcallback,
    boost::function<void(PandarPacketsArray *)> rawcallback,
    boost::function<void(double)> gpscallback,
    boost::function<void(AT128FaultMessageInfo &)> faultmessagecallback,
    std::string certFile, std::string privateKeyFile, std::string caFile,
    int startangle, int timezone, int viewMode, \ 
							std::string publishmode, std::string multicast_ip,
    std::map<std::string, int32_t> threadPriority, std::string datatype) {
  m_sSdkVersion = "PandarSwiftSDK_1.2.37";
  printf("\n--------PandarSwift SDK version: %s--------\n",
         m_sSdkVersion.c_str());
  m_sDeviceIpAddr = deviceipaddr;
  m_sFrameId = frameid;
  m_sLidarFiretimeFile = firtimeflie;
  m_sLidarCorrectionFile = correctionfile;
  m_sPublishmodel = publishmode;
  m_iLidarRotationStartAngle = int(startangle * 100);
  m_sPcapFile = pcapfile;
  m_iWorkMode = 0;
  m_iReturnMode = 0;
  m_iMotorSpeed = 0;
  m_iLaserNum = 0;
  m_iTimeZoneSecond = timezone * 3600;  // time zone
  m_iPublishPointsIndex = 0;
  m_u8UdpVersionMajor = 0;
  m_u8UdpVersionMinor = 0;
  m_iFirstAzimuthIndex = 0;
  m_iLastAzimuthIndex = 0;
  m_dTimestamp = 0;
  m_bPublishPointsFlag = false;
  m_bClockwise == true;
  m_funcPclCallback = pclcallback;
  m_funcGpsCallback = gpscallback;
  m_funcFaultMessageCallback = faultmessagecallback;
  m_iViewMode = viewMode;
  m_iField = 0;
  m_sDatatype = datatype;
  m_bIsReadPcapOver = false;
  m_configMap = threadPriority;
  m_dAzimuthInterval = 0;
  m_sCaFilePath = caFile;
  m_sCertFilePath = certFile;
  m_sPrivateKeyFilePath = privateKeyFile;
  m_fTxTemperature = 0;
  m_fRxTemperature = 0;
  m_fMbTemperature = 0;
  m_fPbTemperature = 0;
  m_bGetCorrectionSuccess = false;
  m_iGetCorrectionCount = 0;
  system("echo 562144000 > /proc/sys/net/core/rmem_max");
  m_pTcpCommandClient =
      TcpCommandClientNew(m_sDeviceIpAddr.c_str(), PANDARSDK_TCP_COMMAND_PORT);
  if (!m_sPcapFile.empty()) {
    m_iEdgeAzimuthSize = 1600;
    m_PacketsBuffer.m_pcapFlag = 1;
  } else {
    m_iEdgeAzimuthSize = 200;
  }
  m_spPandarDriver.reset(new PandarSwiftDriver(deviceipaddr, lidarport, gpsport,
                                               frameid, pcapfile, rawcallback,
                                               this, multicast_ip, publishmode, datatype));
  TcpCommandSetSsl(certFile.c_str(), privateKeyFile.c_str(), caFile.c_str());
  // if (!m_sPcapFile.empty() && getPtcsLidarMode() != 1) {
    TcpCommandSetNonSsl();
  // }
  printf("frame id: %s\n", m_sFrameId.c_str());
  printf("lidar firetime file: %s\n", m_sLidarFiretimeFile.c_str());
  printf("lidar correction file: %s\n", m_sLidarCorrectionFile.c_str());
  SetEnvironmentVariableTZ();
  for (int i = 0; i < PANDAR128_LASER_NUM; i++) {
    m_fElevAngle[i] = elevAngle[i];
    m_fHorizatalAzimuth[i] = azimuthOffset[i];
  }
  loadCorrectionFile();
  loadOffsetFile(m_sLidarFiretimeFile);
  pthread_mutex_init(&m_RedundantPointLock, NULL);
  memset(m_fCosAllAngle, 0, sizeof(m_fCosAllAngle));
  memset(m_fSinAllAngle, 0, sizeof(m_fSinAllAngle));
  for (int j = 0; j < CIRCLE; j++) {
    float angle = static_cast<float>(j) / 100.0f;
    m_fCosAllAngle[j] = cosf(degreeToRadian(angle));
    m_fSinAllAngle[j] = sinf(degreeToRadian(angle));
  }
}

bool PandarSwiftSDK::loadCorrectionFile() {
  printf("Load correction file, count:%d\n", m_iGetCorrectionCount);
  m_iGetCorrectionCount++;
  bool loadCorrectionFileSuccess = false;
  int ret;
  if (m_sPcapFile
          .empty()) {  // connect to lidar,load correction file frome lidar
    if (NULL != m_pTcpCommandClient) {
      char *buffer = NULL;
      uint32_t len = 0;
      std::string correntionString;
      ret = TcpCommandGetLidarCalibration(m_pTcpCommandClient, &buffer, &len);

      if (ret == 0 && buffer) {
        printf("Load correction file from lidar now\n");
        ret = loadCorrectionString(buffer);
        if (ret != 0) {
          printf("Parse Lidar Correction Error\n");
        } else {
          loadCorrectionFileSuccess = true;
          m_bGetCorrectionSuccess = true;
          printf("Parse Lidar Correction Success!!!\n");
          return true;
        }
        free(buffer);
      } else {
        printf("Get lidar calibration filed\n");
        return false;
      }
    }
  }
  else {  // load correction file from locaf file
    printf("load correction file from local correction.csv now, count:%d\n", m_iGetCorrectionCount);
    std::ifstream fin(m_sLidarCorrectionFile);
    if (fin.is_open()) {
      printf("Open correction file success\n");
      int length = 0;
      std::string strlidarCalibration;
      fin.seekg(0, std::ios::end);
      length = fin.tellg();
      fin.seekg(0, std::ios::beg);
      char *buffer = new char[length];
      fin.read(buffer, length);
      fin.close();
      strlidarCalibration = buffer;
      ret = loadCorrectionString(buffer);
      if (ret != 0) {
        printf("Parse local Correction file Error\n");
      } else {
        m_bGetCorrectionSuccess = true;
        printf("Parse local Correction file Success!!!\n");
        return true;
      }
    } else {
      printf("Open correction file failed\n");
      return false;
    }
  }
  return false;
}

int PandarSwiftSDK::loadCorrectionString(char *data) {
  try {
    char *p = data;
    PandarATCorrectionsHeader header = *(PandarATCorrectionsHeader *)p;
    if (0xee == header.delimiter[0] && 0xff == header.delimiter[1]) {
      switch (header.version[1]) {
        case 3: {
          m_PandarAT_corrections.header = header;
          auto frame_num = m_PandarAT_corrections.header.frame_number;
          auto channel_num = m_PandarAT_corrections.header.channel_number;
          p += sizeof(PandarATCorrectionsHeader);
          memcpy((void *)&m_PandarAT_corrections.start_frame, p,
                 sizeof(uint16_t) * frame_num);
          p += sizeof(uint16_t) * frame_num;
          memcpy((void *)&m_PandarAT_corrections.end_frame, p,
                 sizeof(uint16_t) * frame_num);
          p += sizeof(uint16_t) * frame_num;
          printf("frame_num: %d\n", frame_num);
          printf("start_frame, end_frame: \n");
          for (int i = 0; i < frame_num; ++i)
            printf("%lf,   %lf\n",
                   m_PandarAT_corrections.start_frame[i] / 100.f,
                   m_PandarAT_corrections.end_frame[i] / 100.f);
          memcpy((void *)&m_PandarAT_corrections.azimuth, p,
                 sizeof(int16_t) * channel_num);
          p += sizeof(int16_t) * channel_num;
          memcpy((void *)&m_PandarAT_corrections.elevation, p,
                 sizeof(int16_t) * channel_num);
          p += sizeof(int16_t) * channel_num;
          memcpy((void *)&m_PandarAT_corrections.azimuth_offset, p,
                 sizeof(int8_t) * 36000);
          p += sizeof(int8_t) * 36000;
          memcpy((void *)&m_PandarAT_corrections.elevation_offset, p,
                 sizeof(int8_t) * 36000);
          p += sizeof(int8_t) * 36000;
          memcpy((void *)&m_PandarAT_corrections.SHA256, p,
                 sizeof(uint8_t) * 32);
          p += sizeof(uint8_t) * 32;

          // for (int i = 0; i < channel_num; i++) {
          //     horizatal_azimuth_[i] = m_PandarAT_corrections.azimuth[i] /
          //     100.f; elev_angle_[i] = m_PandarAT_corrections.elevation[i] /
          //     100.f;

          // }
          return 0;
        } break;
        case 5: {
          m_PandarAT_corrections.header = header;
          auto frame_num = m_PandarAT_corrections.header.frame_number;
          auto channel_num = m_PandarAT_corrections.header.channel_number;
          p += sizeof(PandarATCorrectionsHeader);
          memcpy((void *)&m_PandarAT_corrections.l.start_frame, p,
                 sizeof(uint32_t) * frame_num);
          p += sizeof(uint32_t) * frame_num;
          memcpy((void *)&m_PandarAT_corrections.l.end_frame, p,
                 sizeof(uint32_t) * frame_num);
          p += sizeof(uint32_t) * frame_num;
          memcpy((void *)&m_PandarAT_corrections.l.azimuth, p,
                 sizeof(int32_t) * channel_num);
          p += sizeof(int32_t) * channel_num;
          memcpy((void *)&m_PandarAT_corrections.l.elevation, p,
                 sizeof(int32_t) * channel_num);
          p += sizeof(int32_t) * channel_num;
          auto adjust_length = channel_num * 180;
          memcpy((void *)&m_PandarAT_corrections.azimuth_offset, p,
                 sizeof(int8_t) * adjust_length);
          p += sizeof(int8_t) * adjust_length;
          memcpy((void *)&m_PandarAT_corrections.elevation_offset, p,
                 sizeof(int8_t) * adjust_length);
          p += sizeof(int8_t) * adjust_length;
          memcpy((void *)&m_PandarAT_corrections.SHA256, p,
                 sizeof(uint8_t) * 32);
          p += sizeof(uint8_t) * 32;
          printf("frame_num: %d\n", frame_num);
          printf("start_frame, end_frame: \n");
          for (int i = 0; i < frame_num; ++i) {
            m_PandarAT_corrections.l.start_frame[i] =
                m_PandarAT_corrections.l.start_frame[i] *
                m_PandarAT_corrections.header.resolution;
            m_PandarAT_corrections.l.end_frame[i] =
                m_PandarAT_corrections.l.end_frame[i] *
                m_PandarAT_corrections.header.resolution;
            printf("%lf,   %lf\n",
                   m_PandarAT_corrections.l.start_frame[i] / 25600.f,
                   m_PandarAT_corrections.l.end_frame[i] / 25600.f);
          }

          for (int i = 0; i < PANDAR128_LASER_NUM; i++) {
            m_PandarAT_corrections.l.azimuth[i] =
                m_PandarAT_corrections.l.azimuth[i] *
                m_PandarAT_corrections.header.resolution;
            m_PandarAT_corrections.l.elevation[i] =
                m_PandarAT_corrections.l.elevation[i] *
                m_PandarAT_corrections.header.resolution;
          }
          for (int i = 0; i < adjust_length; i++) {
            m_PandarAT_corrections.azimuth_offset[i] =
                m_PandarAT_corrections.azimuth_offset[i] *
                m_PandarAT_corrections.header.resolution;
            m_PandarAT_corrections.elevation_offset[i] =
                m_PandarAT_corrections.elevation_offset[i] *
                m_PandarAT_corrections.header.resolution;
          }

          return 0;
        } break;
        default:
          break;
      }
    }

    return 1;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }

  return 1;
}

void PandarSwiftSDK::driverReadThread() {
  int32_t priority = m_configMap["read_thread"];
  if (priority > 0) {
    SetThreadPriority(SCHED_FIFO, priority);
  }
  pthread_setname_np(pthread_self(), "lidar_sdk_read");
  while (1) {
    boost::this_thread::interruption_point();
    m_spPandarDriver->poll();
  }
}

void PandarSwiftSDK::publishRawDataThread() {
  int32_t priority = m_configMap["publish_thread"];
  if (priority > 0) {
    SetThreadPriority(SCHED_FIFO, priority);
  }
  pthread_setname_np(pthread_self(), "lidar_sdk_pub");
  while (1) {
    boost::this_thread::interruption_point();
    m_spPandarDriver->publishRawData();
  }
}

void PandarSwiftSDK::pushLiDARData(PandarPacket packet) {
  //  printf("PandarSwiftSDK::pushLiDARData");
  m_PacketsBuffer.push_back(packet);
  // printf("%d, %d\n",pkt.blocks[0].fAzimuth,pkt.blocks[1].fAzimuth);
}

void PandarSwiftSDK::stop() {
  if (m_driverReadThread) {
    m_driverReadThread->interrupt();
    m_driverReadThread->join();
    delete m_driverReadThread;
    m_driverReadThread = NULL;
  }

  if (m_processLiDARDataThread) {
    m_processLiDARDataThread->interrupt();
    m_processLiDARDataThread->join();
    delete m_processLiDARDataThread;
    m_processLiDARDataThread = NULL;
  }

  if (m_publishPointsThread) {
    m_publishPointsThread->interrupt();
    m_publishPointsThread->join();
    delete m_publishPointsThread;
    m_publishPointsThread = NULL;
  }

  if (m_publishRawDataThread) {
    m_publishRawDataThread->interrupt();
    m_publishRawDataThread->join();
    delete m_publishRawDataThread;
    m_publishRawDataThread = NULL;
  }
  return;
}

void PandarSwiftSDK::start() {
  stop();
  if (LIDAR_DATA_TYPE == m_sDatatype) {
    m_driverReadThread =
        new boost::thread(boost::bind(&PandarSwiftSDK::driverReadThread, this));
  }
  if (m_sPublishmodel == "both_point_raw" || m_sPublishmodel == "point" ||
      LIDAR_DATA_TYPE != m_sDatatype) {
    m_processLiDARDataThread =
        new boost::thread(boost::bind(&PandarSwiftSDK::processLiDARData, this));
  }
  if ((m_sPublishmodel == "both_point_raw" || m_sPublishmodel == "raw") &&
      LIDAR_DATA_TYPE == m_sDatatype) {
    m_publishRawDataThread = new boost::thread(
        boost::bind(&PandarSwiftSDK::publishRawDataThread, this));
  }
}

int PandarSwiftSDK::parseData(Pandar128PacketVersion13 &packet,
                              const uint8_t *recvbuf, const int len) {
  int index = 0;
  if (recvbuf[0] != 0xEE && recvbuf[1] != 0xFF && recvbuf[2] != 1) {
    printf("Lidar type is error %d\n", len);
    return -1;
  }
  if (3 == recvbuf[3]) {  // udp version :1.3
    memcpy(&packet, recvbuf, sizeof(Pandar128PacketVersion13));
    return 0;
  }
  if (4 == recvbuf[3]) {  // udp version 1.4
    memcpy(&(packet.head), &recvbuf[index], PANDAR128_HEAD_SIZE);

    index += PANDAR128_HEAD_SIZE;
    memcpy(&(packet.blocks), &recvbuf[index],
           PANDAR128_BLOCK_SIZE * PANDAR128_BLOCK_NUM);

    index += PANDAR128_BLOCK_SIZE * PANDAR128_BLOCK_NUM + PANDAR128_CRC_SIZE +
             PANDAR128_FUNCTION_SAFETY_SIZE;
    memcpy(&(packet.tail.nReserved1[0]), &recvbuf[index], 3);

    index += 3;
    memcpy(&(packet.tail.nReserved2[0]), &recvbuf[index], 3);

    index += 3 + 3 + 2;
    packet.tail.nShutdownFlag = (recvbuf[index] & 0x0F) |
                                (recvbuf[index + 1] & 0xC0) |
                                (recvbuf[index + 1] & 0x30);

    index += 1;
    packet.tail.nReturnMode = recvbuf[index];

    index += 1;
    packet.tail.nMotorSpeed =
        (recvbuf[index] & 0xff) | ((recvbuf[index + 1] & 0xff) << 8);

    index += 2;
    memcpy(&(packet.tail.nUTCTime[0]), &recvbuf[index], 6);

    index += 6;
    memcpy(&(packet.tail.nTimestamp), &recvbuf[index], 4);

    index += 4 + 1;
    memcpy(&(packet.tail.nSeqNum), &recvbuf[index], 4);
    return 0;
  }
  return -1;
}

int PandarSwiftSDK::processLiDARData() {
  int32_t priority = m_configMap["process_thread"];
  if (priority > 0) {
    SetThreadPriority(SCHED_FIFO, priority);
  }
  pthread_setname_np(pthread_self(), "lidar_sdk_proc");
  double lastTimestamp = 0.0;
  struct timespec ts;
  int ret = 0;
  int cursor = 0;
  init();
  uint32_t pclCallbackStartTime = GetTickCount();
  uint32_t pclCallbackEndTime = GetTickCount();
  while (1) {
    pclCallbackEndTime = GetTickCount();
    if ((pclCallbackEndTime - pclCallbackStartTime) > m_configMap["untragger_pclcallback_warning_time"]) {
      printf("can not trgger pclCallback for %d ms\n", m_configMap["untragger_pclcallback_warning_time"]);
      pclCallbackStartTime = pclCallbackEndTime;
    }
    boost::this_thread::interruption_point();
    if ((!m_PacketsBuffer.hasEnoughPackets())) {
      usleep(1000);
      if (!m_bIsSocketTimeout || m_PacketsBuffer.empty()) {
        continue;
      }
    }

    if (0 == checkLiadaMode()) {
      // printf("checkLiadaMode now!!");
      m_OutMsgArray[cursor]->clear();
      m_OutMsgArray[cursor]->resize(calculatePointBufferSize());
      // m_PacketsBuffer.creatNewTask();
      continue;
    }
    if (!m_bGetCorrectionSuccess) 
    {
      if (!loadCorrectionFile()) {
        usleep(10000);
        continue;
      }
    }
    if (isNeedPublish()) {  // Judging whether pass the  start angle
      moveTaskEndToStartAngle();
      doTaskFlow(cursor);
      m_iPublishPointsIndex = cursor;
      cursor = (cursor + 1) % 2;
      publishPoints();
      m_OutMsgArray[cursor]->clear();
      m_OutMsgArray[cursor]->resize(calculatePointBufferSize());
      m_bIsSocketTimeout = false;
      m_OutMsgArray[cursor]->header.frame_id = m_sFrameId;
      m_OutMsgArray[cursor]->height = 1;
      pclCallbackStartTime = pclCallbackEndTime;
      continue;
    }
    // uint32_t taskflow1 = GetTickCount();
    if ((m_PacketsBuffer.hasEnoughPackets())) doTaskFlow(cursor);
    // uint32_t taskflow2 = GetTickCount();
    // printf("taskflow time: %d\n", taskflow2 - taskflow1);
  }
}

void PandarSwiftSDK::moveTaskEndToStartAngle() {
  // uint32_t startTick = GetTickCount();
  if ((m_PacketsBuffer.m_iterPush - m_PacketsBuffer.m_buffers.begin()) > 2) {
    if (m_sPcapFile != "") {
      m_PacketsBuffer.moveTaskEnd(m_PacketsBuffer.m_iterPush - 1);
    } else {
      m_PacketsBuffer.moveTaskEnd(m_PacketsBuffer.m_iterPush);
    }
  }
  // uint32_t endTick = GetTickCount();
  // printf("moveTaskEndToStartAngle time: %d\n", endTick - startTick);
}

void PandarSwiftSDK::publishPoints() {
  // uint32_t start = GetTickCount();
  if (m_dTimestamp == 0) return;
  int point_size = 0;
  for (int i = 0; i < m_OutMsgArray[m_iPublishPointsIndex]->points.size();
       i++) {
    if (m_OutMsgArray[m_iPublishPointsIndex]->points[i].ring != 0) {
      point_size++;
    }
  }
  m_PublishMsgArray->clear();
  m_PublishMsgArray->resize(point_size);
  point_size = 0;
  for (int i = 0; i < m_OutMsgArray[m_iPublishPointsIndex]->points.size();
       i++) {
    if (m_OutMsgArray[m_iPublishPointsIndex]->points[i].ring != 0) {
      m_PublishMsgArray->points[point_size] =
          m_OutMsgArray[m_iPublishPointsIndex]->points[i];
      point_size++;
    }
  }
  m_PublishMsgArray->header.frame_id = m_sFrameId;
  m_PublishMsgArray->height = 1;
  if (point_size < MIN_POINT_NUM || point_size > MAX_POINT_NUM) {
    printf("point num in this frame is wrong, num = %d\n",point_size);
    return;
  }
  if (NULL != m_funcPclCallback) {
    m_funcPclCallback(m_PublishMsgArray, m_dTimestamp);
    m_dTimestamp = 0;
#ifdef PRINT_TEMPERATURE
    printf("TX:%f, RX:%f, MB:%f, PB:%f\n", getTxTemperature(),
           getRxTemperature(), getMbTemperature(), getPbTemperature());
#endif
  }
  // uint32_t end = GetTickCount();
  // if(end - start > 150) printf("publishPoints time:%d\n", end - start);
}

void PandarSwiftSDK::doTaskFlow(int cursor) {
  tf::Taskflow taskFlow;
  taskFlow.parallel_for(
      m_PacketsBuffer.getTaskBegin(), m_PacketsBuffer.getTaskEnd(),
      [this, cursor](auto &taskpkt) { calcPointXYZIT(taskpkt, cursor); });
  executor.run(taskFlow).wait();
  m_PacketsBuffer.creatNewTask();
}

void PandarSwiftSDK::init() {
  int count = 0;
  int Azimuth = 0;
  setTimeStampNum(m_configMap["timestamp_num"]);
  while (1) {
    boost::this_thread::interruption_point();
    if (!m_PacketsBuffer.hasEnoughPackets()) {
      usleep(1000);
      continue;
    }
    int16_t lidarmotorspeed = 0;
    if (m_PacketsBuffer.getTaskBegin()->data[0] != 0xEE) {
      m_PacketsBuffer.m_iterTaskBegin++;
    }
    auto header = (PandarAT128Head *)(&(
        (m_PacketsBuffer.getTaskBegin() + count)->data[0]));
    count++;
    switch (header->u8VersionMinor) {
      case 3: {
        auto tail =
            (PandarAT128TailVersion43
                 *)(&((m_PacketsBuffer.getTaskBegin() + count)->data[0]) +
                    PANDAR_AT128_HEAD_SIZE +
                    (header->hasConfidence()
                         ? PANDAR_AT128_UNIT_WITH_CONFIDENCE_SIZE *
                               header->u8LaserNum * header->u8BlockNum
                         : PANDAR_AT128_UNIT_WITHOUT_CONFIDENCE_SIZE *
                               header->u8LaserNum * header->u8BlockNum) +
                    PANDAR_AT128_CRC_SIZE +
                    PANDAR_AT128_AZIMUTH_SIZE * header->u8BlockNum +
                    PANDAR_AT128_FINE_AZIMUTH_SIZE * header->u8BlockNum);
        m_iWorkMode = tail->nShutdownFlag & 0x03;
        m_iReturnMode = tail->nReturnMode;
        lidarmotorspeed = tail->nMotorSpeed / 10;
        m_iLaserNum = header->u8LaserNum;
        m_iFirstAzimuthIndex = PANDAR_AT128_HEAD_SIZE;
        m_iLastAzimuthIndex =
            PANDAR_AT128_HEAD_SIZE +
            (header->hasConfidence()
                 ? PANDAR_AT128_UNIT_WITH_CONFIDENCE_SIZE * header->u8LaserNum *
                       (header->u8BlockNum - 1)
                 : PANDAR_AT128_UNIT_WITHOUT_CONFIDENCE_SIZE *
                       header->u8LaserNum * (header->u8BlockNum - 1)) +
            PANDAR_AT128_AZIMUTH_SIZE * (header->u8BlockNum - 1) +
            PANDAR_AT128_FINE_AZIMUTH_SIZE * (header->u8BlockNum - 1);
        m_spPandarDriver->setUdpVersion(header->u8VersionMajor,
                                        header->u8VersionMinor);
        m_u8UdpVersionMinor = header->u8VersionMinor;
        Azimuth =
            *(uint16_t *)(&((m_PacketsBuffer.getTaskBegin() + count)->data[0]) +
                          m_iFirstAzimuthIndex) *
            LIDAR_AZIMUTH_UNIT;
        int field_count = 0;
        m_iField = 0;
        while (field_count < m_PandarAT_corrections.header.frame_number &&
               (((Azimuth + MAX_AZI_LEN -
                  m_PandarAT_corrections.l.start_frame[m_iField]) %
                     MAX_AZI_LEN +
                 (m_PandarAT_corrections.l.end_frame[m_iField] + MAX_AZI_LEN -
                  Azimuth) %
                     MAX_AZI_LEN) !=
                (m_PandarAT_corrections.l.end_frame[m_iField] + MAX_AZI_LEN -
                 m_PandarAT_corrections.l.start_frame[m_iField]) %
                    MAX_AZI_LEN)) {
          m_iField =
              (m_iField + 1) % m_PandarAT_corrections.header.frame_number;
          field_count++;
        }
        if (field_count >= m_PandarAT_corrections.header.frame_number)
          printf("correction file or azimuth is wrong\n");

      } break;
      default:
        usleep(1000);
        continue;
        break;
    }
    if (abs(abs(lidarmotorspeed) - MOTOR_SPEED_600) <
        30) {  // ignore the speed gap of 600 rpm
      m_iMotorSpeed = MOTOR_SPEED_600;
    } else if (abs(abs(lidarmotorspeed) - MOTOR_SPEED_750) <
               30) {  // ignore the speed gap of 750 rpm
      m_iMotorSpeed = MOTOR_SPEED_750;
    } else if (abs(abs(lidarmotorspeed) - MOTOR_SPEED_500) <
               30) {  // ignore the speed gap of 500 rpm
      m_iMotorSpeed = MOTOR_SPEED_500;
    } else if (abs(abs(lidarmotorspeed) - MOTOR_SPEED_400) <
               30) {  // ignore the speed gap of 400 rpm
      m_iMotorSpeed = MOTOR_SPEED_400;
    } else if (abs(abs(lidarmotorspeed) - MOTOR_SPEED_300) <
               30) {  // ignore the speed gap of 300 rpm
      m_iMotorSpeed = MOTOR_SPEED_300;
    } else if (abs(abs(lidarmotorspeed) - MOTOR_SPEED_200) <
               30) {  // ignore the speed gap of 200 rpm
      m_iMotorSpeed = MOTOR_SPEED_200;
    } else if (abs(abs(lidarmotorspeed) - MOTOR_SPEED_150) <
               30) {  // ignore the speed gap of 150 rpm
      m_iMotorSpeed = MOTOR_SPEED_150;
    } else {
      m_iMotorSpeed = MOTOR_SPEED_200;  // changing the speed,give enough size
    }
    printf("init mode: workermode: %x,return mode: %x,speed: %d\n", m_iWorkMode,
           m_iReturnMode, m_iMotorSpeed);
    changeAngleSize();
    changeReturnBlockSize();
    checkClockwise(lidarmotorspeed);
    boost::shared_ptr<PPointCloud> outMag0(
        new PPointCloud(calculatePointBufferSize(), 1));
    boost::shared_ptr<PPointCloud> outMag1(
        new PPointCloud(calculatePointBufferSize(), 1));
    boost::shared_ptr<PPointCloud> publishMsg(
        new PPointCloud(calculatePointBufferSize(), 1));

    m_OutMsgArray[0] = outMag0;
    m_OutMsgArray[1] = outMag1;
    m_PublishMsgArray = publishMsg;
    break;
  }
}

void PandarSwiftSDK::checkClockwise(int16_t lidarmotorspeed) {
  m_bClockwise = (lidarmotorspeed >= 0);
}

int PandarSwiftSDK::checkLiadaMode() {
  uint8_t lidarworkmode = 0;
  uint8_t lidarreturnmode = 0;
  uint16_t lidarmotorspeed = 0;
  uint8_t laserNum = 0;
  uint8_t blockNum = 0;
  if ((m_PacketsBuffer.getTaskBegin() - m_PacketsBuffer.m_buffers.begin()) < 4)
    m_iField = -1;
  if (((m_PacketsBuffer.m_buffers.end() - m_PacketsBuffer.getTaskBegin()) <=
       2) ||
      (abs(m_PacketsBuffer.m_iterPush - m_PacketsBuffer.getTaskBegin()) <= 2) ||
      ((m_PacketsBuffer.getTaskBegin() + 1)->data[0] != 0xEE)) {
    m_iField = -1;
    // printf("field %d %d`` %d %d\n", m_iField,m_PacketsBuffer.m_iterPush -
    // m_PacketsBuffer.getTaskBegin(), m_PacketsBuffer.getTaskBegin() + 1 -
    // m_PacketsBuffer.m_buffers.begin(), m_PacketsBuffer.m_buffers.end() -
    // m_PacketsBuffer.getTaskBegin());
    return 1;
  }
  int Azimuth = 0;
  auto header =
      (PandarAT128Head *)(&((m_PacketsBuffer.getTaskBegin() + 1)->data[0]));
  switch (header->u8VersionMinor) {
    case 3: {
      auto tail = (PandarAT128TailVersion43
                       *)(&((m_PacketsBuffer.getTaskBegin() + 1)->data[0]) +
                          PANDAR_AT128_HEAD_SIZE +
                          (header->hasConfidence()
                               ? PANDAR_AT128_UNIT_WITH_CONFIDENCE_SIZE *
                                     header->u8LaserNum * header->u8BlockNum
                               : PANDAR_AT128_UNIT_WITHOUT_CONFIDENCE_SIZE *
                                     header->u8LaserNum * header->u8BlockNum) +
                          PANDAR_AT128_CRC_SIZE +
                          PANDAR_AT128_AZIMUTH_SIZE * header->u8BlockNum +
                          PANDAR_AT128_FINE_AZIMUTH_SIZE * header->u8BlockNum);
      lidarworkmode = tail->nShutdownFlag & 0x03;
      lidarreturnmode = tail->nReturnMode;
      lidarmotorspeed = tail->nMotorSpeed / 10;
      checkClockwise(lidarmotorspeed);
      laserNum = header->u8LaserNum;
      blockNum = header->u8BlockNum;
      m_iFirstAzimuthIndex = PANDAR_AT128_HEAD_SIZE;
      m_iLastAzimuthIndex =
          PANDAR_AT128_HEAD_SIZE +
          (header->hasConfidence()
               ? PANDAR_AT128_UNIT_WITH_CONFIDENCE_SIZE * header->u8LaserNum *
                     (header->u8BlockNum - 1)
               : PANDAR_AT128_UNIT_WITHOUT_CONFIDENCE_SIZE *
                     header->u8LaserNum * (header->u8BlockNum - 1)) +
          PANDAR_AT128_AZIMUTH_SIZE * (header->u8BlockNum - 1) +
          PANDAR_AT128_FINE_AZIMUTH_SIZE * (header->u8BlockNum - 1);
      Azimuth = *(uint16_t *)(&((m_PacketsBuffer.getTaskBegin() + 1)->data[0]) +
                              m_iFirstAzimuthIndex) *
                LIDAR_AZIMUTH_UNIT;
      int field_count = 0;
      m_iField = 0;
      while (field_count < m_PandarAT_corrections.header.frame_number &&
             (((Azimuth + MAX_AZI_LEN -
                m_PandarAT_corrections.l.start_frame[m_iField]) %
                   MAX_AZI_LEN +
               (m_PandarAT_corrections.l.end_frame[m_iField] + MAX_AZI_LEN -
                Azimuth) %
                   MAX_AZI_LEN) !=
              (m_PandarAT_corrections.l.end_frame[m_iField] + MAX_AZI_LEN -
               m_PandarAT_corrections.l.start_frame[m_iField]) %
                  MAX_AZI_LEN)) {
        m_iField = (m_iField + 1) % m_PandarAT_corrections.header.frame_number;
        field_count++;
      }
      if (field_count >= m_PandarAT_corrections.header.frame_number) {
        m_iField = -1;
        return 1;
      }
      // printf("field %f %d %d\n", Azimuth/25600.0f, m_iField,
      // m_PacketsBuffer.getTaskBegin() + 1 -
      // m_PacketsBuffer.m_buffers.begin());

    } break;
    default:
      m_iField = -1;
      break;
  }
  if (abs(abs(lidarmotorspeed) - MOTOR_SPEED_600) <
      30) {  // ignore the speed gap of 600 rpm
    lidarmotorspeed = MOTOR_SPEED_600;
  } else if (abs(abs(lidarmotorspeed) - MOTOR_SPEED_750) <
             30) {  // ignore the speed gap of 750 rpm
    lidarmotorspeed = MOTOR_SPEED_750;
  } else if (abs(abs(lidarmotorspeed) - MOTOR_SPEED_500) <
             30) {  // ignore the speed gap of 500 rpm
    lidarmotorspeed = MOTOR_SPEED_500;
  } else if (abs(abs(lidarmotorspeed) - MOTOR_SPEED_400) <
             30) {  // ignore the speed gap of 400 rpm
    lidarmotorspeed = MOTOR_SPEED_400;
  } else if (abs(abs(lidarmotorspeed) - MOTOR_SPEED_300) <
             30) {  // ignore the speed gap of 300 rpm
    lidarmotorspeed = MOTOR_SPEED_300;
  } else if (abs(abs(lidarmotorspeed) - MOTOR_SPEED_200) <
             30) {  // ignore the speed gap of 200 rpm
    lidarmotorspeed = MOTOR_SPEED_200;
  } else if (abs(abs(lidarmotorspeed) - MOTOR_SPEED_150) <
             30) {  // ignore the speed gap of 150 rpm
    lidarmotorspeed = MOTOR_SPEED_150;
  } else {
    lidarmotorspeed = MOTOR_SPEED_200;  // changing the speed,give enough size
  }

  if (0 == m_iWorkMode && 0 == m_iReturnMode && 0 == m_iMotorSpeed &&
      0 == m_iLaserNum) {  // init lidar mode
    m_iWorkMode = lidarworkmode;
    m_iReturnMode = lidarreturnmode;
    m_iMotorSpeed = lidarmotorspeed;
    m_iLaserNum = laserNum;
    printf(
        "init mode: workermode: %x,return mode: %x,speed: %d,laser number: %d\n",
        m_iWorkMode, m_iReturnMode, m_iMotorSpeed, m_iLaserNum);
    changeAngleSize();
    changeReturnBlockSize();
    // checkClockwise();
    boost::shared_ptr<PPointCloud> outMag0(
        new PPointCloud(calculatePointBufferSize(), 1));
    boost::shared_ptr<PPointCloud> outMag1(
        new PPointCloud(calculatePointBufferSize(), 1));
    m_OutMsgArray[0] = outMag0;
    m_OutMsgArray[1] = outMag1;
    return 1;
  } else {
    if (m_iReturnMode != lidarreturnmode) {  // return mode change
      printf("change return mode:  %x to %x \n", m_iReturnMode, lidarreturnmode);
      m_iReturnMode = lidarreturnmode;
      changeReturnBlockSize();
      return 0;
    }
    if (m_iMotorSpeed != lidarmotorspeed) {  // motor speed change
      printf("change motor speed:  %d to %d \n",m_iMotorSpeed,
      lidarmotorspeed);
      m_iMotorSpeed = lidarmotorspeed;
      changeAngleSize();
      return 0;
    }
    if (m_iLaserNum != laserNum) {  // laser number change
      m_iLaserNum = laserNum;
      return 0;
    }
    return 1;
  }
}

void PandarSwiftSDK::changeAngleSize() {
  switch (m_u8UdpVersionMinor) {
    case 1: {
      if (MOTOR_SPEED_600 == m_iMotorSpeed ||
          MOTOR_SPEED_400 == m_iMotorSpeed) {
        m_iAngleSize = LIDAR_ANGLE_SIZE_10;  // 10->0.1degree
      } else if (MOTOR_SPEED_300 == m_iMotorSpeed ||
                 MOTOR_SPEED_200 == m_iMotorSpeed) {
        m_iAngleSize = LIDAR_ANGLE_SIZE_5;  // 5->0.05degree
      }
    } break;
    case 3: {
      if (MOTOR_SPEED_600 == m_iMotorSpeed) {
        m_iAngleSize = LIDAR_ANGLE_SIZE_15;  // 10->0.15degree
      } else if (MOTOR_SPEED_750 == m_iMotorSpeed) {
        m_iAngleSize = LIDAR_ANGLE_SIZE_18_75;  // 18_75->0.01875degree
      } else if (MOTOR_SPEED_500 == m_iMotorSpeed) {
        m_iAngleSize = LIDAR_ANGLE_SIZE_12_5;  // 12_5->0.0125degree
      } else if (MOTOR_SPEED_400 == m_iMotorSpeed) {
        m_iAngleSize = LIDAR_ANGLE_SIZE_10;  // 10->0.1degree
      } else if (MOTOR_SPEED_300 == m_iMotorSpeed) {
        m_iAngleSize = LIDAR_ANGLE_SIZE_7_5;  // 7_5->0.075degree
      } else if (MOTOR_SPEED_200 == m_iMotorSpeed) {
        m_iAngleSize = LIDAR_ANGLE_SIZE_5;  // 5->0.05degree
      } else if (MOTOR_SPEED_150 == m_iMotorSpeed) {
        m_iAngleSize = LIDAR_ANGLE_SIZE_3_75;  // 3_75->0.0375degree
      }
    } break;
    default:
      break;
  }
}

void PandarSwiftSDK::changeReturnBlockSize() {
  if (0x39 == m_iReturnMode || 0x3b == m_iReturnMode || 0x3c == m_iReturnMode) {
    m_iReturnBlockSize = LIDAR_RETURN_BLOCK_SIZE_2;
  } else {
    m_iReturnBlockSize = LIDAR_RETURN_BLOCK_SIZE_1;
  }
}

void PandarSwiftSDK::calcPointXYZIT(PandarPacket &packet, int cursor) {
  if (packet.data[0] != 0xEE || packet.data[1] != 0xFF) return;
  auto header = (PandarAT128Head *)(&packet.data[0]);
  switch (header->u8VersionMinor) {
    case 3: {
      auto tail = (PandarAT128TailVersion43
                       *)(&(packet.data[0]) + PANDAR_AT128_HEAD_SIZE +
                          (header->hasConfidence()
                               ? PANDAR_AT128_UNIT_WITH_CONFIDENCE_SIZE *
                                     header->u8LaserNum * header->u8BlockNum
                               : PANDAR_AT128_UNIT_WITHOUT_CONFIDENCE_SIZE *
                                     header->u8LaserNum * header->u8BlockNum) +
                          PANDAR_AT128_CRC_SIZE +
                          PANDAR_AT128_AZIMUTH_SIZE * header->u8BlockNum +
                          PANDAR_AT128_FINE_AZIMUTH_SIZE * header->u8BlockNum);
      updateMoniterInfo(tail->nReserved2[2],
                        *(uint16_t *)(&tail->nReserved2[0]));
      if (m_dAzimuthInterval != 0) {
        uint16_t beginAzimuth =
            *(uint16_t *)(&((m_PacketsBuffer.getTaskBegin())->data[0]) +
                          m_iLastAzimuthIndex);
        uint16_t endAzimuth =
            *(uint16_t *)(&((m_PacketsBuffer.getTaskEnd() - 2)->data[0]) +
                          m_iFirstAzimuthIndex);
        uint16_t azimuth =
            *(uint16_t *)(&(packet.data[0]) + m_iLastAzimuthIndex);
        if (endAzimuth < beginAzimuth) {
          endAzimuth += CIRCLE_ANGLE;
        }
        if (azimuth < beginAzimuth) {
          azimuth += CIRCLE_ANGLE;
        }
        int timeStampIndex = (azimuth - beginAzimuth + m_dAzimuthInterval / 2) /
                             m_dAzimuthInterval;
        float taskFlowTimestampNum =
            (endAzimuth - beginAzimuth) / m_dAzimuthInterval;
        float taskFlowPacketNum =
            m_PacketsBuffer.getTaskEnd() - m_PacketsBuffer.getTaskBegin();
        int packetIndex =
            timeStampIndex * taskFlowPacketNum / taskFlowTimestampNum;

        if ((packetIndex >= taskFlowPacketNum))
          packetIndex = taskFlowPacketNum - 1;
        if (taskFlowTimestampNum < 5) packetIndex = 0;
        tail =
            (PandarAT128TailVersion43
                 *)(&((m_PacketsBuffer.getTaskBegin() + packetIndex)->data[0]) +
                    PANDAR_AT128_HEAD_SIZE +
                    (header->hasConfidence()
                         ? PANDAR_AT128_UNIT_WITH_CONFIDENCE_SIZE *
                               header->u8LaserNum * header->u8BlockNum
                         : PANDAR_AT128_UNIT_WITHOUT_CONFIDENCE_SIZE *
                               header->u8LaserNum * header->u8BlockNum) +
                    PANDAR_AT128_CRC_SIZE +
                    PANDAR_AT128_AZIMUTH_SIZE * header->u8BlockNum +
                    PANDAR_AT128_FINE_AZIMUTH_SIZE * header->u8BlockNum);
      }
      double unix_second = 0;
      if (tail->nUTCTime[0] != 0) {
        struct tm t = {0};
        t.tm_year = tail->nUTCTime[0];
        if (t.tm_year >= 200) {
          t.tm_year -= 100;
        }
        t.tm_mon = tail->nUTCTime[1] - 1;
        t.tm_mday = tail->nUTCTime[2];
        t.tm_hour = tail->nUTCTime[3];
        t.tm_min = tail->nUTCTime[4];
        t.tm_sec = tail->nUTCTime[5];
        t.tm_isdst = 0;

        unix_second = static_cast<double>(mktime(&t) + m_iTimeZoneSecond);
      } else {
        uint32_t utc_time_big = *(uint32_t *)(&tail->nUTCTime[0] + 2);
        unix_second = ((utc_time_big >> 24) & 0xff) |
                      ((utc_time_big >> 8) & 0xff00) |
                      ((utc_time_big << 8) & 0xff0000) | ((utc_time_big << 24));
      }
      int index = 0;
      index += PANDAR_AT128_HEAD_SIZE;
      for (int blockid = 0; blockid < header->u8BlockNum; blockid++) {
        uint16_t u16Azimuth = *(uint16_t *)(&packet.data[0] + index);
        index += PANDAR_AT128_AZIMUTH_SIZE;
        uint8_t u8FineAzimuth = *(uint8_t *)(&packet.data[0] + index);
        index += PANDAR_AT128_FINE_AZIMUTH_SIZE;
        int Azimuth = u16Azimuth * 256 + u8FineAzimuth;
        // ROS_WARN("#####block.fAzimuth[%u]",u16Azimuth);
        int count = 0, field = 0;
        while (count < m_PandarAT_corrections.header.frame_number &&
               (((Azimuth + MAX_AZI_LEN -
                  m_PandarAT_corrections.l.start_frame[field]) %
                     MAX_AZI_LEN +
                 (m_PandarAT_corrections.l.end_frame[field] + MAX_AZI_LEN -
                  Azimuth) %
                     MAX_AZI_LEN) !=
                (m_PandarAT_corrections.l.end_frame[field] + MAX_AZI_LEN -
                 m_PandarAT_corrections.l.start_frame[field]) %
                    MAX_AZI_LEN)) {
          field = (field + 1) % m_PandarAT_corrections.header.frame_number;
          count++;
        }
        if (count >= m_PandarAT_corrections.header.frame_number) continue;
        for (int i = 0; i < header->u8LaserNum; i++) {
          /* for all the units in a block */
          uint16_t u16Distance = *(uint16_t *)(&packet.data[0] + index);
          index += DISTANCE_SIZE;
          uint8_t u8Intensity = *(uint8_t *)(&packet.data[0] + index);
          index += INTENSITY_SIZE;
          if (header->u8VersionMinor == 1 || header->hasConfidence())
            index += CONFIDENCE_SIZE;
          PPoint point;
          point.timestamp =
              unix_second + static_cast<double>(tail->nTimestamp) / 1000000.0;
          float distance =
              static_cast<float>(u16Distance) * PANDAR128_DISTANCE_UNIT;
          auto elevation =
              (m_PandarAT_corrections.l.elevation[i] +
               m_PandarAT_corrections.getElevationAdjustV3(i, Azimuth) * 256);
          elevation = (MAX_AZI_LEN + elevation) % MAX_AZI_LEN;
          auto azimuth =
              ((Azimuth + MAX_AZI_LEN -
                m_PandarAT_corrections.l.start_frame[field]) *
                   2 -
               m_PandarAT_corrections.l.azimuth[i] +
               m_PandarAT_corrections.getAzimuthAdjustV3(i, Azimuth) * 256);
#ifdef ENABLE_FIRETIME_CORRECTION
          if (m_dAzimuthInterval == 0) {
            float offset = m_objLaserOffset.getTSOffset(i, 0, 0, distance,
                                                        header->u8VersionMajor);
            azimuth += m_objLaserOffset.getAngleOffset(
                offset, tail->nMotorSpeed / 10, header->u8VersionMajor);
            point.timestamp +=
                static_cast<double>(
                    m_objLaserOffset.getBlockTS(blockid, tail->nReturnMode, 0,
                                                header->u8LaserNum,
                                                header->u8VersionMajor) +
                    offset) /
                1000000.0;
          }
#endif
          azimuth = (MAX_AZI_LEN + azimuth) % MAX_AZI_LEN;
          float xyDistance =
              distance * m_PandarAT_corrections.cos_map[(elevation)];
          point.x = xyDistance * m_PandarAT_corrections.sin_map[(azimuth)];
          point.y = xyDistance * m_PandarAT_corrections.cos_map[(azimuth)];
          point.z = distance * m_PandarAT_corrections.sin_map[(elevation)];

          point.intensity = u8Intensity;

          point.ring = i + 1;
          int point_index;
          point_index = calculatePointIndex(u16Azimuth, blockid, i, field);
          if (field == m_iField || m_iField == -1) {
            m_OutMsgArray[cursor]->points[point_index] = point;
            if (0 == m_dTimestamp) {
              m_dTimestamp = point.timestamp;
            } else if (m_dTimestamp > point.timestamp) {
              m_dTimestamp = point.timestamp;
            }
          }
          // else{
          // 	pthread_mutex_lock(&m_RedundantPointLock);
          // 	m_RedundantPointBuffer.push_back(RedundantPoint{point_index,
          // point}); 	pthread_mutex_unlock(&m_RedundantPointLock);
          // }
        }
      }
    }
  }
}

void PandarSwiftSDK::loadOffsetFile(std::string file) {
  m_objLaserOffset.setFilePath(file);
}

void PandarSwiftSDK::processGps(PandarGPS *gpsMsg) {
  struct tm t;
  t.tm_sec = gpsMsg->second;
  t.tm_min = gpsMsg->minute;
  t.tm_hour = gpsMsg->hour;
  t.tm_mday = gpsMsg->day;
  t.tm_mon = gpsMsg->month - 1;
  t.tm_year = gpsMsg->year + 2000 - 1900;
  t.tm_isdst = 0;
  if (NULL != m_funcGpsCallback) {
    m_funcGpsCallback(static_cast<double>(mktime(&t) + m_iTimeZoneSecond));
  }
}

void PandarSwiftSDK::processFaultMessage(PandarPacket &packet) {
  if (packet.data[0] != 0xCD || packet.data[1] != 0xDC) return;
  int version = packet.data[2];
  AT128FaultMessageInfo faultMessageInfo;
  switch (version) {
    case 3: {
      AT128FaultMessageVersion3 *faultMessageRawdata =
          (AT128FaultMessageVersion3 *)(&(packet.data[0]));
      faultMessageRawdata->ParserAT128FaultMessage(faultMessageInfo);
      break;
    }
    default:
      break;
  }
  if (NULL != m_funcFaultMessageCallback) {
    m_funcFaultMessageCallback(faultMessageInfo);
  }
}


bool PandarSwiftSDK::isNeedPublish() {
  switch (m_u8UdpVersionMinor) {
    case 3: {
      if (m_PacketsBuffer.hasEnoughPackets()) {
        return false;
      }
      uint32_t beginAzimuth =
          *(uint16_t *)(&(m_PacketsBuffer.getTaskBegin()->data[0]) +
                        m_iFirstAzimuthIndex) *
              LIDAR_AZIMUTH_UNIT +
          *(uint8_t *)(&(m_PacketsBuffer.getTaskBegin()->data[0]) +
                       m_iFirstAzimuthIndex + 1);
      uint32_t endAzimuth = 0;
      if ((m_PacketsBuffer.m_iterPush - m_PacketsBuffer.m_buffers.begin()) > 2)
        endAzimuth =
            *(uint16_t *)(&((m_PacketsBuffer.m_iterPush - 2)->data[0]) +
                          m_iFirstAzimuthIndex) *
                LIDAR_AZIMUTH_UNIT +
            *(uint8_t *)(&((m_PacketsBuffer.m_iterPush - 2)->data[0]) +
                         m_iLastAzimuthIndex + 1);
      else
        endAzimuth = beginAzimuth;
      // printf("%f %f %d %d %d\n",beginAzimuth/25600.0f, endAzimuth/25600.0f,
      // m_PacketsBuffer.m_iterPush - m_PacketsBuffer.m_buffers.begin(),
      // m_PacketsBuffer.getTaskEnd() - m_PacketsBuffer.m_buffers.begin(),
      // m_PacketsBuffer.getTaskBegin() - m_PacketsBuffer.m_buffers.begin());
      if (m_bClockwise) {
        if ((m_bIsSocketTimeout || !m_PacketsBuffer.hasEnoughPackets()) &&
            !m_PacketsBuffer.empty()) {
          for (int i = 0; i < m_PandarAT_corrections.header.frame_number; i++) {
            if ((fabs(float(endAzimuth) -
                      (m_PandarAT_corrections.l.start_frame[i] +
                       (PANDAR_AT128_EDGE_AZIMUTH_OFFSET +
                        PANDAR_AT128_FRAME_ANGLE_SIZE / 2) *
                           LIDAR_AZIMUTH_UNIT) %
                          MAX_AZI_LEN) <=
                 m_iEdgeAzimuthSize * LIDAR_AZIMUTH_UNIT)) {
              return true;
            }
          }
          return false;
        }
        return false;
      }
    } break;
    default:
      return false;
      break;
  }
}

int PandarSwiftSDK::calculatePointIndex(uint16_t u16Azimuth, int blockid,
                                        int laserid, int field) {
  int point_index = 0;
  uint16_t Azimuth = (u16Azimuth) % CIRCLE_ANGLE;
  if (field == 2 && Azimuth < PANDAR_AT128_FRAME_ANGLE_SIZE)
    Azimuth = Azimuth + CIRCLE_ANGLE;
  if (field == 2) Azimuth = Azimuth - CIRCLE_ANGLE / 3;
  if (LIDAR_RETURN_BLOCK_SIZE_2 == m_iReturnBlockSize) {
    point_index =
        int(Azimuth / m_iAngleSize) * m_iLaserNum * m_iReturnBlockSize +
        m_iLaserNum * (blockid % 2) + laserid;
  } else {
    point_index = int(Azimuth / m_iAngleSize) * m_iLaserNum + laserid;
  }
  return point_index;
}
int PandarSwiftSDK::calculatePointBufferSize() {
  return CIRCLE_ANGLE / m_iAngleSize * m_iLaserNum * m_iReturnBlockSize;
}

void PandarSwiftSDK::SetEnvironmentVariableTZ() {
  char *TZ;
  if ((TZ = getenv("TZ"))) {
    printf("TZ=%s\n", TZ);
    return;
  }
  unsigned int timezone = 0;
  time_t t1, t2;
  struct tm *tm_local, *tm_utc;
  time(&t1);
  t2 = t1;
  tm_local = localtime(&t1);
  t1 = mktime(tm_local);
  tm_utc = gmtime(&t2);
  t2 = mktime(tm_utc);
  timezone = 0;
  std::string data = "TZ=UTC" + std::to_string(timezone);
  int len = data.length();
  TZ = (char *)malloc((len + 1) * sizeof(char));
  data.copy(TZ, len, 0);
  if (putenv(TZ) == 0) {
    printf("set environment %s\n", TZ);
  } else {
    printf("set environment fail\n");
  }
}

void PandarSwiftSDK::setIsSocketTimeout(bool isSocketTimeout) {
  m_bIsSocketTimeout = isSocketTimeout;
}

bool PandarSwiftSDK::getIsSocketTimeout() { return m_bIsSocketTimeout; }

bool PandarSwiftSDK::setStandbyLidarMode() {
  int ret = -1;
  ret = TcpCommandSetLidarStandbyMode(m_pTcpCommandClient);
  return ret == 0;
}

bool PandarSwiftSDK::setNormalLidarMode() {
  int ret = -1;
  ret = TcpCommandSetLidarNormalMode(m_pTcpCommandClient);
  return ret == 0;
}

bool PandarSwiftSDK::getLidarReturnMode(uint8_t &mode) {
  int ret = -1;
  unsigned char *buffer = NULL;
  uint32_t len = 0;
  ret = TcpCommandGetLidarConfigInfo(m_pTcpCommandClient, &buffer, &len);
  if (ret == 0) {
    mode = buffer[32];
  }
  return ret == 0;
}

bool PandarSwiftSDK::setLidarReturnMode(uint8_t mode) {
  int ret = -1;
  ret = TcpCommandSetLidarReturnMode(m_pTcpCommandClient, mode);
  return ret == 0;
}

bool PandarSwiftSDK::getLidarSpinRate(uint16_t &spinRate) {
  int ret = -1;
  unsigned char *buffer = NULL;
  uint32_t len = 0;
  ret = TcpCommandGetLidarStatus(m_pTcpCommandClient, &buffer, &len);
  if (ret == 0) {
    spinRate = (uint16_t)((buffer[4]) << 8) + buffer[5];
  }
  return ret == 0;
}

bool PandarSwiftSDK::setLidarSpinRate(uint16_t spinRate) {
  int ret = -1;
  ret = TcpCommandSetLidarSpinRate(m_pTcpCommandClient, spinRate);
  return ret == 0;
}

bool PandarSwiftSDK::getLidarLensHeatSwitch(uint8_t &heatSwitch) {
  int ret = -1;
  unsigned char *buffer = NULL;
  uint32_t len = 0;
  ret = TcpCommandGetLidarLensHeatSwitch(m_pTcpCommandClient, &buffer, &len);
  if (ret == 0) {
    heatSwitch = buffer[0];
  }
  return ret == 0;
}

bool PandarSwiftSDK::setLidarLensHeatSwitch(uint8_t heatSwitch) {
  int ret = -1;
  ret = TcpCommandSetLidarLensHeatSwitch(m_pTcpCommandClient, heatSwitch);
  return ret == 0;
}

bool PandarSwiftSDK::GetIsReadPcapOver() { return m_bIsReadPcapOver; }
void PandarSwiftSDK::SetIsReadPcapOver(bool enable) {
  m_bIsReadPcapOver = enable;
}

void PandarSwiftSDK::setTimeStampNum(int num) {
  if (num <= 0) {
    m_dAzimuthInterval = 0;
    return;
  }
  m_dAzimuthInterval = double(double(PANDAR_AT128_FRAME_ANGLE_SIZE) / num);
}

bool PandarSwiftSDK::setPtcsLidarMode() {
  TcpCommandSetNonSsl();
  std::ifstream fin;
  fin.open(m_sPtcsModeSetFilePath, std::ios::binary);
  int ret = -1;
  unsigned char *buff = NULL;
  if (fin.is_open()) {
    uint32_t length = 0;
    fin.seekg(0, std::ios::end);
    length = fin.tellg();
    fin.seekg(0, std::ios::beg);
    char *data = new char[length];
    fin.read(data, length);
    fin.close();
    buff = reinterpret_cast<unsigned char *>(data);
    ret = TcpCommandSetPtcsLidarMode(m_pTcpCommandClient, buff, length);
    if (ret != 0) {
      printf("Set ptcs lidar mode fail!!!\n");
    } else {
      TcpCommandSetSsl(m_sCertFilePath.c_str(), m_sPrivateKeyFilePath.c_str(),
                       m_sCaFilePath.c_str());
      printf("Set ptcs lidar mode success!!!\n");
      return true;
    }
  } else {
    printf("Open ptcs mode set file failed\n");
    return false;
  }
  return false;
}

bool PandarSwiftSDK::setPtcLidarMode() {
  TcpCommandSetNonSsl();
  TcpCommandSetSsl(m_sCertFilePath.c_str(), m_sPrivateKeyFilePath.c_str(),
                   m_sCaFilePath.c_str());
  int ret = -1;
  ret = TcpCommandSetPtcLidarMode(m_pTcpCommandClient);
  if (ret != 0) {
    printf("Set ptc lidar mode fail!!!\n");
  } else {
    TcpCommandSetNonSsl();
    printf("Set ptc lidar mode success!!!\n");
  }
  return ret == 0;
}

int PandarSwiftSDK::getPtcsLidarMode() {
  return TcpCommandGetPtcsLidarMode(m_pTcpCommandClient);
}

float PandarSwiftSDK::getTxTemperature() { return m_fTxTemperature; }

float PandarSwiftSDK::getRxTemperature() { return m_fRxTemperature; }

float PandarSwiftSDK::getPbTemperature() { return m_fPbTemperature; }

float PandarSwiftSDK::getMbTemperature() { return m_fMbTemperature; }

void PandarSwiftSDK::updateMoniterInfo(int id, uint16_t data) {
  switch (id) {
    case TX_TDM_ID:
      m_fTxTemperature = data * 0.01f;
      break;
    case RX_TDM_ID:
      m_fRxTemperature = data * 0.01f;
      break;
    case MB_TDM_ID:
      m_fMbTemperature = data * 0.01f;
      break;
    case PB_TDM_ID:
      m_fPbTemperature = data * 0.01f;
      break;
    default:
      break;
  }
}
