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
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sys/syscall.h>
#include <time.h>
#include <unistd.h>
#include <fstream>
#include <iostream>
#include "taskflow.hpp"
#include "platUtil.h"

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

/** @brief Constructor. */
PandarSwiftSDK::PandarSwiftSDK(std::string deviceipaddr, uint16_t lidarport, uint16_t gpsport, std::string frameid, std::string correctionfile, std::string firtimeflie, std::string pcapfile, \
							boost::function<void(boost::shared_ptr<PPointCloud>, double)> pclcallback, \
							boost::function<void(PandarPacketsArray*)> rawcallback, \
							boost::function<void(double)> gpscallback, \
							std::string certFile, std::string privateKeyFile, std::string caFile, \
							int startangle, int timezone, std::string publishmode, std::string datatype) {
	m_sSdkVersion = "PandarSwiftSDK_1.2.9";
	printf("\n--------PandarSwift SDK version: %s--------\n",m_sSdkVersion.c_str());
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
	m_funcPclCallback = pclcallback;
	m_funcGpsCallback = gpscallback;
	m_spPandarDriver.reset(new PandarSwiftDriver(deviceipaddr, lidarport, gpsport, frameid, pcapfile, rawcallback, this, publishmode, datatype));
	TcpCommandSetSsl(certFile.c_str(), privateKeyFile.c_str(), caFile.c_str());
	printf("frame id: %s\n", m_sFrameId.c_str());
	printf("lidar firetime file: %s\n", m_sLidarFiretimeFile.c_str());
	printf("lidar correction file: %s\n", m_sLidarCorrectionFile.c_str());
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
	if(LIDAR_DATA_TYPE == datatype) {
		boost::thread thrd(boost::bind(&PandarSwiftSDK::driverReadThread, this));
	}
	if(m_sPublishmodel == "both_point_raw" || m_sPublishmodel == "point" || LIDAR_DATA_TYPE != datatype) {
		boost::thread processThr(boost::bind(&PandarSwiftSDK::processLiDARData, this));
		boost::thread publishPointsThr(boost::bind(&PandarSwiftSDK::publishPointsThread, this));
	}
	if((m_sPublishmodel == "both_point_raw" || m_sPublishmodel == "raw") && LIDAR_DATA_TYPE == datatype) {
		boost::thread processThr(boost::bind(&PandarSwiftSDK::publishRawDataThread, this));
	}
}

void PandarSwiftSDK::loadCorrectionFile() {
	bool loadCorrectionFileSuccess = false;
	int ret;
	if(m_sPcapFile.empty()) { //connect to lidar,load correction file frome lidar
		m_pTcpCommandClient =TcpCommandClientNew(m_sDeviceIpAddr.c_str(), PANDARSDK_TCP_COMMAND_PORT);
		if(NULL != m_pTcpCommandClient) {
			char *buffer = NULL;
			uint32_t len = 0;
			std::string correntionString;
			ret = TcpCommandGetLidarCalibration(m_pTcpCommandClient, &buffer, &len);
			if(ret == 0 && buffer) {
				printf("Load correction file from lidar now!\n");
				correntionString = std::string(buffer);
				ret = loadCorrectionString(correntionString);
				if(ret != 0) {
					printf("Parse Lidar Correction Error\n");
				} 
				else {
					loadCorrectionFileSuccess = true;
					printf("Parse Lidar Correction Success!!!\n");
				}
				free(buffer);
			}
			else{
				printf("Get lidar calibration filed\n");
			}
		}
	}
	if(!loadCorrectionFileSuccess) { //load correction file from locaf file
		printf("load correction file from local correction.csv now!\n");
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
			ret = loadCorrectionString(strlidarCalibration);
			if(ret != 0) {
				printf("Parse local Correction file Error\n");
			} 
			else {
				printf("Parse local Correction file Success!!!\n");
			}
		}
		else
		{
			printf("Open correction file failed\n");
			return;
		}
	}
}

int PandarSwiftSDK::loadCorrectionString(std::string correction_content) {
    std::istringstream ifs(correction_content);
	std::string line;
	if(std::getline(ifs, line)) {  // first line "Laser id,Elevation,Azimuth"
		printf("Parse Lidar Correction...\n");
	}
	float pitchList[PANDAR128_LASER_NUM];
	float azimuthList[PANDAR128_LASER_NUM];
	int lineCounter = 0;
	while (std::getline(ifs, line)) {
		if(line.length() < strlen("1,1,1")) {
			return -1;
		} 
		else {
			lineCounter++;
		}
		float elev, azimuth;
		int lineId = 0;
		std::stringstream ss(line);
		std::string subline;
		std::getline(ss, subline, ',');
		std::stringstream(subline) >> lineId;
		std::getline(ss, subline, ',');
		std::stringstream(subline) >> elev;
		std::getline(ss, subline, ',');
		std::stringstream(subline) >> azimuth;
		if(lineId != lineCounter) {
			printf("laser id error %d %d\n", lineId, lineCounter);
			return -1;
		}
		pitchList[lineId - 1] = elev;
		azimuthList[lineId - 1] = azimuth;
	}
	for (int i = 0; i < lineCounter; ++i) {
		m_fElevAngle[i] = pitchList[i];
		m_fHorizatalAzimuth[i] = azimuthList[i];
	}
	return 0;
}

void PandarSwiftSDK::driverReadThread() {
	SetThreadPriority(SCHED_RR, 99);
	while (1) {
		m_spPandarDriver->poll();
	}
}

void PandarSwiftSDK::publishRawDataThread() {
	SetThreadPriority(SCHED_FIFO, 90);
	while (1) {
		m_spPandarDriver->publishRawData();
	}
}

void PandarSwiftSDK::pushLiDARData(PandarPacket packet) {
	//  printf("PandarSwiftSDK::pushLiDARData");
	m_PacketsBuffer.push_back(packet);
	// printf("%d, %d\n",pkt.blocks[0].fAzimuth,pkt.blocks[1].fAzimuth);
}

int PandarSwiftSDK::parseData(Pandar128PacketVersion13 &packet, const uint8_t *recvbuf, const int len) {
	int index = 0;
	if(recvbuf[0] != 0xEE && recvbuf[1] != 0xFF && recvbuf[2] != 1 ) {    
		printf("Lidar type is error %d\n", len);
		return -1;
	}
	if(3 == recvbuf[3]) { //udp version :1.3
		memcpy(&packet, recvbuf, sizeof(Pandar128PacketVersion13));
		return 0;
	}
	if(4 == recvbuf[3]) { //udp version 1.4
		memcpy(&(packet.head), &recvbuf[index], PANDAR128_HEAD_SIZE);
	
		index += PANDAR128_HEAD_SIZE;
		memcpy(&(packet.blocks), &recvbuf[index], PANDAR128_BLOCK_SIZE * PANDAR128_BLOCK_NUM);
		
		index += PANDAR128_BLOCK_SIZE * PANDAR128_BLOCK_NUM + PANDAR128_CRC_SIZE + PANDAR128_FUNCTION_SAFETY_SIZE;
		memcpy(&(packet.tail.nReserved1[0]),  &recvbuf[index], 3);

		index += 3;
		memcpy(&(packet.tail.nReserved2[0]),  &recvbuf[index], 3);

		index += 3 + 3 + 2;
		packet.tail.nShutdownFlag = (recvbuf[index] & 0x0F) | (recvbuf[index + 1] & 0xC0)  | (recvbuf[index+ 1] & 0x30);

		index += 1;
		packet.tail.nReturnMode = recvbuf[index];

		index += 1;
		packet.tail.nMotorSpeed = (recvbuf[index] & 0xff) | ((recvbuf[index + 1] & 0xff) << 8);

		index += 2;
		memcpy(&(packet.tail.nUTCTime[0]),  &recvbuf[index], 6);

		index += 6;
		memcpy(&(packet.tail.nTimestamp),  &recvbuf[index], 4);

		index += 4 + 1;
		memcpy(&(packet.tail.nSeqNum),  &recvbuf[index], 4);
		return 0;
	}
	return -1;
}

int PandarSwiftSDK::processLiDARData() {
	SetThreadPriority(SCHED_FIFO, 91);
	double lastTimestamp = 0.0;
	struct timespec ts;
	int ret = 0;
	int cursor = 0;
	uint32_t startTick = GetTickCount();
	uint32_t endTick;
	init();
	while (1) {
		if(!m_PacketsBuffer.hasEnoughPackets()) {
			// printf("dont have enough packet\n");
			usleep(1000);
			continue;
		}
		
		if(0 == checkLiadaMode()) {
			// printf("checkLiadaMode now!!");
			m_OutMsgArray[cursor]->clear();
			m_OutMsgArray[cursor]->resize(CIRCLE_ANGLE / m_iAngleSize * m_iLaserNum * m_iReturnBlockSize);
			m_PacketsBuffer.creatNewTask();
			continue;
		}

		// printf("begin: %d, end: %d\n",m_PacketsBuffer.getTaskBegin()->blocks[0].fAzimuth, (m_PacketsBuffer.getTaskEnd() - 1)->blocks[1].fAzimuth);
		uint32_t ifstart = GetTickCount();
		if((*(uint16_t*)(&(m_PacketsBuffer.getTaskBegin()->data[0]) + m_iFirstAzimuthIndex) > *(uint16_t*)(&((m_PacketsBuffer.getTaskEnd() - 1)->data[0]) + m_iLastAzimuthIndex)) && 
				(m_iLidarRotationStartAngle <= *(uint16_t*)(&((m_PacketsBuffer.getTaskEnd() - 1)->data[0]) + m_iLastAzimuthIndex)) ||
				((*(uint16_t*)(&(m_PacketsBuffer.getTaskBegin()->data[0]) + m_iFirstAzimuthIndex) < m_iLidarRotationStartAngle) && 
				(m_iLidarRotationStartAngle <= *(uint16_t*)(&((m_PacketsBuffer.getTaskEnd() - 1)->data[0]) + m_iLastAzimuthIndex)) ||
				(CIRCLE_ANGLE - *(uint16_t*)(&((m_PacketsBuffer.getTaskEnd() - 1)->data[0]) + m_iLastAzimuthIndex) - m_iLidarRotationStartAngle) <= m_iAngleSize)) {
			uint32_t startTick1 = GetTickCount();
			moveTaskEndToStartAngle();
			doTaskFlow(cursor);
			uint32_t startTick2 = GetTickCount();
			// printf("move and taskflow time:%d\n", startTick2 - startTick1);
			if(m_bPublishPointsFlag == false) {
				m_bPublishPointsFlag = true;
				m_iPublishPointsIndex = cursor;
				cursor = (cursor + 1) % 2;
			} 
			else
				printf("publishPoints not done yet, new publish is comming\n");
			m_OutMsgArray[cursor]->clear();
			m_OutMsgArray[cursor]->resize(CIRCLE_ANGLE / m_iAngleSize * m_iLaserNum * m_iReturnBlockSize );
			if(m_RedundantPointBuffer.size() > 0 && m_RedundantPointBuffer.size() < 1000){
				for(int i = 0; i < m_RedundantPointBuffer.size(); i++){
				m_OutMsgArray[cursor]->points[m_RedundantPointBuffer[i].index] = m_RedundantPointBuffer[i].point;
				}
			}
			m_RedundantPointBuffer.clear();
			uint32_t endTick2 = GetTickCount();
			if(endTick2 - startTick2 > 2) {
				// printf("m_OutMsgArray time:%d\n", endTick2 - startTick2);
			}
			m_OutMsgArray[cursor]->header.frame_id = m_sFrameId;
			m_OutMsgArray[cursor]->height = 1;
			endTick = GetTickCount();
			// printf("total time: %d\n", endTick - startTick);
			startTick = endTick;
			continue;
		}
		uint32_t taskflow1 = GetTickCount();
			// printf("if compare time: %d\n", ifTick - startTick);
		doTaskFlow(cursor);
		uint32_t taskflow2 = GetTickCount();
			// printf("taskflow time: %d\n", taskflow2 - taskflow1);

	}
}

void PandarSwiftSDK::moveTaskEndToStartAngle() {
	uint32_t startTick = GetTickCount();
	for(PktArray::iterator iter = m_PacketsBuffer.m_iterTaskBegin; iter < m_PacketsBuffer.m_iterTaskEnd; iter++) {
		if ((*(uint16_t*)(&(iter->data[0]) + m_iFirstAzimuthIndex) > *(uint16_t*)(&((iter + 1)->data[0]) + m_iFirstAzimuthIndex)) &&
				(m_iLidarRotationStartAngle <= *(uint16_t*)(&((iter + 1)->data[0]) + m_iFirstAzimuthIndex)) ||
				((*(uint16_t*)(&(iter->data[0]) + m_iFirstAzimuthIndex) < m_iLidarRotationStartAngle) &&
            	(m_iLidarRotationStartAngle <= *(uint16_t*)(&((iter + 1)->data[0]) + m_iFirstAzimuthIndex)))) {
			// printf("move iter to : %d\n", (iter + 1)->blocks[0].fAzimuth);
			m_PacketsBuffer.moveTaskEnd(iter + 1);
			break;
		}
	}
	uint32_t endTick = GetTickCount();
	// printf("moveTaskEndToStartAngle time: %d\n", endTick - startTick);
}

void PandarSwiftSDK::publishPointsThread() {
	SetThreadPriority(SCHED_FIFO, 90);
	while (1) {
		usleep(1000);
		if(m_bPublishPointsFlag) {
			uint32_t start = GetTickCount();
			if(NULL != m_funcPclCallback) {
				m_funcPclCallback(m_OutMsgArray[m_iPublishPointsIndex], m_dTimestamp);
				m_dTimestamp = 0;
				m_bPublishPointsFlag = false;
			}
			uint32_t end = GetTickCount();
  			if(end - start > 150) printf("publishPoints time:%d\n", end - start);
		}
	}
}

void PandarSwiftSDK::doTaskFlow(int cursor) {
	tf::Taskflow taskFlow;
	taskFlow.parallel_for(m_PacketsBuffer.getTaskBegin(),m_PacketsBuffer.getTaskEnd(),
							[this, cursor](auto &taskpkt) {calcPointXYZIT(taskpkt, cursor);});
	executor.run(taskFlow).wait();
	m_PacketsBuffer.creatNewTask();
}

void PandarSwiftSDK::init() {
	while (1) {
		if(!m_PacketsBuffer.hasEnoughPackets()) {
			usleep(1000);
			continue;
		}
		uint16_t lidarmotorspeed = 0;
		if((m_PacketsBuffer.getTaskEnd() - 1)->data[3] == 3){
			Pandar128PacketVersion13 packet;
			memcpy(&packet, &((m_PacketsBuffer.getTaskEnd() - 1)->data[0]), sizeof(Pandar128PacketVersion13));
			m_iWorkMode = packet.tail.nShutdownFlag & 0x03;
			m_iReturnMode = packet.tail.nReturnMode;
			m_u8UdpVersionMajor = packet.head.u8VersionMajor;
			m_u8UdpVersionMinor = packet.head.u8VersionMinor;
			m_spPandarDriver->setUdpVersion(m_u8UdpVersionMajor, m_u8UdpVersionMinor);
			lidarmotorspeed = packet.tail.nMotorSpeed;
			m_iLaserNum = packet.head.u8LaserNum;
			m_iFirstAzimuthIndex = PANDAR128_HEAD_SIZE;
			m_iLastAzimuthIndex = PANDAR128_HEAD_SIZE + PANDAR128_BLOCK_SIZE;
		}
		else if((m_PacketsBuffer.getTaskEnd() - 1)->data[3] == 4){
			auto header = (Pandar128HeadVersion14*)(&((m_PacketsBuffer.getTaskEnd() - 1)->data[0]));
			auto tail = (Pandar128TailVersion14*)(&((m_PacketsBuffer.getTaskEnd() - 1)->data[0]) + PANDAR128_HEAD_SIZE +
						(header->hasConfidence() ? PANDAR128_UNIT_WITH_CONFIDENCE_SIZE * header->u8LaserNum * header->u8BlockNum : PANDAR128_UNIT_WITHOUT_CONFIDENCE_SIZE * header->u8LaserNum * header->u8BlockNum) + 
						PANDAR128_AZIMUTH_SIZE * header->u8BlockNum + 
						PANDAR128_CRC_SIZE + 
						(header->hasFunctionSafety()? PANDAR128_FUNCTION_SAFETY_SIZE : 0) - 1);
			m_iWorkMode = tail->nShutdownFlag & 0x03;
			m_iReturnMode = tail->nReturnMode;
			m_u8UdpVersionMajor = header->u8VersionMajor;
			m_u8UdpVersionMinor = header->u8VersionMinor;
			m_spPandarDriver->setUdpVersion(m_u8UdpVersionMajor, m_u8UdpVersionMinor);
			lidarmotorspeed = tail->nMotorSpeed;
			m_iLaserNum = header->u8LaserNum;
			m_iFirstAzimuthIndex = PANDAR128_HEAD_SIZE;
			m_iLastAzimuthIndex = PANDAR128_HEAD_SIZE + 
									(header->hasConfidence() ? PANDAR128_UNIT_WITH_CONFIDENCE_SIZE * header->u8LaserNum * (header->u8BlockNum - 1) : PANDAR128_UNIT_WITHOUT_CONFIDENCE_SIZE * header->u8LaserNum * (header->u8BlockNum - 1)) + 
									PANDAR128_AZIMUTH_SIZE * (header->u8BlockNum - 1);
		}
		if(abs(lidarmotorspeed - MOTOR_SPEED_600) < 100) { //ignore the speed gap of 6000 rpm
			lidarmotorspeed = MOTOR_SPEED_600;
		}
		else if(abs(lidarmotorspeed - MOTOR_SPEED_1200) < 100) { //ignore the speed gap of 1200 rpm
			lidarmotorspeed = MOTOR_SPEED_1200;
		}
		else {
			lidarmotorspeed = MOTOR_SPEED_600; //changing the speed,give enough size
		}
		m_iMotorSpeed = lidarmotorspeed;
		printf("init mode: workermode: %x,return mode: %x,speed: %d\n",m_iWorkMode, m_iReturnMode, m_iMotorSpeed);
		printf("UDP version %d.%d \n",m_u8UdpVersionMajor, m_u8UdpVersionMinor);
		changeAngleSize();
		changeReturnBlockSize();
		boost::shared_ptr<PPointCloud> outMag0(new PPointCloud(CIRCLE_ANGLE / m_iAngleSize * m_iLaserNum * m_iReturnBlockSize, 1));
		boost::shared_ptr<PPointCloud> outMag1(new PPointCloud(CIRCLE_ANGLE / m_iAngleSize * m_iLaserNum * m_iReturnBlockSize, 1));
		m_OutMsgArray[0] = outMag0;
		m_OutMsgArray[1] = outMag1;
		break;
	}
}

int PandarSwiftSDK::checkLiadaMode() {
	uint8_t lidarworkmode = 0;
	uint8_t lidarreturnmode = 0;
	uint16_t lidarmotorspeed = 0;
	uint8_t laserNum = 0;
	if((m_PacketsBuffer.getTaskEnd() - 1)->data[3] == 3){
		Pandar128PacketVersion13 packet;
		memcpy(&packet, &((m_PacketsBuffer.getTaskEnd() - 1)->data[0]), sizeof(Pandar128PacketVersion13));
		lidarworkmode = packet.tail.nShutdownFlag & 0x03;
		lidarreturnmode = packet.tail.nReturnMode;
		lidarmotorspeed = packet.tail.nMotorSpeed;
		laserNum = packet.head.u8LaserNum;
	}
	else if((m_PacketsBuffer.getTaskEnd() - 1)->data[3] == 4){
		auto header = (Pandar128HeadVersion14*)(&((m_PacketsBuffer.getTaskEnd() - 1)->data[0]));
		auto tail = (Pandar128TailVersion14*)(&((m_PacketsBuffer.getTaskEnd() - 1)->data[0]) + PANDAR128_HEAD_SIZE +
					(header->hasConfidence() ? PANDAR128_UNIT_WITH_CONFIDENCE_SIZE * header->u8LaserNum * header->u8BlockNum : PANDAR128_UNIT_WITHOUT_CONFIDENCE_SIZE * header->u8LaserNum * header->u8BlockNum) + 
					PANDAR128_AZIMUTH_SIZE * header->u8BlockNum + 
					PANDAR128_CRC_SIZE + 
					(header->hasFunctionSafety()? PANDAR128_FUNCTION_SAFETY_SIZE : 0) - 1);
		lidarworkmode = tail->nShutdownFlag & 0x03;
		lidarreturnmode = tail->nReturnMode;
		lidarmotorspeed = tail->nMotorSpeed;
		laserNum = header->u8LaserNum;
	}
	if(abs(lidarmotorspeed - MOTOR_SPEED_600) < 100) { //ignore the speed gap of 6000 rpm
		lidarmotorspeed = MOTOR_SPEED_600;
	}
	else if(abs(lidarmotorspeed - MOTOR_SPEED_1200) < 100) { //ignore the speed gap of 1200 rpm
		lidarmotorspeed = MOTOR_SPEED_1200;
	}
	else {
		lidarmotorspeed = MOTOR_SPEED_600; //changing the speed,give enough size
	}
    //mode change 
	if (0 == m_iWorkMode && 0 == m_iReturnMode && 0 == m_iMotorSpeed && 0 == m_iLaserNum) { //init lidar mode 
		m_iWorkMode = lidarworkmode;
		m_iReturnMode = lidarreturnmode;
		m_iMotorSpeed = lidarmotorspeed;
		m_iLaserNum = laserNum;
		printf("init mode: workermode: %x,return mode: %x,speed: %d,laser number: %d",m_iWorkMode, m_iReturnMode, m_iMotorSpeed, m_iLaserNum);
		changeAngleSize();
		changeReturnBlockSize();
		boost::shared_ptr<PPointCloud> outMag0(new PPointCloud(
			CIRCLE_ANGLE * 100 / m_iAngleSize * m_iLaserNum * m_iReturnBlockSize, 1));
		boost::shared_ptr<PPointCloud> outMag1(new PPointCloud(
			CIRCLE_ANGLE * 100 / m_iAngleSize * m_iLaserNum * m_iReturnBlockSize, 1));
		m_OutMsgArray[0] = outMag0;
		m_OutMsgArray[1] = outMag1;
		return 1;
	} 
	else{
		if(m_iWorkMode != lidarworkmode) { //work mode change
			printf("change work mode:  %x to %x\n",m_iWorkMode, lidarworkmode);
			m_iWorkMode = lidarworkmode;
			m_iMotorSpeed = lidarmotorspeed;
			changeAngleSize();
			return 0;
		}
		if(m_iReturnMode != lidarreturnmode) { //return mode change
			printf("change return mode:  %x to %x\n",m_iReturnMode, lidarreturnmode);
			m_iReturnMode = lidarreturnmode;
			changeReturnBlockSize();
			return 0;
		}
		if(m_iMotorSpeed != lidarmotorspeed) { //motor speed change
			// printf("change motor speed:  %d to %d\n",m_iMotorSpeed, lidarmotorspeed);
			m_iMotorSpeed = lidarmotorspeed;
			changeAngleSize();
			return 0;
		}
		if (m_iLaserNum != laserNum) { //laser number change
			m_iLaserNum = laserNum;
			return 0;
		}
		return 1;
	}
}

void PandarSwiftSDK::changeAngleSize() {
	if(m_iLaserNum == 80) {
		m_iAngleSize = LIDAR_ANGLE_SIZE_18;  // 18->0.18degree
		return;
	} 
	if(0 == m_iWorkMode && MOTOR_SPEED_600 == m_iMotorSpeed) {
		m_iAngleSize = LIDAR_ANGLE_SIZE_10 * m_iMotorSpeed / MOTOR_SPEED_600;  // 20->0.1degree
	}
	if(0 == m_iWorkMode && MOTOR_SPEED_1200 == m_iMotorSpeed) {
		m_iAngleSize = LIDAR_ANGLE_SIZE_20;  // 20->0.2degreepktCount[2]
	}
	if(0 != m_iWorkMode && MOTOR_SPEED_600 == m_iMotorSpeed) {
		m_iAngleSize = LIDAR_ANGLE_SIZE_20;  // 20->0.2degree
	}
	if(0 != m_iWorkMode && MOTOR_SPEED_1200 == m_iMotorSpeed) {
		m_iAngleSize = LIDAR_ANGLE_SIZE_40;  // 40->0.4degree
	}  
}

void PandarSwiftSDK::changeReturnBlockSize() {
	if(0x39 == m_iReturnMode || 0x3b == m_iReturnMode || 0x3c == m_iReturnMode) {
		m_iReturnBlockSize = LIDAR_RETURN_BLOCK_SIZE_2;
	} else {
		m_iReturnBlockSize = LIDAR_RETURN_BLOCK_SIZE_1;
	}
}

void PandarSwiftSDK::calcPointXYZIT(PandarPacket &pkt, int cursor) {
	if (pkt.data[3] == 3){
		Pandar128PacketVersion13 packet;
		memcpy(&packet, &pkt.data[0], sizeof(Pandar128PacketVersion13));
		struct tm t = {0};
		t.tm_year = packet.tail.nUTCTime[0];
		t.tm_mon = packet.tail.nUTCTime[1] - 1;
		t.tm_mday = packet.tail.nUTCTime[2];
		t.tm_hour = packet.tail.nUTCTime[3];
		t.tm_min = packet.tail.nUTCTime[4];
		t.tm_sec = packet.tail.nUTCTime[5];
		t.tm_isdst = 0;
		double unix_second = static_cast<double>(mktime(&t) + m_iTimeZoneSecond);
		for (int blockid = 0; blockid < packet.head.u8BlockNum; blockid++) {
			Pandar128Block &block = packet.blocks[blockid];
			int mode = packet.tail.nShutdownFlag & 0x03;
			int state = 0;
			if(0 == blockid)
				state = (packet.tail.nShutdownFlag & 0xC0) >> 6;
			if(1 == blockid)
				state = (packet.tail.nShutdownFlag & 0x30) >> 4;
			for (int i = 0; i < packet.head.u8LaserNum; i++) {
				/* for all the units in a block */
				Pandar128Unit &unit = block.units[i];
				PPoint point;
				float distance =static_cast<float>(unit.u16Distance) * PANDAR128_DISTANCE_UNIT;
				/* filter distance */
				// if(distance < 0.1) {
				// 	continue;
				// }
				float azimuth = m_fHorizatalAzimuth[i] + (block.fAzimuth / 100.0f);
				float originAzimuth = azimuth;
				float pitch = m_fElevAngle[i];
				float originPitch = pitch;
				int offset = m_objLaserOffset.getTSOffset(i, mode, state, distance, packet.head.u8LaserNum);
				azimuth += m_objLaserOffset.getAngleOffset(offset, i, packet.head.u8LaserNum);
				pitch += m_objLaserOffset.getPitchOffset(m_sFrameId, pitch, distance);
				if(pitch < 0) {
					pitch += 360.0f;
				} 
				else if(pitch >= 360.0f) {
					pitch -= 360.0f;
				}
				float xyDistance = distance * m_fCosAllAngle[static_cast<int>(pitch * 100 + 0.5)];
				azimuth += m_objLaserOffset.getAzimuthOffset(m_sFrameId, originAzimuth, block.fAzimuth / 100.0f, xyDistance);
				int azimuthIdx = static_cast<int>(azimuth * 100 + 0.5);
				if(azimuthIdx >= CIRCLE) {
					azimuthIdx -= CIRCLE;
				} 
				else if(azimuthIdx < 0) {
					azimuthIdx += CIRCLE;
				}
				point.x = xyDistance * m_fSinAllAngle[azimuthIdx];
				point.y = xyDistance * m_fCosAllAngle[azimuthIdx];
				point.z = distance * m_fSinAllAngle[static_cast<int>(pitch * 100 + 0.5)];
				point.intensity = unit.u8Intensity;
				point.timestamp = unix_second + (static_cast<double>(packet.tail.nTimestamp)) / 1000000.0;
				point.timestamp = point.timestamp + m_objLaserOffset.getBlockTS(blockid, packet.tail.nReturnMode, mode, packet.head.u8LaserNum) / 1000000000.0 + offset / 1000000000.0;
				if(0 == m_dTimestamp) {
					m_dTimestamp = point.timestamp;
				}
				else if(m_dTimestamp > point.timestamp) {
					m_dTimestamp = point.timestamp;
				}
				point.ring = i + 1;
				int point_index;
				if(LIDAR_RETURN_BLOCK_SIZE_2 == m_iReturnBlockSize) {
					point_index = (block.fAzimuth - m_iLidarRotationStartAngle) / m_iAngleSize * m_iLaserNum * m_iReturnBlockSize + m_iLaserNum * blockid + i;
					// printf("block 2 index:[%d]",index);
				} 
				else {
					point_index = (block.fAzimuth - m_iLidarRotationStartAngle) / m_iAngleSize * m_iLaserNum + i;
				}
				if(m_OutMsgArray[cursor]->points[point_index].ring == 0){
					m_OutMsgArray[cursor]->points[point_index] = point;
				}
				else{
					pthread_mutex_lock(&m_RedundantPointLock);
					m_RedundantPointBuffer.push_back(RedundantPoint{point_index, point});
					pthread_mutex_unlock(&m_RedundantPointLock);
				}
			}
		}
	}
	else if(pkt.data[3] == 4){
		auto header = (Pandar128HeadVersion14*)(&pkt.data[0]);
		auto tail = (Pandar128TailVersion14*)(&pkt.data[0] + PANDAR128_HEAD_SIZE + 
					(header->hasConfidence() ? PANDAR128_UNIT_WITH_CONFIDENCE_SIZE * header->u8LaserNum * header->u8BlockNum : PANDAR128_UNIT_WITHOUT_CONFIDENCE_SIZE * header->u8LaserNum * header->u8BlockNum) + 
					PANDAR128_AZIMUTH_SIZE * header->u8BlockNum + 
					PANDAR128_CRC_SIZE + 
					(header->hasFunctionSafety()? PANDAR128_FUNCTION_SAFETY_SIZE : 0) - 1);
		struct tm t = {0};
		t.tm_year = tail->nUTCTime[0];
		t.tm_mon = tail->nUTCTime[1] - 1;
		t.tm_mday = tail->nUTCTime[2];
		t.tm_hour = tail->nUTCTime[3];
		t.tm_min = tail->nUTCTime[4];
		t.tm_sec = tail->nUTCTime[5];
		t.tm_isdst = 0;
		double unix_second = static_cast<double>(mktime(&t) + m_iTimeZoneSecond);
		int index = 0;
		index += PANDAR128_HEAD_SIZE;
		for (int blockid = 0; blockid < header->u8BlockNum; blockid++) {
			uint16_t u16Azimuth = *(uint16_t*)(&pkt.data[0] + index);
			index += PANDAR128_AZIMUTH_SIZE;
			int mode = tail->nShutdownFlag & 0x03;
			int state = 0;
			if(0 == blockid)
				state = (tail->nShutdownFlag & 0xC0) >> 6;
			if(1 == blockid)
				state = (tail->nShutdownFlag & 0x30) >> 4;
			for (int i = 0; i < header->u8LaserNum; i++) {
				/* for all the units in a block */
				uint16_t u16Distance = *(uint16_t*)(&pkt.data[0] + index);
				index += DISTANCE_SIZE;
				uint8_t u8Intensity = *(uint8_t*)(&pkt.data[0] + index);
				index += INTENSITY_SIZE;
				index += header->hasConfidence() ? CONFIDENCE_SIZE : 0;
				PPoint point;
				float distance =static_cast<float>(u16Distance) * PANDAR128_DISTANCE_UNIT;
				/* filter distance */
				// if(distance < 0.1) {
				// 	continue;
				// }
				float azimuth = m_fHorizatalAzimuth[i] + (u16Azimuth / 100.0f);
				float originAzimuth = azimuth;
				float pitch = m_fElevAngle[i];
				float originPitch = pitch;
				int offset = m_objLaserOffset.getTSOffset(i, mode, state, distance, header->u8LaserNum);
				azimuth += m_objLaserOffset.getAngleOffset(offset, i, header->u8LaserNum);
				pitch += m_objLaserOffset.getPitchOffset(m_sFrameId, pitch, distance);
				if(pitch < 0) {
					pitch += 360.0f;
				} 
				else if(pitch >= 360.0f) {
					pitch -= 360.0f;
				}
				float xyDistance = distance * m_fCosAllAngle[static_cast<int>(pitch * 100 + 0.5)];
				azimuth += m_objLaserOffset.getAzimuthOffset(m_sFrameId, originAzimuth, u16Azimuth / 100.0f, xyDistance);
				int azimuthIdx = static_cast<int>(azimuth * 100 + 0.5);
				if(azimuthIdx >= CIRCLE) {
					azimuthIdx -= CIRCLE;
				} 
				else if(azimuthIdx < 0) {
					azimuthIdx += CIRCLE;
				}
				point.x = xyDistance * m_fSinAllAngle[azimuthIdx];
				point.y = xyDistance * m_fCosAllAngle[azimuthIdx];
				point.z = distance * m_fSinAllAngle[static_cast<int>(pitch * 100 + 0.5)];
				point.intensity = u8Intensity;
				point.timestamp = unix_second + (static_cast<double>(tail->nTimestamp)) / 1000000.0;
				point.timestamp = point.timestamp + m_objLaserOffset.getBlockTS(blockid, tail->nReturnMode, mode, header->u8LaserNum) / 1000000000.0 + offset / 1000000000.0;
				if(0 == m_dTimestamp) {
					m_dTimestamp = point.timestamp;
				}
				else if(m_dTimestamp > point.timestamp) {
					m_dTimestamp = point.timestamp;
				}
				point.ring = i + 1;
				int point_index;
				if(LIDAR_RETURN_BLOCK_SIZE_2 == m_iReturnBlockSize) {
					point_index = (u16Azimuth - m_iLidarRotationStartAngle) / m_iAngleSize * m_iLaserNum * m_iReturnBlockSize + m_iLaserNum * (blockid % 2) + i;
					// printf("block 2 index:[%d]",index);
				} 
				else {
					point_index = (u16Azimuth - m_iLidarRotationStartAngle) / m_iAngleSize * m_iLaserNum + i;
				}
				if(m_OutMsgArray[cursor]->points[point_index].ring == 0){
					m_OutMsgArray[cursor]->points[point_index] = point;
				}
				else{
					pthread_mutex_lock(&m_RedundantPointLock);
					m_RedundantPointBuffer.push_back(RedundantPoint{point_index, point});
					pthread_mutex_unlock(&m_RedundantPointLock);
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
	if(NULL != m_funcGpsCallback) {
		m_funcGpsCallback(static_cast<double>(mktime(&t) + m_iTimeZoneSecond));
	}
}