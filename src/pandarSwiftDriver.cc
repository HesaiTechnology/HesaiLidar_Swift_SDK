/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009-2012 Austin Robot Technology, Jack O'Quin
 *  Copyright (c) 2017 Hesai Photonics Technology, Yang Sheng
 *  Copyright (c) 2020 Hesai Photonics Technology, Lingwen Fang
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  ROS driver implementation for the Pandar128 3D LIDARs
 */

#include <cmath>
#include <string>
#include "pandarSwiftSDK.h"
#include "pandarSwiftDriver.h"
#include "platUtil.h"

PandarSwiftDriver::PandarSwiftDriver(std::string deviceipaddr, uint16_t lidarport, uint16_t gpsport, std::string frameid, std::string pcapfile,
                        	boost::function<void(PandarPacketsArray*)> rawcallback, \
						    PandarSwiftSDK *pandarSwiftSDK, std::string publishmode, std::string datatype) {
	m_sFrameId = frameid;
	m_funcRawCallback = rawcallback;
	m_pPandarSwiftSDK = pandarSwiftSDK;
	m_sPublishmodel = publishmode;
	m_sDataType = datatype;
	m_bNeedPublish = false;
	m_iPktPushIndex = 0;
	m_iPktPopIndex = 1;
	m_bGetScanArraySizeFlag = false;
    m_iPandarScanArraySize = PANDAR128_READ_PACKET_SIZE;
	// open Pandar input device or file
	if(pcapfile != "") {  // have PCAP file
		// read data from packet capture file
		m_spInput.reset(new InputPCAP(deviceipaddr, lidarport, pcapfile));
		m_bPaserPacp = true; 
	} 
	else {
		// read data from live socket
		m_spInput.reset(new InputSocket(deviceipaddr, lidarport, gpsport));
		m_bPaserPacp = false; 
	}
}

//-------------------------------------------------------------------------------
int parseGPS(PandarGPS *packet, const uint8_t *recvbuf, const int size) {
	if(size != GPS_PACKET_SIZE) {
		return -1;
	}
	int index = 0;
	packet->flag = (recvbuf[index] & 0xff) | ((recvbuf[index + 1] & 0xff) << 8);
	index += GPS_PACKET_FLAG_SIZE;
	packet->year = (recvbuf[index] & 0xff - 0x30) + (recvbuf[index + 1] & 0xff - 0x30) * 10;
	index += GPS_PACKET_YEAR_SIZE;
	packet->month = (recvbuf[index] & 0xff - 0x30) + (recvbuf[index + 1] & 0xff - 0x30) * 10;
	index += GPS_PACKET_MONTH_SIZE;
	packet->day = (recvbuf[index] & 0xff - 0x30) + (recvbuf[index + 1] & 0xff - 0x30) * 10;
	index += GPS_PACKET_DAY_SIZE;
	packet->second = (recvbuf[index] & 0xff - 0x30) + (recvbuf[index + 1] & 0xff - 0x30) * 10;
	index += GPS_PACKET_SECOND_SIZE;
	packet->minute = (recvbuf[index] & 0xff - 0x30) + (recvbuf[index + 1] & 0xff - 0x30) * 10;
	index += GPS_PACKET_MINUTE_SIZE;
	packet->hour = (recvbuf[index] & 0xff - 0x30) + (recvbuf[index + 1] & 0xff - 0x30) * 10;
	index += GPS_PACKET_HOUR_SIZE;
	packet->fineTime = (recvbuf[index] & 0xff) | (recvbuf[index + 1] & 0xff) << 8 | ((recvbuf[index + 2] & 0xff) << 16) | ((recvbuf[index + 3] & 0xff) << 24);
	return 0;
}

bool PandarSwiftDriver::poll(void) {
	uint64_t startTime = 0;
	uint64_t endTime = 0;
	timespec time;
	memset(&time, 0, sizeof(time));
	bool skipSleep = true;
	for (int i = 0; i < m_iPandarScanArraySize; ++i) {
		if (m_bPaserPacp)  // have PCAP file?
		{
			int count = 0;
			while(m_pPandarSwiftSDK->getIsSocketTimeout()&& count < 2000){
				// printf("timeout %d\n", m_pPandarSwiftSDK->getIsSocketTimeout());
				usleep(1000);
				count++;
			}
		}
		
		bool isSocketTimeout = m_pPandarSwiftSDK->getIsSocketTimeout();
		int rc = m_spInput->getPacket(&m_arrPandarPackets[m_iPktPushIndex][i], isSocketTimeout, skipSleep);
		m_pPandarSwiftSDK->setIsSocketTimeout(isSocketTimeout);
		if(rc == GPS_PACKET) {
			// gps packet;
			PandarGPS packet;
			if(parseGPS(&packet, &m_arrPandarPackets[m_iPktPushIndex][i].data[0], GPS_PACKET_SIZE) == 0) {
				m_pPandarSwiftSDK->processGps(&packet);// gps callback
			}
			i--;
			continue;
		}
		if(rc == FAULT_MESSAGE_PACKET) {
			m_pPandarSwiftSDK->processFaultMessage(m_arrPandarPackets[m_iPktPushIndex][i]);
			i--;
			continue;
		}
		if(rc == LOG_REPORT_PACKET) {
			i--;
			continue;
		}
		if(rc == ERROR_PACKET){    // error packet
			i--;
			continue;
		} 
		if(rc == PCAP_END_PACKET){
			m_pPandarSwiftSDK->SetIsReadPcapOver(true);
			return true;
		}
			
		if(m_arrPandarPackets[m_iPktPushIndex][i].size < 500){
			i--;
			continue;
		}	
		if(m_sPublishmodel == "both_point_raw" || m_sPublishmodel == "point") {
			m_pPandarSwiftSDK->pushLiDARData(m_arrPandarPackets[m_iPktPushIndex][i]);
		}
		if(skipSleep || isSocketTimeout){
			usleep(3000);
		}
	}
	int temp;
	temp = m_iPktPushIndex;
	m_iPktPushIndex = m_iPktPopIndex;
	m_iPktPopIndex = temp;
	if(m_bNeedPublish == false)
		m_bNeedPublish = true;
	// else
	// 	printf("CPU not fast enough, data not published yet, new data comming!!!\n");
	return true;
}

void PandarSwiftDriver::publishRawData() {
	if(m_bNeedPublish && (NULL != m_funcRawCallback)) {
		m_funcRawCallback(&m_arrPandarPackets[m_iPktPopIndex]);
		m_bNeedPublish = false;
	}
	else {
		usleep(1000);
	}
}

void PandarSwiftDriver::setUdpVersion(uint8_t major, uint8_t minor) {
	m_spInput->setUdpVersion(major, minor);
}
int PandarSwiftDriver::getPandarScanArraySize(boost::shared_ptr<Input> input_){
  for (int i = 0; i < 256; ++i) {
    PandarPacket packet;
    bool isTimeout = false;
	bool isSkipSleep = true;
    int rc = input_->getPacket(&packet, isTimeout, isSkipSleep);
    switch (packet.data[PANDAR_LASER_NUMBER_INDEX]){
    case PANDAR128_LASER_NUM:
      return PANDAR128_READ_PACKET_SIZE;
    case PANDAR80_LASER_NUM:
      return PANDAR80_READ_PACKET_SIZE;
    case PANDAR64S_LASER_NUM:
      return PANDAR64S_READ_PACKET_SIZE;
    case PANDAR40S_LASER_NUM:
      return PANDAR40S_READ_PACKET_SIZE;
    default:
      break;
    }
  }
}