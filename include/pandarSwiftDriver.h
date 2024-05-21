/* -*- mode: C++ -*- */
/*
 *  Copyright (c) 2020 Hesai Photonics Technology Co., Ltd
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  SDK driver interface for the pandar 3D LIDARs
 */

#ifndef _PANDAR_DRIVER_H_
#define _PANDAR_DRIVER_H_ 1

#include <string>
#include <input.h>
#include <boost/function.hpp>

#define PANDAR128_READ_PACKET_SIZE (1800)
#define PANDARQT128_READ_PACKET_SIZE (200)
#define PANDAR80_READ_PACKET_SIZE (1800)
#define PANDAR64S_READ_PACKET_SIZE (450)
#define PANDAR90_READ_PACKET_SIZE (200)
#define PANDAR40S_READ_PACKET_SIZE (225)
#define PANDAR_LASER_NUMBER_INDEX (6)
#define PANDAR_MAJOR_VERSION_INDEX (2)
typedef struct PandarGPS_s PandarGPS;
typedef std::array<PandarPacket, PANDAR128_READ_PACKET_SIZE> PandarPacketsArray;
class PandarSwiftSDK;

class PandarSwiftDriver {
 public:
	PandarSwiftDriver(std::string deviceipaddr, std::string hostipaddr, uint16_t lidarport, uint16_t gpsport, std::string frameid, std::string pcapfile,
                	boost::function<void(PandarPacketsArray*)> rawcallback, \
					PandarSwiftSDK *pandarSwiftSDK, std::string multicast_ip, std::string publishmode, std::string datatype);
	~PandarSwiftDriver() {}

/** poll the device
 *
 *  @returns true unless end of file reached
 */
	bool poll(void);
	void publishRawData();
	void setUdpVersion(uint8_t major, uint8_t minor);
	int getPandarScanArraySize(boost::shared_ptr<Input>);

 private:

	boost::shared_ptr<Input> m_spInput;
	boost::function<void(PandarPacketsArray*)> m_funcRawCallback;
	std::string m_sFrameId;
	std::array<PandarPacketsArray, 2> m_arrPandarPackets;
	bool m_bNeedPublish;
	int m_iPktPushIndex;
	int m_iPktPopIndex;
	std::string m_sPublishmodel;
	std::string m_sDataType;
	PandarSwiftSDK *m_pPandarSwiftSDK;
	int m_iPandarScanArraySize;
    bool m_bGetScanArraySizeFlag;
};

#endif  // _PANDAR_DRIVER_H_
