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

#define READ_PACKET_SIZE (1800)
typedef struct PandarGPS_s PandarGPS;
typedef std::array<PandarPacket, READ_PACKET_SIZE> PandarPacketsArray;
class Pandar128SDK;

class Pandar128Driver {
 public:
	Pandar128Driver(std::string deviceipaddr, uint16_t lidarport, uint16_t gpsport, std::string frameid, std::string pcapfile,
                	boost::function<void(PandarPacketsArray*)> rawcallback, \
					Pandar128SDK *pandar128sdk, std::string publishmode, std::string datatype);
	~Pandar128Driver() {}

/** poll the device
 *
 *  @returns true unless end of file reached
 */
	bool poll(void);
	void publishRawData();
	void setUdpVersion(uint8_t major, uint8_t minor);

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
	Pandar128SDK *m_pPandar128SDK;
};

#endif  // _PANDAR_DRIVER_H_
