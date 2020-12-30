/* -*- mode: C++ -*-
 *
 *  Copyright (c) 2020 Hesai Photonics Technology, Lingwen Fang
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file
 *
 *  Pandar128 3D LIDAR data input classes
 *
 *    These classes provide raw Pandar128 LIDAR input packets from
 *    either a live socket interface or a previously-saved PCAP dump
 *    file.
 *
 *  Classes:
 *
 *     pandar::Input -- base class for accessing the data
 *                      independently of its source
 *
 *     pandar::InputSocket -- derived class reads live data from the
 *                      device via a UDP socket
 *
 *     pandar::InputPCAP -- derived class provides a similar interface
 *                      from a PCAP dump file
 */

#ifndef __PANDAR_INPUT_H
#define __PANDAR_INPUT_H

#include <unistd.h>
#include <stdio.h>
#include <pcap.h>
#include <netinet/in.h>
#include <string>
#include <map>
#include "util.h"

#define ETHERNET_MTU (1500)
#define UDP_VERSION_MAJOR_1 (1)
#define UDP_VERSION_MINOR_3 (3)
#define UDP_VERSION_MINOR_4 (4)
#define UDP_VERSION_1_3 "1.3"
#define UDP_VERSION_1_4 "1.4"
#define GPS_PACKET_SIZE (512)

enum enumIndex{
	TIMESTAMP_INDEX,
	UTC_INDEX,
	SEQUENCE_NUMBER_INDEX,
	PACKET_SIZE,
};

static std::map<enumIndex, int> udpVersion13 = {
	{TIMESTAMP_INDEX, 796},
	{UTC_INDEX, 802},
	{SEQUENCE_NUMBER_INDEX, 817},
	{PACKET_SIZE, 812},
};

static std::map<enumIndex, int> udpVersion14 = {
	{TIMESTAMP_INDEX, 826},
	{UTC_INDEX, 820},
	{SEQUENCE_NUMBER_INDEX, 831},
	{PACKET_SIZE, 893},
};

typedef struct PandarPacket_s {
  double stamp;
  uint8_t data[ETHERNET_MTU];
  uint32_t size;
} PandarPacket;

static uint16_t DATA_PORT_NUMBER = 2368;     // default data port
static uint16_t GPS_PORT_NUMBER = 10110;     // default gps port
/** @brief pandar input base class */
class Input
{
public:
	Input(std::string deviceipaddr, uint16_t lidarport);
	virtual ~Input() {}

	/** @brief Read one pandar packet.
	 *
	 * @param pkt points to pandarPacket message
	 *
	 * @returns 0 if successful,
	 *          -1 if end of file
	 *          > 0 if incomplete packet (is this possible?)
	 */
	virtual int getPacket(PandarPacket *pkt) = 0;
	bool checkPacketSize(PandarPacket *pkt);
	void setUdpVersion(uint8_t major, uint8_t minor);
	std::string getUdpVersion();

protected:
	uint16_t m_u16LidarPort;
	std::string m_sDeviceIpAddr;
	std::string m_sUdpVresion;
	bool m_bGetUdpVersion;
	int m_iTimestampIndex;
	int m_iUtcIindex;
	int m_iSequenceNumberIndex;
	int m_iPacketSize;
};

/** @brief Live pandar input from socket. */
class InputSocket: public Input
{
public:
	InputSocket(std::string deviceipaddr, uint16_t lidarport = DATA_PORT_NUMBER, uint16_t gpsport = GPS_PORT_NUMBER);
	virtual ~InputSocket();
	virtual int getPacket(PandarPacket *pkt);
	void calcPacketLoss(PandarPacket *pkt);

private:
	int m_iSockfd;
	int m_iSockGpsfd;
	int m_iSocktNumber;
	uint32_t m_u32Sequencenum;
};

/** @brief pandar input from PCAP dump file.
 *
 * Dump files can be grabbed by libpcap, pandar's DSR software,
 * ethereal, wireshark, tcpdump, or the \ref vdump_command.
 */
class InputPCAP: public Input
{
public:
	InputPCAP(std::string deviceipaddr, uint16_t lidarport, std::string pcapfile);
	virtual ~InputPCAP();
	virtual int getPacket(PandarPacket *pkt);
	void sleep(const uint8_t *packet);

private:
	std::string m_sPcapFile;
	pcap_t *m_pcapt;
	bpf_program m_objPcapPacketFilter;
	char m_cErrorArray[PCAP_ERRBUF_SIZE];
	int m_iTimeGap;
	int64_t m_i64LastPktTimestamp;
	int m_iPktCount;
	int64_t m_i64LastTime;
	int64_t m_i64CurrentTime;
	int64_t m_i64PktTimestamp;
};

#endif // __PANDAR_INPUT_H
