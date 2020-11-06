/*
 *  Copyright (c) 2020 Hesai Photonics Technology, Lingwen Fang
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  Input classes for the Pandar128 3D LIDAR:
 *
 *     Input -- base class used to access the data independently of
 *              its source
 *
 *     InputSocket -- derived class reads live data from the device
 *              via a UDP socket
 *
 *     InputPCAP -- derived class provides a similar interface from a
 *              PCAP dump
 */

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <sys/file.h>
#include <sys/socket.h>
#include <unistd.h>
#include <sstream>
#include "input.h"
#include "platUtil.h"


static const size_t packet_size = sizeof(PandarPacket().data);

////////////////////////////////////////////////////////////////////////
// Input base class implementation
////////////////////////////////////////////////////////////////////////

/** @brief constructor
 *
 *  @param deviceipaddr device ip address
 *  @param port UDP port number.
 */
Input::Input(std::string deviceipaddr, uint16_t lidarport){
	m_sDeviceIpAddr = deviceipaddr;
	m_u16LidarPort = lidarport;
	m_sUdpVresion = "";
	m_bGetUdpVersion = false;
	m_iTimestampIndex = 0;
	m_iUtcIindex = 0;
	m_iSequenceNumberIndex = 0;
	m_iPacketSize = 0;
	if(!m_sDeviceIpAddr.empty())
		printf("Accepting packets from IP address: %s\n", m_sDeviceIpAddr.c_str());
}

void Input::setUdpVersion(uint8_t major, uint8_t minor) {
	if(UDP_VERSION_MAJOR_1 == major) {
		if(UDP_VERSION_MINOR_3 == minor) {
			m_sUdpVresion = UDP_VERSION_1_3;
			m_iTimestampIndex = udpVersion13[TIMESTAMP_INDEX];
			m_iUtcIindex = udpVersion13[UTC_INDEX];
			m_iSequenceNumberIndex = udpVersion13[SEQUENCE_NUMBER_INDEX];
			m_iPacketSize = udpVersion13[PACKET_SIZE];
			m_bGetUdpVersion = true;
			return;
		}
		if(UDP_VERSION_MINOR_4 == minor) {
			m_sUdpVresion = UDP_VERSION_1_4;
			m_iTimestampIndex = udpVersion14[TIMESTAMP_INDEX];
			m_iUtcIindex = udpVersion14[UTC_INDEX];
			m_iSequenceNumberIndex = udpVersion14[SEQUENCE_NUMBER_INDEX];
			m_iPacketSize = udpVersion14[PACKET_SIZE];
			m_bGetUdpVersion = true;
			return;
		}
		printf("error udp version minor: %d\n", minor);
	}
	else{
		printf("error udp version major: %d\n", major);
	}	
}

std::string Input::getUdpVersion() {
	return m_sUdpVresion;
}

////////////////////////////////////////////////////////////////////////
// InputSocket class implementation
////////////////////////////////////////////////////////////////////////

/** @brief constructor
 *
 *  @param deviceipaddr device ip address
 *  @param port UDP port number
 */
InputSocket::InputSocket(std::string deviceipaddr, uint16_t lidarport, uint16_t gpsport)
	: Input(deviceipaddr, lidarport) {
	m_iSockfd = -1;
	m_iSockGpsfd = -1;
	m_u32Sequencenum = 0;

	// connect to Pandar UDP port
	printf("Opening UDP socket: %d\n", lidarport);
	m_iSockfd = socket(PF_INET, SOCK_DGRAM, 0);
	if(m_iSockfd == -1) {
		perror("socket");  // TODO: ROS_ERROR errno
		return;
  	}

	sockaddr_in my_addr;                   // my address information
	memset(&my_addr, 0, sizeof(my_addr));  // initialize to zeros
	my_addr.sin_family = AF_INET;          // host byte order
	my_addr.sin_port = htons(lidarport);        // port in network byte order
	my_addr.sin_addr.s_addr = INADDR_ANY;  // automatically fill in my IP

	if(bind(m_iSockfd, (sockaddr *)&my_addr, sizeof(sockaddr)) == -1) {
		perror("bind error");  // TODO: ERROR errno
		return;
	}

	if(fcntl(m_iSockfd, F_SETFL, O_NONBLOCK | FASYNC) < 0) {
		perror("non-block");
		return;
	}
	m_iSocktNumber = 1;

	if(0 != gpsport) {
		m_iSockGpsfd = socket(PF_INET, SOCK_DGRAM, 0);
		if (m_iSockGpsfd == -1) {
			perror("socket");  // TODO: perror errno.
			return;
		}

		sockaddr_in myAddressGPS;                        // my address information
		memset(&myAddressGPS, 0, sizeof(myAddressGPS));  // initialize to zeros
		myAddressGPS.sin_family = AF_INET;               // host byte order
		myAddressGPS.sin_port = htons(gpsport);          // port in network byte order
		myAddressGPS.sin_addr.s_addr = INADDR_ANY;  // automatically fill in my IP

		if (bind(m_iSockGpsfd, reinterpret_cast<sockaddr *>(&myAddressGPS), sizeof(sockaddr)) == -1) {
			perror("bind");  // TODO: perror errno
			return;
		}

		if (fcntl(m_iSockGpsfd, F_SETFL, O_NONBLOCK | FASYNC) < 0) {
			perror("non-block");
			return;
		}
		m_iSocktNumber = 2;
	}

	int getchecksum = 0;
	socklen_t option_int = sizeof(int);
	int get_error = getsockopt(m_iSockfd, SOL_SOCKET, SO_NO_CHECK, &getchecksum, &option_int);
	int nochecksum = 1;
	int set_error = setsockopt(m_iSockfd, SOL_SOCKET, SO_NO_CHECK, &nochecksum, sizeof(nochecksum));
	printf("Pandar socket fd is %d\n", m_iSockfd);
}

/** @brief destructor */
InputSocket::~InputSocket(void) { 
	if(m_iSockGpsfd >0) close(m_iSockGpsfd);
	if(m_iSockfd >0) (void)close(m_iSockfd); 
}

// return : 0 - lidar
//          2 - gps
//          1 - error
/** @brief Get one pandar packet. */
int InputSocket::getPacket(PandarPacket *pkt) {
	// double time1 = ros::Time::now().toSec();

	uint64_t startTime = 0;
	uint64_t endTime = 0;
	uint64_t midTime = 0;
	timespec time;
	memset(&time, 0, sizeof(time));

	struct pollfd fds[m_iSocktNumber];
	if(m_iSocktNumber == 2) {
		fds[0].fd = m_iSockGpsfd;
		fds[0].events = POLLIN;

		fds[1].fd = m_iSockfd;
		fds[1].events = POLLIN;
	} 
	else if(m_iSocktNumber == 1) {
		fds[0].fd = m_iSockfd;
		fds[0].events = POLLIN;
	}
	static const int POLL_TIMEOUT = 20;  // one second (in msec)

	sockaddr_in sender_address;
	socklen_t sender_address_len = sizeof(sender_address);
	int retval = poll(fds, m_iSocktNumber, POLL_TIMEOUT);
	if(retval < 0) { // poll() error?
		if(errno != EINTR) printf("poll() error: %s\n", strerror(errno));
		return 1;
	}
	if(retval == 0) { // poll() timeout?
		printf("Pandar poll() timeout\n");
		return 1;
	}
	if((fds[0].revents & POLLERR) || (fds[0].revents & POLLHUP) ||(fds[0].revents & POLLNVAL)) { // device error?
		printf("poll() reports Pandar error\n");
		return 1;
	}
	ssize_t nbytes;
  	for (int i = 0; i != m_iSocktNumber; ++i) {
		  printf("m_iSocktNumber %d\n",m_iSocktNumber);
    	if (fds[i].revents & POLLIN) {
      		nbytes = recvfrom(m_iSockfd, &pkt->data[0], 10000, 0, (sockaddr *)&sender_address, &sender_address_len);
			
			printf("fds[%d] size: %d\n",i, nbytes);
      		break;
    	}
  	}
	if(GPS_PACKET_SIZE == nbytes) {
		return 2;
	}
	calcPacketLoss(pkt);
	return 0;
}

void InputSocket::calcPacketLoss(PandarPacket *pkt) {
	if(m_bGetUdpVersion) {
		static uint32_t dropped = 0, u32StartSeq = 0;
		static uint32_t startTick = GetTickCount();
		uint32_t *pSeq = (uint32_t *)&pkt->data[m_iSequenceNumberIndex];
		uint32_t seqnub = *pSeq;
		// printf("index: %d,%d, seq: %d\n", nbytes, m_iSequenceNumberIndex, seqnub);

		if(m_u32Sequencenum == 0) {
			m_u32Sequencenum = seqnub;
			u32StartSeq = m_u32Sequencenum;
		} 
		else {
			uint32_t diff = seqnub - m_u32Sequencenum;
			if(diff > 1) {
				printf("seq diff: %x \n", diff);
				dropped += diff - 1;
			}
		}
		m_u32Sequencenum = seqnub;
		uint32_t endTick = GetTickCount();
		if(endTick - startTick >= 1000 && dropped > 0) {
			printf("dropped: %d, %d, percent, %f\n", dropped, m_u32Sequencenum - u32StartSeq,
					float(dropped) / float(m_u32Sequencenum - u32StartSeq) * 100.0);
			dropped = 0;
			u32StartSeq = m_u32Sequencenum;
			startTick = endTick;
		}
	}
}

////////////////////////////////////////////////////////////////////////
// InputPCAP class implementation
////////////////////////////////////////////////////////////////////////

/** @brief constructor
 *
 *  @param private_nh ROS private handle for calling node.
 *  @param port UDP port number
 *  @param packet_rate expected device packet frequency (Hz)
 *  @param filename PCAP dump file name
 */
InputPCAP::InputPCAP(std::string deviceipaddr, uint16_t lidarport, std::string pcapfile)
    : Input(deviceipaddr, lidarport) {
	m_pcapt = NULL;
	m_sPcapFile = pcapfile;
	
	// Open the PCAP dump file
	printf("Opening PCAP file \"%s\"\n", m_sPcapFile.c_str());
	if((m_pcapt = pcap_open_offline(m_sPcapFile.c_str(), m_cErrorArray)) == NULL) {
		printf("Error opening Pandar socket dump file.\n");
		return;
	}

	std::stringstream filter;
	if(m_sDeviceIpAddr != "") { // using specific IP?
		filter << "src host " << m_sDeviceIpAddr << " && ";
	}
	filter << "udp dst port " << lidarport;
	pcap_compile(m_pcapt, &m_objPcapPacketFilter, filter.str().c_str(), 1, PCAP_NETMASK_UNKNOWN);
	m_iTimeGap = 1000;
	m_i64LastPktTimestamp = 0;
	m_iPktCount = 0;
	m_i64LastTime = 0;
	m_i64CurrentTime = 0;
	m_i64PktTimestamp = 0;
}

/** destructor */
InputPCAP::~InputPCAP(void) { pcap_close(m_pcapt); }

// return : 0 - lidar
//          2 - gps
//          1 - error
/** @brief Get one pandar packet. */
int InputPCAP::getPacket(PandarPacket *pkt) {
	pcap_pkthdr *pktHeader;
	const unsigned char *packetBuf;

	if(pcap_next_ex(m_pcapt, &pktHeader, &packetBuf) >= 0) {
		const uint8_t *packet = packetBuf + 42;
		memcpy(&pkt->data[0], packetBuf + 42, packet_size);
		m_iPktCount++;
		if(m_bGetUdpVersion && (m_iPktCount >= m_iTimeGap)) {
			sleep(packet);
		}
		pkt->stamp = getNowTimeSec();  // time_offset not considered here, as no synchronization required
		if(!m_bGetUdpVersion)
			return 0;
		if(pktHeader->caplen == (GPS_PACKET_SIZE + 42)) {
			// ROS_ERROR("GPS");
			return 2;
		}
		if(pktHeader->caplen == (m_iPacketSize + 42)) {
			return 0;  // success
		}
	}
	return 1;
}

void InputPCAP::sleep(const uint8_t *packet) {
	struct tm t;
	m_iPktCount = 0;
	t.tm_year  = packet[m_iUtcIindex];
	t.tm_mon   = packet[m_iUtcIindex+1] - 1;
	t.tm_mday  = packet[m_iUtcIindex+2];
	t.tm_hour  = packet[m_iUtcIindex+3];
	t.tm_min   = packet[m_iUtcIindex+4];
	t.tm_sec   = packet[m_iUtcIindex+5];
	t.tm_isdst = 0;
	m_i64PktTimestamp = mktime(&t) * 1000000 + ((packet[m_iTimestampIndex]& 0xff) | \
		(packet[m_iTimestampIndex+1]& 0xff) << 8 | \
		((packet[m_iTimestampIndex+2]& 0xff) << 16) | \
		((packet[m_iTimestampIndex+3]& 0xff) << 24));
	struct timeval sys_time;
	gettimeofday(&sys_time, NULL);
	m_i64CurrentTime = sys_time.tv_sec * 1000000 + sys_time.tv_usec;

	if(0 == m_i64LastPktTimestamp) {
		m_i64LastPktTimestamp = m_i64PktTimestamp;
		m_i64LastTime = m_i64CurrentTime;
	} 
	else {
		int64_t sleep_time = (m_i64PktTimestamp - m_i64LastPktTimestamp) - (m_i64CurrentTime - m_i64LastTime);
		// printf("pkt time: %u,use time: %u,sleep time: %u",pkt_ts - m_i64LastPktTimestamp,m_i64CurrentTime - m_i64LastTime, sleep_time);
		if(sleep_time > 0) {
			struct timeval waitTime;
			waitTime.tv_sec = sleep_time / 1000000;
			waitTime.tv_usec = sleep_time % 1000000;
			int err;
			do {
				err = select(0, NULL, NULL, NULL, &waitTime);
			} while (err < 0 && errno != EINTR);
		}
		m_i64LastPktTimestamp = m_i64PktTimestamp;
		m_i64LastTime = m_i64CurrentTime;
		m_i64LastTime += sleep_time;
	}
}
