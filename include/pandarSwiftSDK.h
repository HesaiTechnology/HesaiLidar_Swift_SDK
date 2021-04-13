/* -*- mode: C++ -*- */
/*
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2011 Jesse Vera
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  Copyright (c) 2017 Hesai Photonics Technology, Yang Sheng
 *  Copyright (c) 2020 Hesai Photonics Technology, Lingwen Fang
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This class Pandar128SDK raw Pandar128 3D LIDAR packets to PointCloud2.

*/

#ifndef _PANDAR_POINTCLOUD_PANDAR128SDK_H_
#define _PANDAR_POINTCLOUD_PANDAR128SDK_H_ 1

#include <pthread.h>
#include <semaphore.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <boost/atomic.hpp>
#include <boost/lockfree/queue.hpp>
#include "pandarSwiftDriver.h"
#include "laser_ts.h"
#include "tcp_command_client.h"
#include "point_types.h"

#ifndef CIRCLE
#define CIRCLE (36000)
#endif

#define PANDARSDK_TCP_COMMAND_PORT (9347)
#define LIDAR_DATA_TYPE "lidar"
#define LIDAR_ANGLE_SIZE_10 (10)
#define LIDAR_ANGLE_SIZE_18 (18)
#define LIDAR_ANGLE_SIZE_20 (20)
#define LIDAR_ANGLE_SIZE_40 (40)
#define LIDAR_RETURN_BLOCK_SIZE_1 (1)
#define LIDAR_RETURN_BLOCK_SIZE_2 (2)

#define GPS_PACKET_SIZE (512)
#define GPS_PACKET_FLAG_SIZE (2)
#define GPS_PACKET_YEAR_SIZE (2)
#define GPS_PACKET_MONTH_SIZE (2)
#define GPS_PACKET_DAY_SIZE (2)
#define GPS_PACKET_HOUR_SIZE (2)
#define GPS_PACKET_MINUTE_SIZE (2)
#define GPS_PACKET_SECOND_SIZE (2)
#define GPS_ITEM_NUM (7)

#define PANDAR128_LASER_NUM (128)
#define PANDAR64S_LASER_NUM (64)
#define PANDAR40S_LASER_NUM (40)
#define PANDAR80_LASER_NUM (80)
#define PANDAR128_BLOCK_NUM (2)
#define MAX_BLOCK_NUM (8)
#define PANDAR128_DISTANCE_UNIT (0.004)
#define PANDAR128_SOB_SIZE (2)
#define PANDAR128_VERSION_MAJOR_SIZE (1)
#define PANDAR128_VERSION_MINOR_SIZE (1)
#define PANDAR128_HEAD_RESERVED1_SIZE (2)
#define PANDAR128_LASER_NUM_SIZE (1)
#define PANDAR128_BLOCK_NUM_SIZE (1)
#define PANDAR128_ECHO_COUNT_SIZE (1)
#define PANDAR128_ECHO_NUM_SIZE (1)
#define PANDAR128_HEAD_RESERVED2_SIZE (2)
#define PANDAR128_HEAD_SIZE                                       \
  (PANDAR128_SOB_SIZE + PANDAR128_VERSION_MAJOR_SIZE +            \
   PANDAR128_VERSION_MINOR_SIZE + PANDAR128_HEAD_RESERVED1_SIZE + \
   PANDAR128_LASER_NUM_SIZE + PANDAR128_BLOCK_NUM_SIZE +          \
   PANDAR128_ECHO_COUNT_SIZE + PANDAR128_ECHO_NUM_SIZE +          \
   PANDAR128_HEAD_RESERVED2_SIZE)
#define PANDAR128_AZIMUTH_SIZE (2)
#define DISTANCE_SIZE (2)
#define INTENSITY_SIZE (1)
#define CONFIDENCE_SIZE (1)
#define PANDAR128_UNIT_WITHOUT_CONFIDENCE_SIZE (DISTANCE_SIZE + INTENSITY_SIZE)
#define PANDAR128_UNIT_WITH_CONFIDENCE_SIZE (DISTANCE_SIZE + INTENSITY_SIZE + CONFIDENCE_SIZE)
#define PANDAR128_BLOCK_SIZE \
  (PANDAR128_UNIT_WITHOUT_CONFIDENCE_SIZE* PANDAR128_LASER_NUM + PANDAR128_AZIMUTH_SIZE)
#define PANDAR128_TAIL_RESERVED1_SIZE (3)
#define PANDAR128_TAIL_RESERVED2_SIZE (3)
#define PANDAR128_SHUTDOWN_FLAG_SIZE (1)
#define PANDAR128_TAIL_RESERVED3_SIZE (3)
#define PANDAR128_MOTOR_SPEED_SIZE (2)
#define PANDAR128_TS_SIZE (4)
#define PANDAR128_RETURN_MODE_SIZE (1)
#define PANDAR128_FACTORY_INFO (1)
#define PANDAR128_UTC_SIZE (6)
#define PANDAR128_TAIL_SIZE                                        \
  (PANDAR128_TAIL_RESERVED1_SIZE + PANDAR128_TAIL_RESERVED2_SIZE + \
   PANDAR128_SHUTDOWN_FLAG_SIZE + PANDAR128_TAIL_RESERVED3_SIZE +  \
   PANDAR128_MOTOR_SPEED_SIZE + PANDAR128_TS_SIZE +                \
   PANDAR128_RETURN_MODE_SIZE + PANDAR128_FACTORY_INFO + PANDAR128_UTC_SIZE)
// #define PANDAR128_PACKET_SIZE                                         \
//   (PANDAR128_HEAD_SIZE + PANDAR128_BLOCK_SIZE * PANDAR128_BLOCK_NUM + \
//    PANDAR128_TAIL_SIZE)
#define PANDAR128_SEQ_NUM_SIZE (4)
// #define PANDAR128_PACKET_SEQ_NUM_SIZE \
//   (PANDAR128_PACKET_SIZE + PANDAR128_SEQ_NUM_SIZE)
#define PANDAR128_WITHOUT_CONF_UNIT_SIZE (DISTANCE_SIZE + INTENSITY_SIZE)

#define TASKFLOW_STEP_SIZE (225)
#define PANDAR128_CRC_SIZE (4)
#define PANDAR128_FUNCTION_SAFETY_SIZE (17)

#define CIRCLE_ANGLE (36000)
#define MOTOR_SPEED_600 (600)
#define MOTOR_SPEED_900 (900)
#define MOTOR_SPEED_1200 (1200)

typedef struct __attribute__((__packed__)) Pandar128Unit_s {
  uint16_t u16Distance;
  uint8_t u8Intensity;
  // uint8_t  u8Confidence;
} Pandar128Unit;

typedef struct __attribute__((__packed__)) Pandar128Block_s {
  uint16_t fAzimuth;
  Pandar128Unit units[PANDAR128_LASER_NUM];
} Pandar128Block;

typedef struct Pandar128HeadVersion13_s {
  uint16_t u16Sob;
  uint8_t u8VersionMajor;
  uint8_t u8VersionMinor;
  uint8_t u8DistUnit;
  uint8_t u8Flags;
  uint8_t u8LaserNum;
  uint8_t u8BlockNum;
  uint8_t u8EchoCount;
  uint8_t u8EchoNum;
  uint16_t u16Reserve1;
} Pandar128HeadVersion13;

typedef struct Pandar128HeadVersion14_s {
  uint16_t u16Sob;
  uint8_t u8VersionMajor;
  uint8_t u8VersionMinor;
  uint16_t u16Reserve1;
  uint8_t u8LaserNum;
  uint8_t u8BlockNum;
  uint8_t u8EchoCount;
  uint8_t u8DistUnit;
  uint8_t u8EchoNum;
  uint8_t u8Flags;
  inline bool hasSeqNum() const { return u8Flags & 1; }
  inline bool hasImu() const { return u8Flags & 2; }
  inline bool hasFunctionSafety() const { return u8Flags & 4; }
  inline bool hasSignature() const { return u8Flags & 8; }
  inline bool hasConfidence() const { return u8Flags & 0x10; }

} Pandar128HeadVersion14;

typedef struct Pandar128TailVersion13_s {
  uint8_t nReserved1[3];
  uint8_t nReserved2[3];
  uint8_t nShutdownFlag;
  uint8_t nReserved3[3];
  uint16_t nMotorSpeed;
  uint32_t nTimestamp;
  uint8_t nReturnMode;
  uint8_t nFactoryInfo;
  uint8_t nUTCTime[6];
  uint32_t nSeqNum;
} Pandar128TailVersion13;

typedef struct Pandar128TailVersion14_s {
  uint8_t nReserved1[3];
  uint8_t nReserved2[3];
  uint8_t nReserved3[3];
  uint16_t nAzimuthFlag;
  uint8_t nShutdownFlag;
  uint8_t nReturnMode;
  uint16_t nMotorSpeed;
  uint8_t nUTCTime[6];
  uint32_t nTimestamp;
  uint8_t nFactoryInfo;
  uint32_t nSeqNum;
} Pandar128TailVersion14;

typedef struct __attribute__((__packed__)) Pandar128PacketVersion13_t {
  Pandar128HeadVersion13 head;
  Pandar128Block blocks[PANDAR128_BLOCK_NUM];
  Pandar128TailVersion13 tail;
} Pandar128PacketVersion13;

struct PandarGPS_s {
  uint16_t flag;
  uint16_t year;
  uint16_t month;
  uint16_t day;
  uint16_t second;
  uint16_t minute;
  uint16_t hour;
  uint32_t fineTime;
};

typedef std::array<PandarPacket, 36000> PktArray;

typedef struct PacketsBuffer_s {
    PktArray m_buffers{};
    PktArray::iterator m_iterPush;
    PktArray::iterator m_iterTaskBegin;
    PktArray::iterator m_iterTaskEnd;
    int m_stepSize;
    bool m_startFlag;
    inline PacketsBuffer_s() {
        m_stepSize = TASKFLOW_STEP_SIZE;
        m_iterPush = m_buffers.begin();
        m_iterTaskBegin = m_buffers.begin();
        m_iterTaskEnd = m_iterTaskBegin + m_stepSize;
        m_startFlag = false;
    }

    inline int push_back(PandarPacket pkt) {
        if(!m_startFlag) {
			*(m_iterPush++) = pkt;
			m_startFlag = true;
          	return 1;
        } 
		else {
          	if(m_buffers.end() == m_iterPush) {
            	m_iterPush = m_buffers.begin();
          	}
			static bool lastOverflowed = false;
			if(m_iterPush == m_iterTaskBegin) {
				static uint32_t tmp = m_iterTaskBegin - m_buffers.begin();
				if(m_iterTaskBegin - m_buffers.begin() != tmp) {
					printf("buffer don't have space!,%d\n",m_iterTaskBegin - m_buffers.begin());
					tmp = m_iterTaskBegin - m_buffers.begin();
				}
				lastOverflowed = true;
				return 0;
			}
			if(lastOverflowed) {
				lastOverflowed = false;
				printf("buffer recovered\n");
			}
			*(m_iterPush++) = pkt;
			return 1;
        }
      }

    inline bool hasEnoughPackets() {
      return ((m_iterPush > m_iterTaskBegin && m_iterPush > m_iterTaskEnd) ||
              (m_iterPush < m_iterTaskBegin && m_iterPush < m_iterTaskEnd));
    }

    inline PktArray::iterator getTaskBegin() { return m_iterTaskBegin; }
    inline PktArray::iterator getTaskEnd() { return m_iterTaskEnd; }
	inline void moveTaskEnd(PktArray::iterator iter) {
		m_iterTaskEnd = iter;
		// printf("push: %d, begin: %d, end: %d\n",m_iterPush-m_buffers.begin(),m_iterTaskBegin-m_buffers.begin(),m_iterTaskEnd-m_buffers.begin());
		}
    inline void creatNewTask() {
		if(m_buffers.end() == m_iterTaskEnd) {
			m_iterTaskBegin = m_buffers.begin();
			m_iterTaskEnd = m_iterTaskBegin + m_stepSize;
		} 
		else if((m_buffers.end() - m_iterTaskEnd) < m_stepSize) {
			m_iterTaskBegin = m_iterTaskEnd;
			m_iterTaskEnd = m_buffers.end();
		}
		else {
			// printf("add step\n");
			m_iterTaskBegin = m_iterTaskEnd;
			m_iterTaskEnd = m_iterTaskBegin + m_stepSize;
		}
    }
} PacketsBuffer;

typedef PointXYZIT PPoint;
typedef pcl::PointCloud<PPoint> PPointCloud;
typedef struct RedundantPoint_s {
  int index;
  PPoint point;
} RedundantPoint;

class PandarSwiftSDK {
 public:
  /**
   * @brief Constructor
   * @param deviceipaddr  	  The ip of the device
   *        lidarport 		  The port number of lidar data
   *        gpsport   		  The port number of gps data
   *        frameid           The id of the point cloud data published to ROS
   *        correctionfile    The correction file path
   *        firtimeflie       The firtime flie path
   *        pcapfile          The pcap flie path
   *        pclcallback       The callback of PCL data structure
   *        rawcallback       The callback of raw data structure
   *        gpscallback       The callback of GPS structure
   *        certFile          Represents the path of the user's certificate
   *        privateKeyFile    Represents the path of the user's private key
   *        caFile            Represents the path of the root certificate
   *        start_angle       The start angle of every point cloud
   *                          should be <real angle> * 100.
   *        timezone          The timezone of local
   *        publishmode       The mode of publish
   *        datatype          The model of input data
   */
	PandarSwiftSDK(std::string deviceipaddr, uint16_t lidarport, uint16_t gpsport, std::string frameid, std::string correctionfile, std::string firtimeflie, std::string pcapfile, \
								boost::function<void(boost::shared_ptr<PPointCloud>, double)> pclcallback, \
								boost::function<void(PandarPacketsArray*)> rawcallback, \
								boost::function<void(double)> gpscallback, \
								std::string certFile, std::string privateKeyFile, std::string caFile, \
								int startangle, int timezone, std::string publishmode, bool coordinateCorrectionFlag, std::string datatype=LIDAR_DATA_TYPE);
	~PandarSwiftSDK() {}

	void driverReadThread();
	void publishRawDataThread();
	void processGps(PandarGPS *gpsMsg);
	void pushLiDARData(PandarPacket packet);
	int processLiDARData();
	void publishPoints();

 private:

	int parseData(Pandar128PacketVersion13 &pkt, const uint8_t *buf, const int len);
  void calcPointXYZIT(PandarPacket &pkt, int cursor);
  void doTaskFlow(int cursor);
	void loadOffsetFile(std::string file);
	void loadCorrectionFile();
	int loadCorrectionString(std::string correctionstring);
	int checkLiadaMode();
	void init();
	void changeAngleSize();
	void changeReturnBlockSize();
	void moveTaskEndToStartAngle();
  void checkClockwise();
  void SetEnvironmentVariableTZ();
  bool isNeedPublish();

  pthread_mutex_t m_RedundantPointLock;
	boost::shared_ptr<PandarSwiftDriver> m_spPandarDriver;
  	LasersTSOffset m_objLaserOffset;
	boost::function<void(boost::shared_ptr<PPointCloud> cld, double timestamp)> m_funcPclCallback;
	boost::function<void(double timestamp)> m_funcGpsCallback;
	std::array<boost::shared_ptr<PPointCloud>, 2> m_OutMsgArray;
  std::vector<RedundantPoint> m_RedundantPointBuffer;
	PacketsBuffer m_PacketsBuffer;
	double m_dTimestamp;
	int m_iLidarRotationStartAngle;
    int m_iTimeZoneSecond;
	float m_fCosAllAngle[CIRCLE];
	float m_fSinAllAngle[CIRCLE];
	float m_fElevAngle[PANDAR128_LASER_NUM];
	float m_fHorizatalAzimuth[PANDAR128_LASER_NUM];
	std::string m_sFrameId;
	std::string m_sLidarFiretimeFile;
	std::string m_sLidarCorrectionFile;
	std::string m_sPublishmodel;
	int m_iWorkMode;
	int m_iReturnMode;
	int m_iMotorSpeed;
  int m_iLaserNum;
	int m_iAngleSize;  // 10->0.1degree,20->0.2degree
	int m_iReturnBlockSize;
	bool m_bPublishPointsFlag;
	int m_iPublishPointsIndex;
	void *m_pTcpCommandClient;
	std::string m_sDeviceIpAddr;
	std::string m_sPcapFile;
	std::string m_sSdkVersion;
	uint8_t m_u8UdpVersionMajor;
	uint8_t m_u8UdpVersionMinor;
  int m_iFirstAzimuthIndex;
  int m_iLastAzimuthIndex;
  bool m_bClockwise;
  bool m_bCoordinateCorrectionFlag;
};

#endif  // _PANDAR_POINTCLOUD_Pandar128SDK_H_
