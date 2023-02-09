#ifndef __YOST_TTS_LX_H_INCLUDED__
#define __YOST_TTS_LX_H_INCLUDED__

#include "Arduino.h"
#include "SPI.h"
#include "IMUAbstract.h"
#include <vector>

namespace Cyberwing
{
	class YOST_TTS_LX  : public IMUAbstract
	{
	public:
		enum class COMMAND : uint8_t
		{
			GET_QUAT_TARED                      = 0x00,
			GET_EULER_TARED                     = 0x01,
			GET_ROT_MAT_TARED                   = 0x02,
			GET_AXIS_ANGLE_TARED                = 0x03,
			GET_FORWARD_DOWN_TARED              = 0x04,
			GET_QUAT                            = 0x06,
			GET_EULER                           = 0x07,
			GET_ROT_MAT                         = 0x08,
			GET_AXIS_ANGLE                      = 0x09,
			GET_NORTH_GRAV                      = 0x0A,
			GET_FORWARD_DOWN_TARED_SENSOR_FRAME = 0x0B,
			GET_NORTH_GRAV_SENSOR_FRAME         = 0x0C,
			SET_EULER_DECOMP_ORDER              = 0x10,
			SET_INTERRUPT_TYPE                  = 0x1D,
			GET_INTERRUPT_TYPE                  = 0x1E,
			GET_INTERRUPT_STATUS                = 0x1F,
			GET_ALL_NORMALIZED                  = 0x20,
			GET_GYRO_NORMALIZED                 = 0x21,
			GET_ACC_NORMALIZED                  = 0x22,
			GET_MAG_NORMALIZED                  = 0x23,
			GET_ALL_CORRECTED                   = 0x25,
			GET_GYRO_CORRECTED                  = 0x26,
			GET_ACC_CORRECTED                   = 0x27,
			GET_MAG_CORRECTED                   = 0x28,
			GET_LIN_ACC                         = 0x29,
			GET_TEMP_C                          = 0x2B,
			GET_TEMP_F                          = 0x2C,
			GET_GYRO_RAW_CORRECTED              = 0x30,
			GET_ACC_RAW_CORRECTED               = 0x31,
			GET_MAG_RAW_CORRECTED               = 0x32,
			GET_ALL_RAW                         = 0x40,
			GET_GYRO_RAW                        = 0x41,
			GET_ACC_RAW                         = 0x42,
			GET_MAG_RAW                         = 0x43,
			SET_STREAM_SLOT                     = 0x50,
			GET_STREAM_SLOT                     = 0x51,
			SET_STREAM_TIMING                   = 0x52,
			GET_STREAM_TIMING                   = 0x53,
			GET_STREAM_BATCH                    = 0x54,
			START_STREAM                        = 0x55,
			STOP_STREAM                         = 0x56,
			UPDATE_TIMESTAMP                    = 0x5F,
			TARE                                = 0x60,
			SET_OVERSAMPLE_RATE                 = 0x6A,
			ENABLE_DISABLE_GYRO                 = 0x6B,
			ENABLE_DISABLE_ACC                  = 0x6C,
			ENABLE_DISABLE_MAG                  = 0x6D,
			SET_AXIS_DIRECTION                  = 0x74,
			SET_RUNNING_AVERAGE                 = 0x75,
			SET_ACC_RANGE                       = 0x79,
			SET_GYRO_RANGE                      = 0x7D,
			SET_MAG_RANGE                       = 0x7E,
			CALIBRATE_GYRO                      = 0xA5,
			SET_RESPONSE_HEADER                 = 0xDD,
			GET_RESPONSE_HEADER                 = 0xDE,
			GET_VERSION_EXTENDED                = 0xDF,
			RESET_SETTINGS                      = 0xE0,
			COMMIT_SETTINGS                     = 0xE1,
			RESET_SENSOR                        = 0xE2,
			GET_VERSION                         = 0xE6,
			GET_SERIAL_NUMBER                   = 0xED
		};

		struct CONFIG
		{
			int32_t spiBaud               = 4000000;
			int32_t spiMSB                = MSBFIRST;
			int32_t spiMode               = SPI_MODE0;
			uint8_t interruptTypeMode     = 1;
			uint8_t interruptTypePin      = 0;
			uint8_t interruptTypePolarity = 0;
			uint8_t timingInterval       = 0;
			uint8_t timingDuration       = -1;
			uint8_t timingDelay          = 0;
			uint8_t oversampleRate        = 1;
			float runningAverage          = 0.0F;
			uint8_t accRange              = 0x30;
			uint8_t gyroRange             = 0x02;
			uint8_t magRange              = 0x04;
			uint8_t SPIwrite              = 0xE9;
			uint8_t SPIread               = 0x69;
			uint8_t SPIstatus             = 0x81;
			uint32_t readTimeoutLen       = 100;
			float quatOffset0[4]           = {0.7071F,0.7071F,0.0F,0.0F};
		};

	public:
		YOST_TTS_LX(const IMUAbstract::MODE &mode,
		            SPIClass &spi,
		            const int32_t &pinSS,
		            const int32_t &pinATT,
		            const uint32_t &receiveTimeout);
		YOST_TTS_LX(const IMUAbstract::MODE &mode,
			          SPIClass &spi,
			          const int32_t &pinSS,
			          const int32_t &pinATT,
			          const uint32_t &receiveTimeout,
			          const CONFIG &config);
		virtual ~YOST_TTS_LX();

		void initSPI(void);
		virtual int32_t init(bool resetSettings = false);
		virtual int32_t update(void);

		void tare(void);

	protected:
		SPIClass &spi_;
    const int32_t pinSS_;
    const int32_t pinATT_;
		const CONFIG config_;
		int32_t pinValNm1_;
		SPISettings SPIsetting_;

		bool newMsgCheck(void);
		int32_t getRawData(void);
		int32_t getFusionedData(void);

		void readMessage(const uint32_t &len,
		                    std::vector<uint8_t> &data);
		int32_t sendMessage(const COMMAND &cmd,
												const std::vector<uint8_t> &data);
		int32_t sendMessage(const COMMAND &cmd);

		void setInterruptType(void);
		void setStreamTiming(void);
		void setAxisConvention(void);
		void disableMag(void);
		void calibrateGyro(void);
		void resetSensor(void);
		void resetSetting(void);
		void commitSetting(void);
		bool isValidCommand(const uint8_t &cmd);

		bool checkConsistancy(const RAW_DATA data);
		bool checkConsistancy(float quatRaw[4], float ratesRaw[3]);
	};

}
#endif
