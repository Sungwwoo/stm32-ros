#ifndef _IMU_DRIVER_H_
#define _IMU_DRIVER_H_

#include "stm32f4xx_hal.h"
typedef enum GyroRange_ {
	GYRO_RANGE_250DPS = 0,
	GYRO_RANGE_500DPS,
	GYRO_RANGE_1000DPS,
	GYRO_RANGE_2000DPS
} GyroRange;

typedef enum AccelRange_ {
	ACCEL_RANGE_2G = 0,
	ACCEL_RANGE_4G,
	ACCEL_RANGE_8G,
	ACCEL_RANGE_16G
} AccelRange;

typedef enum DLPFBandwidth_ {
	DLPF_BANDWIDTH_184HZ = 0,
	DLPF_BANDWIDTH_92HZ,
	DLPF_BANDWIDTH_41HZ,
	DLPF_BANDWIDTH_20HZ,
	DLPF_BANDWIDTH_10HZ,
	DLPF_BANDWIDTH_5HZ
} DLPFBandwidth;

typedef enum SampleRateDivider_ {
	LP_ACCEL_ODR_0_24HZ = 0,
	LP_ACCEL_ODR_0_49HZ,
	LP_ACCEL_ODR_0_98HZ,
	LP_ACCEL_ODR_1_95HZ,
	LP_ACCEL_ODR_3_91HZ,
	LP_ACCEL_ODR_7_81HZ,
	LP_ACCEL_ODR_15_63HZ,
	LP_ACCEL_ODR_31_25HZ,
	LP_ACCEL_ODR_62_50HZ,
	LP_ACCEL_ODR_125HZ,
	LP_ACCEL_ODR_250HZ,
	LP_ACCEL_ODR_500HZ
} SampleRateDivider;

class ImuDriver{
private:
	SPI_HandleTypeDef* imuPort;
	GPIO_TypeDef* MPU9250_CS_GPIO;
	uint16_t MPU9250_CS_PIN;
	__weak void MPU9250_OnActivate(){};
	inline void MPU9250_Activate();
	inline void MPU9250_Deactivate();
	uint8_t SPI_WriteRead(uint8_t byte);
	void MPU_SPI_Write (uint8_t *pBuffer, uint8_t writeAddr, uint16_t numByteToWrite);
	void MPU_SPI_Read(uint8_t *pBuffer, uint8_t readAddr, uint16_t numByteToRead);
	void writeRegister(uint8_t subAddress, uint8_t data);
	void readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest);
	uint8_t whoAmI();
	int whoAmIAK8963();
	uint8_t MPU9250_Init();
	void writeAK8963Register(uint8_t subAddress, uint8_t data);
	void readAK8963Registers(uint8_t subAddress, uint8_t count, uint8_t* dest);
public:
	void MPU9250_SetAccelRange(AccelRange range);
	void MPU9250_SetGyroRange(GyroRange range);
	void MPU9250_SetDLPFBandwidth(DLPFBandwidth bandwidth);
	void MPU9250_SetSampleRateDivider(SampleRateDivider srd);
	void imu_init(SPI_HandleTypeDef* port, GPIO_TypeDef* portx, uint16_t pin);
	void MPU9250_GetData(int16_t* accData, int16_t* magData, int16_t* gyroData);
};


#endif
