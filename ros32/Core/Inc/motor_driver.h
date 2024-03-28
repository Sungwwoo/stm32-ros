#ifndef _MOTOR_DRIVER_H_
#define _MOTOR_DRIVER_H_

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "stm32f4xx_hal.h"
// Control table address (Dynamixel X-series)
#define ADDR_X_ID						 7
#define ADDR_X_BAUD						 8
#define ADDR_X_DRIVE_MODE				 10
#define ADDR_X_OP_MODE 					 11
#define ADDR_X_TORQUE_ENABLE            64
#define ADDR_X_GOAL_VELOCITY            104
#define ADDR_X_GOAL_POSITION            116
#define ADDR_X_REALTIME_TICK            120
#define ADDR_X_PRESENT_VELOCITY         128
#define ADDR_X_PRESENT_POSITION         132

// Limit values (XM430-W250-T)
#define DXL_LIMIT_MAX_VELOCITY            265     // MAX RPM is 61 when XL is powered 12.0V

// Data Byte Length
#define LEN_X_BAUD             			 1
#define LEN_X_ID             			 1
#define LEN_X_TORQUE_ENABLE             1
#define LEN_X_DRIVE_MODE             	 1
#define LEN_X_OP_MODE            		 1
#define LEN_X_GOAL_VELOCITY             4
#define LEN_X_GOAL_POSITION             4
#define LEN_X_REALTIME_TICK             2
#define LEN_X_PRESENT_VELOCITY          4
#define LEN_X_PRESENT_POSITION          4

#define PROTOCOL_VERSION                2.0     // Dynamixel protocol version 2.0

#define DEFAULT_BAUDRATE                56700   // baud rate of Dynamixel
#define DEVICENAME                      ""      // no need setting on OpenCR

#define TORQUE_ENABLE                   1       // Value for enabling the torque
#define TORQUE_DISABLE                  0       // Value for disabling the torque

#define TARGET_BAUD                   	 3		 // baud rate 1M

#define LEFT                            0
#define RIGHT                           1

#define LINEAR                          0
#define ANGULAR                         1

#define DEFAULT_ID						 1
#define LEFT_ID							 1
#define RIGHT_ID 						 2

#define LEFT_DRIVE_MODE					 0
#define RIGHT_DRIVE_MODE 				 1

#define VEL_CONTROL_MODE				 1
#define VELOCITY_CONSTANT_VALUE         41.69988758  // V = r * w = r     *        (RPM             * 0.10472)
                                                     //           = r     * (0.229 * Goal_Velocity) * 0.10472
                                                     //
                                                     // Goal_Velocity = V / r * 41.69988757710309

#define constrain(amt, low, high) ((amt) <= (low) ? (low) : ((amt) >= (high) ? (high) : (amt)))

// Macro for Control Table Value
#define DXL_MAKEWORD(a, b)  ((uint16_t)(((uint8_t)(((uint64_t)(a)) & 0xff)) | ((uint16_t)((uint8_t)(((uint64_t)(b)) & 0xff))) << 8))
#define DXL_MAKEDWORD(a, b) ((uint32_t)(((uint16_t)(((uint64_t)(a)) & 0xffff)) | ((uint32_t)((uint16_t)(((uint64_t)(b)) & 0xffff))) << 16))
#define DXL_LOWORD(l)       ((uint16_t)(((uint64_t)(l)) & 0xffff))
#define DXL_HIWORD(l)       ((uint16_t)((((uint64_t)(l)) >> 16) & 0xffff))
#define DXL_LOBYTE(w)       ((uint8_t)(((uint64_t)(w)) & 0xff))
#define DXL_HIBYTE(w)       ((uint8_t)((((uint64_t)(w)) >> 8) & 0xff))

// Instruction for DXL Protocol
#define INST_PING               1
#define INST_READ               2
#define INST_WRITE              3
#define INST_REG_WRITE          4
#define INST_ACTION             5
#define INST_FACTORY_RESET      6
#define INST_SYNC_WRITE         131     // 0x83
#define INST_BULK_READ          146     // 0x92
// --- Only for 2.0 --- //
#define INST_REBOOT             8
#define INST_CLEAR              16      // 0x10
#define INST_STATUS             85      // 0x55
#define INST_SYNC_READ          130     // 0x82
#define INST_BULK_WRITE         147     // 0x93

// Communication Result
#define COMM_SUCCESS        0       // tx or rx packet communication success
#define COMM_PORT_BUSY      -1000   // Port is busy (in use)
#define COMM_TX_FAIL        -1001   // Failed transmit instruction packet
#define COMM_RX_FAIL        -1002   // Failed get status packet
#define COMM_TX_ERROR       -2000   // Incorrect instruction packet
#define COMM_RX_WAITING     -3000   // Now recieving status packet
#define COMM_RX_TIMEOUT     -3001   // There is no status packet
#define COMM_RX_CORRUPT     -3002   // Incorrect status packet
#define COMM_NOT_AVAILABLE  -9000   //

// Packet Bit Positions
#define PKT_HEADER0             0
#define PKT_HEADER1             1
#define PKT_HEADER2             2
#define PKT_RESERVED            3
#define PKT_ID                  4
#define PKT_LENGTH_L            5
#define PKT_LENGTH_H            6
#define PKT_INSTRUCTION         7
#define PKT_ERROR               8
#define PKT_PARAMETER0          8

// Error bit
#define ERRNUM_RESULT_FAIL      1       // Failed to process the instruction packet.
#define ERRNUM_INSTRUCTION      2       // Instruction error
#define ERRNUM_CRC              3       // CRC check error
#define ERRNUM_DATA_RANGE       4       // Data range error
#define ERRNUM_DATA_LENGTH      5       // Data length error
#define ERRNUM_DATA_LIMIT       6       // Data limit error
#define ERRNUM_ACCESS           7       // Access error


class MotorDriver{
private:
	UART_HandleTypeDef *leftPort, *rightPort;
	uint8_t *left_packet;
	uint8_t *right_packet;
	uint8_t readPacketL[14], readPacketR[14];

	void makeData(uint8_t* packet, uint8_t id, uint8_t op, uint8_t* data, uint16_t dataLength, uint16_t address);
	bool writeVelocity(int64_t left_value, int64_t right_value);
	uint16_t    updateCRC(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size);
	void        addStuffing(uint8_t *packet);
	void        removeStuffing(uint8_t *packet);
	int txPacket(uint8_t *txpacketL, uint8_t *txpacketR);
	int rxPacket(uint8_t *rxpacketL, uint8_t *rxpacketR);
	int txRxPacket(uint8_t *txpacketL, uint8_t *txpacketR, uint8_t *rxpacketL, uint8_t *rxpacketR);


public:
	MotorDriver(){};
	~MotorDriver(){};
	void init(UART_HandleTypeDef* left, UART_HandleTypeDef* right);
	bool readEncoder(int32_t &left_value, int32_t &right_value);
	bool controlMotor(const float wheel_radius, const float wheel_separation, float* value);

};
#endif
