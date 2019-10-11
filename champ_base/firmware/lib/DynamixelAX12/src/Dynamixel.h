#ifndef DYNAMIXEL_H
#define DYNAMIXEL_H

#include <stdint.h>


/** \brief Type of dynamixel status code */
typedef uint8_t DynamixelStatus;

/**
 * \brief Dynamixel status values
*/
enum DynStatus
{
	DYN_STATUS_OK					= 0,

	DYN_STATUS_INPUT_VOLTAGE_ERROR	= 1,
	DYN_STATUS_ANGLE_LIMIT_ERROR	= 2,
	DYN_STATUS_OVERHEATING_ERROR	= 4,
	DYN_STATUS_RANGE_ERROR			= 8,
	DYN_STATUS_CHECKSUM_ERROR		= 16,
	DYN_STATUS_OVERLOAD_ERROR		= 32,
	DYN_STATUS_INSTRUCTION_ERROR	= 64
};

/**
 * \brief Dynamixel control table addresses (only addresses used by all models)
*/
enum DynCommonAddress
{
	/** \brief Model number, uint16_t , read only */
	DYN_ADDRESS_MODEL		=0x00,
	/** \brief Firmware version, uint8_t, read only */
	DYN_ADDRESS_FIRMWARE	=0x02,
	/** \brief Device ID, uint8_t, writable */
	DYN_ADDRESS_ID			=0x03,
	/** \brief Communication baudrate, uint8_t, writable */
	DYN_ADDRESS_BAUDRATE	=0x04,
	/** \brief Return Delay Time , uint8_t, writable */
	DYN_ADDRESS_RDT			=0x05,
	/** \brief Status Return Level , uint8_t, writable 
	 *
	 * Define when the device will send back a status packet :
	 * 0 : Ping only
	 * 1 : Read and ping
	 * 2 : All instructions
	*/
	DYN_ADDRESS_SRL			=0x10 
};

/**
 * \brief Dynamixel motor control table addresses (only addresses used by all motor models)
*/
enum DynMotorAddress
{
	/** \brief Clockwise angle limit, uint16_t , writable, EEPROM */
	DYN_ADDRESS_CW_LIMIT = 0x06,
	/** \brief Counnter clockwise angle limit, uint16_t , writable, EEPROM */
	DYN_ADDRESS_CCW_LIMIT = 0x08,
	/** \brief Default maximum torque, uint16_t , writable, EEPROM */
	DYN_ADDRESS_MAX_TORQUE = 0x0E,
	/** \brief Dynamic maximum torque, uint16_t , writable, RAM */
	DYN_ADDRESS_TORQUE_LIMIT = 0x22,
	/** \brief Define error cases leading to shutdown, uint8_t , writable, EEPROM */
	DYN_ADDRESS_ALARM_SHUTDOWN = 0x12,
	/** \brief Enable torque, uint8_t , writable, RAM */
	DYN_ADDRESS_ENABLE_TORQUE = 0x18,
	/** \brief LED state, uint8_t , writable, RAM */
	DYN_ADDRESS_LED = 0x19,
	/** \brief Goal position, uint16_t , writable, RAM */
	DYN_ADDRESS_GOAL_POSITION = 0x1E,
	/** \brief Goal speed, uint16_t , writable, RAM */
	DYN_ADDRESS_GOAL_SPEED = 0x20,
	/** \brief Current position, uint16_t , readable, RAM */
	DYN_ADDRESS_CURRENT_POSITION = 0x24,

	DYN_ADDRESS_TEMP_LIMIT = 0x0B,
	DYN_ADDRESS_LOW_VOLTAGE_LIMIT = 0x0C,
	DYN_ADDRESS_HIGH_VOLTAGE_LIMIT = 0x0D,
	DYN_ADDRESS_ALARM_LED = 0x11,

};

/**
 * \brief Dynamixel model number values
*/
enum DynModel
{
	DYN_MODEL_AX12A	=0x0C,
	DYN_MODEL_AX12W	=0x2C,
	DYN_MODEL_AX18A	=0x12,
	
	DYN_MODEL_DX113	=0x71,
	DYN_MODEL_DX114	=0x74,
	DYN_MODEL_DX117	=0x75,
	
	DYN_MODEL_RX10	=0x0A,
	DYN_MODEL_RX24F	=0x18,
	DYN_MODEL_RX28	=0x1C,
	DYN_MODEL_RX64	=0x40,
	
	DYN_MODEL_EX106	=0x6B,
	
	DYN_MODEL_MX12W	=0x68,
	DYN_MODEL_MX28T	=0x1D,
	DYN_MODEL_MX28R	=0x1D,
	DYN_MODEL_MX64T	=0x36,
	DYN_MODEL_MX64R	=0x36,
	DYN_MODEL_MX106T=0x40,
	DYN_MODEL_MX106R=0x40,
	
	DYN_MODEL_AXS1	=0x0D
};


#endif
