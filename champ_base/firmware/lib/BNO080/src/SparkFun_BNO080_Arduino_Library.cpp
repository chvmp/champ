/*
  This is a library written for the BNO080
  SparkFun sells these at its website: www.sparkfun.com
  Do you like this library? Help support SparkFun. Buy a board!
  https://www.sparkfun.com/products/14686

  Written by Nathan Seidle @ SparkFun Electronics, December 28th, 2017

  The BNO080 IMU is a powerful triple axis gyro/accel/magnetometer coupled with an ARM processor
  to maintain and complete all the complex calculations for various VR, inertial, step counting,
  and movement operations.

  This library handles the initialization of the BNO080 and is able to query the sensor
  for different readings.

  https://github.com/sparkfun/SparkFun_BNO080_Arduino_Library

  Development environment specifics:
  Arduino IDE 1.8.5

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "SparkFun_BNO080_Arduino_Library.h"

//Attempt communication with the device
//Return true if we got a 'Polo' back from Marco
boolean BNO080::begin(uint8_t deviceAddress, TwoWire &wirePort, uint8_t intPin)
{
	_deviceAddress = deviceAddress; //If provided, store the I2C address from user
	_i2cPort = &wirePort;			//Grab which port the user wants us to use
	_int = intPin;					//Get the pin that the user wants to use for interrupts. By default, it's NULL and we'll not use it in dataAvailable() function.

	//We expect caller to begin their I2C port, with the speed of their choice external to the library
	//But if they forget, we start the hardware here.
	//_i2cPort->begin();

	//Begin by resetting the IMU
	softReset();

	//Check communication with device
	shtpData[0] = SHTP_REPORT_PRODUCT_ID_REQUEST; //Request the product ID and reset info
	shtpData[1] = 0;							  //Reserved

	//Transmit packet on channel 2, 2 bytes
	sendPacket(CHANNEL_CONTROL, 2);

	//Now we wait for response
	if (receivePacket() == true)
	{
		if (shtpData[0] == SHTP_REPORT_PRODUCT_ID_RESPONSE)
		{
			return (true);
		}
	}

	return (false); //Something went wrong
}

boolean BNO080::beginSPI(uint8_t user_CSPin, uint8_t user_WAKPin, uint8_t user_INTPin, uint8_t user_RSTPin, uint32_t spiPortSpeed, SPIClass &spiPort)
{
	_i2cPort = NULL; //This null tells the send/receive functions to use SPI

	//Get user settings
	_spiPort = &spiPort;
	_spiPortSpeed = spiPortSpeed;
	if (_spiPortSpeed > 3000000)
		_spiPortSpeed = 3000000; //BNO080 max is 3MHz

	_cs = user_CSPin;
	_wake = user_WAKPin;
	_int = user_INTPin;
	_rst = user_RSTPin;

	pinMode(_cs, OUTPUT);
	pinMode(_wake, OUTPUT);
	pinMode(_int, INPUT_PULLUP);
	pinMode(_rst, OUTPUT);

	digitalWrite(_cs, HIGH); //Deselect BNO080

	//Configure the BNO080 for SPI communication
	digitalWrite(_wake, HIGH); //Before boot up the PS0/WAK pin must be high to enter SPI mode
	digitalWrite(_rst, LOW);   //Reset BNO080
	delay(2);				   //Min length not specified in datasheet?
	digitalWrite(_rst, HIGH);  //Bring out of reset

	//Wait for first assertion of INT before using WAK pin. Can take ~104ms
	waitForSPI();

	//if(wakeBNO080() == false) //Bring IC out of sleep after reset
	//  Serial.println("BNO080 did not wake up");

	_spiPort->begin(); //Turn on SPI hardware

	//At system startup, the hub must send its full advertisement message (see 5.2 and 5.3) to the
	//host. It must not send any other data until this step is complete.
	//When BNO080 first boots it broadcasts big startup packet
	//Read it and dump it
	waitForSPI(); //Wait for assertion of INT before reading advert message.
	receivePacket();

	//The BNO080 will then transmit an unsolicited Initialize Response (see 6.4.5.2)
	//Read it and dump it
	waitForSPI(); //Wait for assertion of INT before reading Init response
	receivePacket();

	//Check communication with device
	shtpData[0] = SHTP_REPORT_PRODUCT_ID_REQUEST; //Request the product ID and reset info
	shtpData[1] = 0;							  //Reserved

	//Transmit packet on channel 2, 2 bytes
	sendPacket(CHANNEL_CONTROL, 2);

	//Now we wait for response
	waitForSPI();
	if (receivePacket() == true)
	{
		if (shtpData[0] == SHTP_REPORT_PRODUCT_ID_RESPONSE)
			return (true);
	}

	return (false); //Something went wrong
}

//Calling this function with nothing sets the debug port to Serial
//You can also call it with other streams like Serial1, SerialUSB, etc.
void BNO080::enableDebugging(Stream &debugPort)
{
	_debugPort = &debugPort;
	_printDebug = true;
}

//Updates the latest variables if possible
//Returns false if new readings are not available
bool BNO080::dataAvailable(void)
{
	//If we have an interrupt pin connection available, check if data is available.
	//If int pin is not set, then we'll rely on receivePacket() to timeout
	//See issue 13: https://github.com/sparkfun/SparkFun_BNO080_Arduino_Library/issues/13
	if (_int != 255)
	{
		if (digitalRead(_int) == HIGH)
			return (false);
	}

	if (receivePacket() == true)
	{
		//Check to see if this packet is a sensor reporting its data to us
		if (shtpHeader[2] == CHANNEL_REPORTS && shtpData[0] == SHTP_REPORT_BASE_TIMESTAMP)
		{
			parseInputReport(); //This will update the rawAccelX, etc variables depending on which feature report is found
			return (true);
		}
		else if (shtpHeader[2] == CHANNEL_CONTROL)
		{
			parseCommandReport(); //This will update responses to commands, calibrationStatus, etc.
			return (true);
		}
	}
	return (false);
}

//This function pulls the data from the command response report

//Unit responds with packet that contains the following:
//shtpHeader[0:3]: First, a 4 byte header
//shtpData[0]: The Report ID
//shtpData[1]: Sequence number (See 6.5.18.2)
//shtpData[2]: Command
//shtpData[3]: Command Sequence Number
//shtpData[4]: Response Sequence Number
//shtpData[5 + 0]: R0
//shtpData[5 + 1]: R1
//shtpData[5 + 2]: R2
//shtpData[5 + 3]: R3
//shtpData[5 + 4]: R4
//shtpData[5 + 5]: R5
//shtpData[5 + 6]: R6
//shtpData[5 + 7]: R7
//shtpData[5 + 8]: R8
void BNO080::parseCommandReport(void)
{
	if (shtpData[0] == SHTP_REPORT_COMMAND_RESPONSE)
	{
		//The BNO080 responds with this report to command requests. It's up to use to remember which command we issued.
		uint8_t command = shtpData[2]; //This is the Command byte of the response

		if (command == COMMAND_ME_CALIBRATE)
		{
			calibrationStatus = shtpData[5 + 0]; //R0 - Status (0 = success, non-zero = fail)
		}
	}
	else
	{
		//This sensor report ID is unhandled.
		//See reference manual to add additional feature reports as needed
	}

	//TODO additional feature reports may be strung together. Parse them all.
}

//This function pulls the data from the input report
//The input reports vary in length so this function stores the various 16-bit values as globals

//Unit responds with packet that contains the following:
//shtpHeader[0:3]: First, a 4 byte header
//shtpData[0:4]: Then a 5 byte timestamp of microsecond clicks since reading was taken
//shtpData[5 + 0]: Then a feature report ID (0x01 for Accel, 0x05 for Rotation Vector)
//shtpData[5 + 1]: Sequence number (See 6.5.18.2)
//shtpData[5 + 2]: Status
//shtpData[3]: Delay
//shtpData[4:5]: i/accel x/gyro x/etc
//shtpData[6:7]: j/accel y/gyro y/etc
//shtpData[8:9]: k/accel z/gyro z/etc
//shtpData[10:11]: real/gyro temp/etc
//shtpData[12:13]: Accuracy estimate
void BNO080::parseInputReport(void)
{
	//Calculate the number of data bytes in this packet
	int16_t dataLength = ((uint16_t)shtpHeader[1] << 8 | shtpHeader[0]);
	dataLength &= ~(1 << 15); //Clear the MSbit. This bit indicates if this package is a continuation of the last.
	//Ignore it for now. TODO catch this as an error and exit

	dataLength -= 4; //Remove the header bytes from the data count

	timeStamp = ((uint32_t)shtpData[4] << (8 * 3)) | ((uint32_t)shtpData[3] << (8 * 2)) | ((uint32_t)shtpData[2] << (8 * 1)) | ((uint32_t)shtpData[1] << (8 * 0));

	uint8_t status = shtpData[5 + 2] & 0x03; //Get status bits
	uint16_t data1 = (uint16_t)shtpData[5 + 5] << 8 | shtpData[5 + 4];
	uint16_t data2 = (uint16_t)shtpData[5 + 7] << 8 | shtpData[5 + 6];
	uint16_t data3 = (uint16_t)shtpData[5 + 9] << 8 | shtpData[5 + 8];
	uint16_t data4 = 0;
	uint16_t data5 = 0; //We would need to change this to uin32_t to capture time stamp value on Raw Accel/Gyro/Mag reports

	if (dataLength - 5 > 9)
	{
		data4 = (uint16_t)shtpData[5 + 11] << 8 | shtpData[5 + 10];
	}
	if (dataLength - 5 > 11)
	{
		data5 = (uint16_t)shtpData[5 + 13] << 8 | shtpData[5 + 12];
	}

	//Store these generic values to their proper global variable
	if (shtpData[5] == SENSOR_REPORTID_ACCELEROMETER)
	{
		accelAccuracy = status;
		rawAccelX = data1;
		rawAccelY = data2;
		rawAccelZ = data3;
	}
	else if (shtpData[5] == SENSOR_REPORTID_LINEAR_ACCELERATION)
	{
		accelLinAccuracy = status;
		rawLinAccelX = data1;
		rawLinAccelY = data2;
		rawLinAccelZ = data3;
	}
	else if (shtpData[5] == SENSOR_REPORTID_GYROSCOPE)
	{
		gyroAccuracy = status;
		rawGyroX = data1;
		rawGyroY = data2;
		rawGyroZ = data3;
	}
	else if (shtpData[5] == SENSOR_REPORTID_MAGNETIC_FIELD)
	{
		magAccuracy = status;
		rawMagX = data1;
		rawMagY = data2;
		rawMagZ = data3;
	}
	else if (shtpData[5] == SENSOR_REPORTID_ROTATION_VECTOR || shtpData[5] == SENSOR_REPORTID_GAME_ROTATION_VECTOR)
	{
		quatAccuracy = status;
		rawQuatI = data1;
		rawQuatJ = data2;
		rawQuatK = data3;
		rawQuatReal = data4;
		rawQuatRadianAccuracy = data5; //Only available on rotation vector, not game rot vector
	}
	else if (shtpData[5] == SENSOR_REPORTID_STEP_COUNTER)
	{
		stepCount = data3; //Bytes 8/9
	}
	else if (shtpData[5] == SENSOR_REPORTID_STABILITY_CLASSIFIER)
	{
		stabilityClassifier = shtpData[5 + 4]; //Byte 4 only
	}
	else if (shtpData[5] == SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER)
	{
		activityClassifier = shtpData[5 + 5]; //Most likely state

		//Load activity classification confidences into the array
		for (uint8_t x = 0; x < 9; x++)					   //Hardcoded to max of 9. TODO - bring in array size
			_activityConfidences[x] = shtpData[5 + 6 + x]; //5 bytes of timestamp, byte 6 is first confidence byte
	}
	else if (shtpData[5] == SENSOR_REPORTID_RAW_ACCELEROMETER)
	{
		memsRawAccelX = data1;
		memsRawAccelY = data2;
		memsRawAccelZ = data3;
	}
	else if (shtpData[5] == SENSOR_REPORTID_RAW_GYROSCOPE)
	{
		memsRawGyroX = data1;
		memsRawGyroY = data2;
		memsRawGyroZ = data3;
	}
	else if (shtpData[5] == SENSOR_REPORTID_RAW_MAGNETOMETER)
	{
		memsRawMagX = data1;
		memsRawMagY = data2;
		memsRawMagZ = data3;
	}
	else if (shtpData[5] == SHTP_REPORT_COMMAND_RESPONSE)
	{
		Serial.println("!");
		//The BNO080 responds with this report to command requests. It's up to use to remember which command we issued.
		uint8_t command = shtpData[5 + 2]; //This is the Command byte of the response

		if (command == COMMAND_ME_CALIBRATE)
		{
			Serial.println("ME Cal report found!");
			calibrationStatus = shtpData[5 + 5]; //R0 - Status (0 = success, non-zero = fail)
		}
	}
	else
	{
		//This sensor report ID is unhandled.
		//See reference manual to add additional feature reports as needed
	}

	//TODO additional feature reports may be strung together. Parse them all.
}

//Return the rotation vector quaternion I
float BNO080::getQuatI()
{
	float quat = qToFloat(rawQuatI, rotationVector_Q1);
	return (quat);
}

//Return the rotation vector quaternion J
float BNO080::getQuatJ()
{
	float quat = qToFloat(rawQuatJ, rotationVector_Q1);
	return (quat);
}

//Return the rotation vector quaternion K
float BNO080::getQuatK()
{
	float quat = qToFloat(rawQuatK, rotationVector_Q1);
	return (quat);
}

//Return the rotation vector quaternion Real
float BNO080::getQuatReal()
{
	float quat = qToFloat(rawQuatReal, rotationVector_Q1);
	return (quat);
}

//Return the rotation vector accuracy
float BNO080::getQuatRadianAccuracy()
{
	float quat = qToFloat(rawQuatRadianAccuracy, rotationVector_Q1);
	return (quat);
}

//Return the acceleration component
uint8_t BNO080::getQuatAccuracy()
{
	return (quatAccuracy);
}

//Return the acceleration component
float BNO080::getAccelX()
{
	float accel = qToFloat(rawAccelX, accelerometer_Q1);
	return (accel);
}

//Return the acceleration component
float BNO080::getAccelY()
{
	float accel = qToFloat(rawAccelY, accelerometer_Q1);
	return (accel);
}

//Return the acceleration component
float BNO080::getAccelZ()
{
	float accel = qToFloat(rawAccelZ, accelerometer_Q1);
	return (accel);
}

//Return the acceleration component
uint8_t BNO080::getAccelAccuracy()
{
	return (accelAccuracy);
}

// linear acceleration, i.e. minus gravity

//Return the acceleration component
float BNO080::getLinAccelX()
{
	float accel = qToFloat(rawLinAccelX, linear_accelerometer_Q1);
	return (accel);
}

//Return the acceleration component
float BNO080::getLinAccelY()
{
	float accel = qToFloat(rawLinAccelY, linear_accelerometer_Q1);
	return (accel);
}

//Return the acceleration component
float BNO080::getLinAccelZ()
{
	float accel = qToFloat(rawLinAccelZ, linear_accelerometer_Q1);
	return (accel);
}

//Return the acceleration component
uint8_t BNO080::getLinAccelAccuracy()
{
	return (accelLinAccuracy);
}

//Return the gyro component
float BNO080::getGyroX()
{
	float gyro = qToFloat(rawGyroX, gyro_Q1);
	return (gyro);
}

//Return the gyro component
float BNO080::getGyroY()
{
	float gyro = qToFloat(rawGyroY, gyro_Q1);
	return (gyro);
}

//Return the gyro component
float BNO080::getGyroZ()
{
	float gyro = qToFloat(rawGyroZ, gyro_Q1);
	return (gyro);
}

//Return the gyro component
uint8_t BNO080::getGyroAccuracy()
{
	return (gyroAccuracy);
}

//Return the magnetometer component
float BNO080::getMagX()
{
	float mag = qToFloat(rawMagX, magnetometer_Q1);
	return (mag);
}

//Return the magnetometer component
float BNO080::getMagY()
{
	float mag = qToFloat(rawMagY, magnetometer_Q1);
	return (mag);
}

//Return the magnetometer component
float BNO080::getMagZ()
{
	float mag = qToFloat(rawMagZ, magnetometer_Q1);
	return (mag);
}

//Return the mag component
uint8_t BNO080::getMagAccuracy()
{
	return (magAccuracy);
}

//Return the step count
uint16_t BNO080::getStepCount()
{
	return (stepCount);
}

//Return the stability classifier
uint8_t BNO080::getStabilityClassifier()
{
	return (stabilityClassifier);
}

//Return the activity classifier
uint8_t BNO080::getActivityClassifier()
{
	return (activityClassifier);
}

//Return the time stamp
uint32_t BNO080::getTimeStamp()
{
	return (timeStamp);
}

//Return raw mems value for the accel
int16_t BNO080::getRawAccelX()
{
	return (memsRawAccelX);
}
//Return raw mems value for the accel
int16_t BNO080::getRawAccelY()
{
	return (memsRawAccelY);
}
//Return raw mems value for the accel
int16_t BNO080::getRawAccelZ()
{
	return (memsRawAccelZ);
}

//Return raw mems value for the gyro
int16_t BNO080::getRawGyroX()
{
	return (memsRawGyroX);
}
int16_t BNO080::getRawGyroY()
{
	return (memsRawGyroY);
}
int16_t BNO080::getRawGyroZ()
{
	return (memsRawGyroZ);
}

//Return raw mems value for the mag
int16_t BNO080::getRawMagX()
{
	return (memsRawMagX);
}
int16_t BNO080::getRawMagY()
{
	return (memsRawMagY);
}
int16_t BNO080::getRawMagZ()
{
	return (memsRawMagZ);
}

//Given a record ID, read the Q1 value from the metaData record in the FRS (ya, it's complicated)
//Q1 is used for all sensor data calculations
int16_t BNO080::getQ1(uint16_t recordID)
{
	//Q1 is always the lower 16 bits of word 7
	uint16_t q = readFRSword(recordID, 7) & 0xFFFF; //Get word 7, lower 16 bits
	return (q);
}

//Given a record ID, read the Q2 value from the metaData record in the FRS
//Q2 is used in sensor bias
int16_t BNO080::getQ2(uint16_t recordID)
{
	//Q2 is always the upper 16 bits of word 7
	uint16_t q = readFRSword(recordID, 7) >> 16; //Get word 7, upper 16 bits
	return (q);
}

//Given a record ID, read the Q3 value from the metaData record in the FRS
//Q3 is used in sensor change sensitivity
int16_t BNO080::getQ3(uint16_t recordID)
{
	//Q3 is always the upper 16 bits of word 8
	uint16_t q = readFRSword(recordID, 8) >> 16; //Get word 8, upper 16 bits
	return (q);
}

//Given a record ID, read the resolution value from the metaData record in the FRS for a given sensor
float BNO080::getResolution(uint16_t recordID)
{
	//The resolution Q value are 'the same as those used in the sensor's input report'
	//This should be Q1.
	int16_t Q = getQ1(recordID);

	//Resolution is always word 2
	uint32_t value = readFRSword(recordID, 2); //Get word 2

	float resolution = qToFloat(value, Q);

	return (resolution);
}

//Given a record ID, read the range value from the metaData record in the FRS for a given sensor
float BNO080::getRange(uint16_t recordID)
{
	//The resolution Q value are 'the same as those used in the sensor's input report'
	//This should be Q1.
	int16_t Q = getQ1(recordID);

	//Range is always word 1
	uint32_t value = readFRSword(recordID, 1); //Get word 1

	float range = qToFloat(value, Q);

	return (range);
}

//Given a record ID and a word number, look up the word data
//Helpful for pulling out a Q value, range, etc.
//Use readFRSdata for pulling out multi-word objects for a sensor (Vendor data for example)
uint32_t BNO080::readFRSword(uint16_t recordID, uint8_t wordNumber)
{
	if (readFRSdata(recordID, wordNumber, 1) == true) //Get word number, just one word in length from FRS
		return (metaData[0]);						  //Return this one word

	return (0); //Error
}

//Ask the sensor for data from the Flash Record System
//See 6.3.6 page 40, FRS Read Request
void BNO080::frsReadRequest(uint16_t recordID, uint16_t readOffset, uint16_t blockSize)
{
	shtpData[0] = SHTP_REPORT_FRS_READ_REQUEST; //FRS Read Request
	shtpData[1] = 0;							//Reserved
	shtpData[2] = (readOffset >> 0) & 0xFF;		//Read Offset LSB
	shtpData[3] = (readOffset >> 8) & 0xFF;		//Read Offset MSB
	shtpData[4] = (recordID >> 0) & 0xFF;		//FRS Type LSB
	shtpData[5] = (recordID >> 8) & 0xFF;		//FRS Type MSB
	shtpData[6] = (blockSize >> 0) & 0xFF;		//Block size LSB
	shtpData[7] = (blockSize >> 8) & 0xFF;		//Block size MSB

	//Transmit packet on channel 2, 8 bytes
	sendPacket(CHANNEL_CONTROL, 8);
}

//Given a sensor or record ID, and a given start/stop bytes, read the data from the Flash Record System (FRS) for this sensor
//Returns true if metaData array is loaded successfully
//Returns false if failure
bool BNO080::readFRSdata(uint16_t recordID, uint8_t startLocation, uint8_t wordsToRead)
{
	uint8_t spot = 0;

	//First we send a Flash Record System (FRS) request
	frsReadRequest(recordID, startLocation, wordsToRead); //From startLocation of record, read a # of words

	//Read bytes until FRS reports that the read is complete
	while (1)
	{
		//Now we wait for response
		while (1)
		{
			uint8_t counter = 0;
			while (receivePacket() == false)
			{
				if (counter++ > 100)
					return (false); //Give up
				delay(1);
			}

			//We have the packet, inspect it for the right contents
			//See page 40. Report ID should be 0xF3 and the FRS types should match the thing we requested
			if (shtpData[0] == SHTP_REPORT_FRS_READ_RESPONSE)
				if (((uint16_t)shtpData[13] << 8 | shtpData[12]) == recordID)
					break; //This packet is one we are looking for
		}

		uint8_t dataLength = shtpData[1] >> 4;
		uint8_t frsStatus = shtpData[1] & 0x0F;

		uint32_t data0 = (uint32_t)shtpData[7] << 24 | (uint32_t)shtpData[6] << 16 | (uint32_t)shtpData[5] << 8 | (uint32_t)shtpData[4];
		uint32_t data1 = (uint32_t)shtpData[11] << 24 | (uint32_t)shtpData[10] << 16 | (uint32_t)shtpData[9] << 8 | (uint32_t)shtpData[8];

		//Record these words to the metaData array
		if (dataLength > 0)
		{
			metaData[spot++] = data0;
		}
		if (dataLength > 1)
		{
			metaData[spot++] = data1;
		}

		if (spot >= MAX_METADATA_SIZE)
		{
			if (_printDebug == true)
				_debugPort->println(F("metaData array over run. Returning."));
			return (true); //We have run out of space in our array. Bail.
		}

		if (frsStatus == 3 || frsStatus == 6 || frsStatus == 7)
		{
			return (true); //FRS status is read completed! We're done!
		}
	}
}

//Send command to reset IC
//Read all advertisement packets from sensor
//The sensor has been seen to reset twice if we attempt too much too quickly.
//This seems to work reliably.
void BNO080::softReset(void)
{
	shtpData[0] = 1; //Reset

	//Attempt to start communication with sensor
	sendPacket(CHANNEL_EXECUTABLE, 1); //Transmit packet on channel 1, 1 byte

	//Read all incoming data and flush it
	delay(50);
	while (receivePacket() == true)
		;
	delay(50);
	while (receivePacket() == true)
		;
}

//Get the reason for the last reset
//1 = POR, 2 = Internal reset, 3 = Watchdog, 4 = External reset, 5 = Other
uint8_t BNO080::resetReason()
{
	shtpData[0] = SHTP_REPORT_PRODUCT_ID_REQUEST; //Request the product ID and reset info
	shtpData[1] = 0;							  //Reserved

	//Transmit packet on channel 2, 2 bytes
	sendPacket(CHANNEL_CONTROL, 2);

	//Now we wait for response
	if (receivePacket() == true)
	{
		if (shtpData[0] == SHTP_REPORT_PRODUCT_ID_RESPONSE)
		{
			return (shtpData[1]);
		}
	}

	return (0);
}

//Given a register value and a Q point, convert to float
//See https://en.wikipedia.org/wiki/Q_(number_format)
float BNO080::qToFloat(int16_t fixedPointValue, uint8_t qPoint)
{
	float qFloat = fixedPointValue;
	qFloat *= pow(2, qPoint * -1);
	return (qFloat);
}

//Sends the packet to enable the rotation vector
void BNO080::enableRotationVector(uint16_t timeBetweenReports)
{
	setFeatureCommand(SENSOR_REPORTID_ROTATION_VECTOR, timeBetweenReports);
}

//Sends the packet to enable the rotation vector
void BNO080::enableGameRotationVector(uint16_t timeBetweenReports)
{
	setFeatureCommand(SENSOR_REPORTID_GAME_ROTATION_VECTOR, timeBetweenReports);
}

//Sends the packet to enable the accelerometer
void BNO080::enableAccelerometer(uint16_t timeBetweenReports)
{
	setFeatureCommand(SENSOR_REPORTID_ACCELEROMETER, timeBetweenReports);
}

//Sends the packet to enable the accelerometer
void BNO080::enableLinearAccelerometer(uint16_t timeBetweenReports)
{
	setFeatureCommand(SENSOR_REPORTID_LINEAR_ACCELERATION, timeBetweenReports);
}

//Sends the packet to enable the gyro
void BNO080::enableGyro(uint16_t timeBetweenReports)
{
	setFeatureCommand(SENSOR_REPORTID_GYROSCOPE, timeBetweenReports);
}

//Sends the packet to enable the magnetometer
void BNO080::enableMagnetometer(uint16_t timeBetweenReports)
{
	setFeatureCommand(SENSOR_REPORTID_MAGNETIC_FIELD, timeBetweenReports);
}

//Sends the packet to enable the step counter
void BNO080::enableStepCounter(uint16_t timeBetweenReports)
{
	setFeatureCommand(SENSOR_REPORTID_STEP_COUNTER, timeBetweenReports);
}

//Sends the packet to enable the Stability Classifier
void BNO080::enableStabilityClassifier(uint16_t timeBetweenReports)
{
	setFeatureCommand(SENSOR_REPORTID_STABILITY_CLASSIFIER, timeBetweenReports);
}

//Sends the packet to enable the raw accel readings
//Note you must enable basic reporting on the sensor as well
void BNO080::enableRawAccelerometer(uint16_t timeBetweenReports)
{
	setFeatureCommand(SENSOR_REPORTID_RAW_ACCELEROMETER, timeBetweenReports);
}

//Sends the packet to enable the raw accel readings
//Note you must enable basic reporting on the sensor as well
void BNO080::enableRawGyro(uint16_t timeBetweenReports)
{
	setFeatureCommand(SENSOR_REPORTID_RAW_GYROSCOPE, timeBetweenReports);
}

//Sends the packet to enable the raw accel readings
//Note you must enable basic reporting on the sensor as well
void BNO080::enableRawMagnetometer(uint16_t timeBetweenReports)
{
	setFeatureCommand(SENSOR_REPORTID_RAW_MAGNETOMETER, timeBetweenReports);
}

//Sends the packet to enable the various activity classifiers
void BNO080::enableActivityClassifier(uint16_t timeBetweenReports, uint32_t activitiesToEnable, uint8_t (&activityConfidences)[9])
{
	_activityConfidences = activityConfidences; //Store pointer to array

	setFeatureCommand(SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER, timeBetweenReports, activitiesToEnable);
}

//Sends the commands to begin calibration of the accelerometer
void BNO080::calibrateAccelerometer()
{
	sendCalibrateCommand(CALIBRATE_ACCEL);
}

//Sends the commands to begin calibration of the gyro
void BNO080::calibrateGyro()
{
	sendCalibrateCommand(CALIBRATE_GYRO);
}

//Sends the commands to begin calibration of the magnetometer
void BNO080::calibrateMagnetometer()
{
	sendCalibrateCommand(CALIBRATE_MAG);
}

//Sends the commands to begin calibration of the planar accelerometer
void BNO080::calibratePlanarAccelerometer()
{
	sendCalibrateCommand(CALIBRATE_PLANAR_ACCEL);
}

//See 2.2 of the Calibration Procedure document 1000-4044
void BNO080::calibrateAll()
{
	sendCalibrateCommand(CALIBRATE_ACCEL_GYRO_MAG);
}

void BNO080::endCalibration()
{
	sendCalibrateCommand(CALIBRATE_STOP); //Disables all calibrations
}

//See page 51 of reference manual - ME Calibration Response
//Byte 5 is parsed during the readPacket and stored in calibrationStatus
boolean BNO080::calibrationComplete()
{
	if (calibrationStatus == 0)
		return (true);
	return (false);
}

//Given a sensor's report ID, this tells the BNO080 to begin reporting the values
void BNO080::setFeatureCommand(uint8_t reportID, uint16_t timeBetweenReports)
{
	setFeatureCommand(reportID, timeBetweenReports, 0); //No specific config
}

//Given a sensor's report ID, this tells the BNO080 to begin reporting the values
//Also sets the specific config word. Useful for personal activity classifier
void BNO080::setFeatureCommand(uint8_t reportID, uint16_t timeBetweenReports, uint32_t specificConfig)
{
	long microsBetweenReports = (long)timeBetweenReports * 1000L;

	shtpData[0] = SHTP_REPORT_SET_FEATURE_COMMAND;	 //Set feature command. Reference page 55
	shtpData[1] = reportID;							   //Feature Report ID. 0x01 = Accelerometer, 0x05 = Rotation vector
	shtpData[2] = 0;								   //Feature flags
	shtpData[3] = 0;								   //Change sensitivity (LSB)
	shtpData[4] = 0;								   //Change sensitivity (MSB)
	shtpData[5] = (microsBetweenReports >> 0) & 0xFF;  //Report interval (LSB) in microseconds. 0x7A120 = 500ms
	shtpData[6] = (microsBetweenReports >> 8) & 0xFF;  //Report interval
	shtpData[7] = (microsBetweenReports >> 16) & 0xFF; //Report interval
	shtpData[8] = (microsBetweenReports >> 24) & 0xFF; //Report interval (MSB)
	shtpData[9] = 0;								   //Batch Interval (LSB)
	shtpData[10] = 0;								   //Batch Interval
	shtpData[11] = 0;								   //Batch Interval
	shtpData[12] = 0;								   //Batch Interval (MSB)
	shtpData[13] = (specificConfig >> 0) & 0xFF;	   //Sensor-specific config (LSB)
	shtpData[14] = (specificConfig >> 8) & 0xFF;	   //Sensor-specific config
	shtpData[15] = (specificConfig >> 16) & 0xFF;	  //Sensor-specific config
	shtpData[16] = (specificConfig >> 24) & 0xFF;	  //Sensor-specific config (MSB)

	//Transmit packet on channel 2, 17 bytes
	sendPacket(CHANNEL_CONTROL, 17);
}

//Tell the sensor to do a command
//See 6.3.8 page 41, Command request
//The caller is expected to set P0 through P8 prior to calling
void BNO080::sendCommand(uint8_t command)
{
	shtpData[0] = SHTP_REPORT_COMMAND_REQUEST; //Command Request
	shtpData[1] = commandSequenceNumber++;	 //Increments automatically each function call
	shtpData[2] = command;					   //Command

	//Caller must set these
	/*shtpData[3] = 0; //P0
	shtpData[4] = 0; //P1
	shtpData[5] = 0; //P2
	shtpData[6] = 0;
	shtpData[7] = 0;
	shtpData[8] = 0;
	shtpData[9] = 0;
	shtpData[10] = 0;
	shtpData[11] = 0;*/

	//Transmit packet on channel 2, 12 bytes
	sendPacket(CHANNEL_CONTROL, 12);
}

//This tells the BNO080 to begin calibrating
//See page 50 of reference manual and the 1000-4044 calibration doc
void BNO080::sendCalibrateCommand(uint8_t thingToCalibrate)
{
	/*shtpData[3] = 0; //P0 - Accel Cal Enable
	shtpData[4] = 0; //P1 - Gyro Cal Enable
	shtpData[5] = 0; //P2 - Mag Cal Enable
	shtpData[6] = 0; //P3 - Subcommand 0x00
	shtpData[7] = 0; //P4 - Planar Accel Cal Enable
	shtpData[8] = 0; //P5 - Reserved
	shtpData[9] = 0; //P6 - Reserved
	shtpData[10] = 0; //P7 - Reserved
	shtpData[11] = 0; //P8 - Reserved*/

	for (uint8_t x = 3; x < 12; x++) //Clear this section of the shtpData array
		shtpData[x] = 0;

	if (thingToCalibrate == CALIBRATE_ACCEL)
		shtpData[3] = 1;
	else if (thingToCalibrate == CALIBRATE_GYRO)
		shtpData[4] = 1;
	else if (thingToCalibrate == CALIBRATE_MAG)
		shtpData[5] = 1;
	else if (thingToCalibrate == CALIBRATE_PLANAR_ACCEL)
		shtpData[7] = 1;
	else if (thingToCalibrate == CALIBRATE_ACCEL_GYRO_MAG)
	{
		shtpData[3] = 1;
		shtpData[4] = 1;
		shtpData[5] = 1;
	}
	else if (thingToCalibrate == CALIBRATE_STOP)
		; //Do nothing, bytes are set to zero

	//Make the internal calStatus variable non-zero (operation failed) so that user can test while we wait
	calibrationStatus = 1;

	//Using this shtpData packet, send a command
	sendCommand(COMMAND_ME_CALIBRATE);
}

//Request ME Calibration Status from BNO080
//See page 51 of reference manual
void BNO080::requestCalibrationStatus()
{
	/*shtpData[3] = 0; //P0 - Reserved
	shtpData[4] = 0; //P1 - Reserved
	shtpData[5] = 0; //P2 - Reserved
	shtpData[6] = 0; //P3 - 0x01 - Subcommand: Get ME Calibration
	shtpData[7] = 0; //P4 - Reserved
	shtpData[8] = 0; //P5 - Reserved
	shtpData[9] = 0; //P6 - Reserved
	shtpData[10] = 0; //P7 - Reserved
	shtpData[11] = 0; //P8 - Reserved*/

	for (uint8_t x = 3; x < 12; x++) //Clear this section of the shtpData array
		shtpData[x] = 0;

	shtpData[6] = 0x01; //P3 - 0x01 - Subcommand: Get ME Calibration

	//Using this shtpData packet, send a command
	sendCommand(COMMAND_ME_CALIBRATE);
}

//This tells the BNO080 to save the Dynamic Calibration Data (DCD) to flash
//See page 49 of reference manual and the 1000-4044 calibration doc
void BNO080::saveCalibration()
{
	/*shtpData[3] = 0; //P0 - Reserved
	shtpData[4] = 0; //P1 - Reserved
	shtpData[5] = 0; //P2 - Reserved
	shtpData[6] = 0; //P3 - Reserved
	shtpData[7] = 0; //P4 - Reserved
	shtpData[8] = 0; //P5 - Reserved
	shtpData[9] = 0; //P6 - Reserved
	shtpData[10] = 0; //P7 - Reserved
	shtpData[11] = 0; //P8 - Reserved*/

	for (uint8_t x = 3; x < 12; x++) //Clear this section of the shtpData array
		shtpData[x] = 0;

	//Using this shtpData packet, send a command
	sendCommand(COMMAND_DCD); //Save DCD command
}

//Wait a certain time for incoming I2C bytes before giving up
//Returns false if failed
boolean BNO080::waitForI2C()
{
	for (uint8_t counter = 0; counter < 100; counter++) //Don't got more than 255
	{
		if (_i2cPort->available() > 0)
			return (true);
		delay(1);
	}

	if (_printDebug == true)
		_debugPort->println(F("I2C timeout"));
	return (false);
}

//Blocking wait for BNO080 to assert (pull low) the INT pin
//indicating it's ready for comm. Can take more than 104ms
//after a hardware reset
boolean BNO080::waitForSPI()
{
	for (uint8_t counter = 0; counter < 125; counter++) //Don't got more than 255
	{
		if (digitalRead(_int) == LOW)
			return (true);
		if (_printDebug == true)
			_debugPort->println(F("SPI Wait"));
		delay(1);
	}

	if (_printDebug == true)
		_debugPort->println(F("SPI INT timeout"));
	return (false);
}

//Check to see if there is any new data available
//Read the contents of the incoming packet into the shtpData array
boolean BNO080::receivePacket(void)
{
	if (_i2cPort == NULL) //Do SPI
	{
		if (digitalRead(_int) == HIGH)
			return (false); //Data is not available

		//Old way: if (waitForSPI() == false) return (false); //Something went wrong

		//Get first four bytes to find out how much data we need to read

		_spiPort->beginTransaction(SPISettings(_spiPortSpeed, MSBFIRST, SPI_MODE3));
		digitalWrite(_cs, LOW);

		//Get the first four bytes, aka the packet header
		uint8_t packetLSB = _spiPort->transfer(0);
		uint8_t packetMSB = _spiPort->transfer(0);
		uint8_t channelNumber = _spiPort->transfer(0);
		uint8_t sequenceNumber = _spiPort->transfer(0); //Not sure if we need to store this or not

		//Store the header info
		shtpHeader[0] = packetLSB;
		shtpHeader[1] = packetMSB;
		shtpHeader[2] = channelNumber;
		shtpHeader[3] = sequenceNumber;

		//Calculate the number of data bytes in this packet
		uint16_t dataLength = ((uint16_t)packetMSB << 8 | packetLSB);
		dataLength &= ~(1 << 15); //Clear the MSbit.
		//This bit indicates if this package is a continuation of the last. Ignore it for now.
		//TODO catch this as an error and exit
		if (dataLength == 0)
		{
			//Packet is empty
			return (false); //All done
		}
		dataLength -= 4; //Remove the header bytes from the data count

		//Read incoming data into the shtpData array
		for (uint16_t dataSpot = 0; dataSpot < dataLength; dataSpot++)
		{
			uint8_t incoming = _spiPort->transfer(0xFF);
			if (dataSpot < MAX_PACKET_SIZE)	//BNO080 can respond with upto 270 bytes, avoid overflow
				shtpData[dataSpot] = incoming; //Store data into the shtpData array
		}

		digitalWrite(_cs, HIGH); //Release BNO080
		_spiPort->endTransaction();
	}
	else //Do I2C
	{
		_i2cPort->requestFrom((uint8_t)_deviceAddress, (uint8_t)4); //Ask for four bytes to find out how much data we need to read
		if (waitForI2C() == false)
			return (false); //Error

		//Get the first four bytes, aka the packet header
		uint8_t packetLSB = _i2cPort->read();
		uint8_t packetMSB = _i2cPort->read();
		uint8_t channelNumber = _i2cPort->read();
		uint8_t sequenceNumber = _i2cPort->read(); //Not sure if we need to store this or not

		//Store the header info.
		shtpHeader[0] = packetLSB;
		shtpHeader[1] = packetMSB;
		shtpHeader[2] = channelNumber;
		shtpHeader[3] = sequenceNumber;

		//Calculate the number of data bytes in this packet
		int16_t dataLength = ((uint16_t)packetMSB << 8 | packetLSB);
		dataLength &= ~(1 << 15); //Clear the MSbit.
		//This bit indicates if this package is a continuation of the last. Ignore it for now.
		//TODO catch this as an error and exit
		if (dataLength == 0)
		{
			//Packet is empty
			return (false); //All done
		}
		dataLength -= 4; //Remove the header bytes from the data count

		getData(dataLength);
	}

	return (true); //We're done!
}

//Sends multiple requests to sensor until all data bytes are received from sensor
//The shtpData buffer has max capacity of MAX_PACKET_SIZE. Any bytes over this amount will be lost.
//Arduino I2C read limit is 32 bytes. Header is 4 bytes, so max data we can read per interation is 28 bytes
boolean BNO080::getData(uint16_t bytesRemaining)
{
	uint16_t dataSpot = 0; //Start at the beginning of shtpData array

	//Setup a series of chunked 32 byte reads
	while (bytesRemaining > 0)
	{
		uint16_t numberOfBytesToRead = bytesRemaining;
		if (numberOfBytesToRead > (I2C_BUFFER_LENGTH - 4))
			numberOfBytesToRead = (I2C_BUFFER_LENGTH - 4);

		_i2cPort->requestFrom((uint8_t)_deviceAddress, (uint8_t)(numberOfBytesToRead + 4));
		if (waitForI2C() == false)
			return (0); //Error

		//The first four bytes are header bytes and are throw away
		_i2cPort->read();
		_i2cPort->read();
		_i2cPort->read();
		_i2cPort->read();

		for (uint8_t x = 0; x < numberOfBytesToRead; x++)
		{
			uint8_t incoming = _i2cPort->read();
			if (dataSpot < MAX_PACKET_SIZE)
			{
				shtpData[dataSpot++] = incoming; //Store data into the shtpData array
			}
			else
			{
				//Do nothing with the data
			}
		}

		bytesRemaining -= numberOfBytesToRead;
	}
	return (true); //Done!
}

//Given the data packet, send the header then the data
//Returns false if sensor does not ACK
//TODO - Arduino has a max 32 byte send. Break sending into multi packets if needed.
boolean BNO080::sendPacket(uint8_t channelNumber, uint8_t dataLength)
{
	uint8_t packetLength = dataLength + 4; //Add four bytes for the header

	if (_i2cPort == NULL) //Do SPI
	{
		//Wait for BNO080 to indicate it is available for communication
		if (waitForSPI() == false)
			return (false); //Something went wrong

		//BNO080 has max CLK of 3MHz, MSB first,
		//The BNO080 uses CPOL = 1 and CPHA = 1. This is mode3
		_spiPort->beginTransaction(SPISettings(_spiPortSpeed, MSBFIRST, SPI_MODE3));
		digitalWrite(_cs, LOW);

		//Send the 4 byte packet header
		_spiPort->transfer(packetLength & 0xFF);			 //Packet length LSB
		_spiPort->transfer(packetLength >> 8);				 //Packet length MSB
		_spiPort->transfer(channelNumber);					 //Channel number
		_spiPort->transfer(sequenceNumber[channelNumber]++); //Send the sequence number, increments with each packet sent, different counter for each channel

		//Send the user's data packet
		for (uint8_t i = 0; i < dataLength; i++)
		{
			_spiPort->transfer(shtpData[i]);
		}

		digitalWrite(_cs, HIGH);
		_spiPort->endTransaction();
	}
	else //Do I2C
	{
		//if(packetLength > I2C_BUFFER_LENGTH) return(false); //You are trying to send too much. Break into smaller packets.

		_i2cPort->beginTransmission(_deviceAddress);

		//Send the 4 byte packet header
		_i2cPort->write(packetLength & 0xFF);			  //Packet length LSB
		_i2cPort->write(packetLength >> 8);				  //Packet length MSB
		_i2cPort->write(channelNumber);					  //Channel number
		_i2cPort->write(sequenceNumber[channelNumber]++); //Send the sequence number, increments with each packet sent, different counter for each channel

		//Send the user's data packet
		for (uint8_t i = 0; i < dataLength; i++)
		{
			_i2cPort->write(shtpData[i]);
		}
		if (_i2cPort->endTransmission() != 0)
		{
			return (false);
		}
	}

	return (true);
}

//Pretty prints the contents of the current shtp header and data packets
void BNO080::printPacket(void)
{
	if (_printDebug == true)
	{
		uint16_t packetLength = (uint16_t)shtpHeader[1] << 8 | shtpHeader[0];

		//Print the four byte header
		_debugPort->print(F("Header:"));
		for (uint8_t x = 0; x < 4; x++)
		{
			_debugPort->print(F(" "));
			if (shtpHeader[x] < 0x10)
				_debugPort->print(F("0"));
			_debugPort->print(shtpHeader[x], HEX);
		}

		uint8_t printLength = packetLength - 4;
		if (printLength > 40)
			printLength = 40; //Artificial limit. We don't want the phone book.

		_debugPort->print(F(" Body:"));
		for (uint8_t x = 0; x < printLength; x++)
		{
			_debugPort->print(F(" "));
			if (shtpData[x] < 0x10)
				_debugPort->print(F("0"));
			_debugPort->print(shtpData[x], HEX);
		}

		if (packetLength & 1 << 15)
		{
			_debugPort->println(F(" [Continued packet] "));
			packetLength &= ~(1 << 15);
		}

		_debugPort->print(F(" Length:"));
		_debugPort->print(packetLength);

		_debugPort->print(F(" Channel:"));
		if (shtpHeader[2] == 0)
			_debugPort->print(F("Command"));
		else if (shtpHeader[2] == 1)
			_debugPort->print(F("Executable"));
		else if (shtpHeader[2] == 2)
			_debugPort->print(F("Control"));
		else if (shtpHeader[2] == 3)
			_debugPort->print(F("Sensor-report"));
		else if (shtpHeader[2] == 4)
			_debugPort->print(F("Wake-report"));
		else if (shtpHeader[2] == 5)
			_debugPort->print(F("Gyro-vector"));
		else
			_debugPort->print(shtpHeader[2]);

		_debugPort->println();
	}
}
