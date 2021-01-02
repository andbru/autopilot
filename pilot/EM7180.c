
#include "EM7180.h"

// This c file serves only one EM7180 
// so file and device address are globals
int file = 0;
int adr = 0;


void initEM7180() {
	
	// Open EM7180 for communication
	file = open("/dev/i2c-3", O_RDWR);
	ioctl(file, I2C_SLAVE, EM7180_ADDRESS);

	
	// Initialize EM7180
    // Check SENtral status, make sure EEPROM upload of firmware was accomplished
    char STAT = (readByte(EM7180_SentralStatus) & 0x01); 
    int count = 0;
    while(!STAT) {
        writeByte(EM7180_ResetRequest, 0x01);
        sleep(1);  
        count++;  
        STAT = (readByte(EM7180_SentralStatus) & 0x01); 
        if(count > 10) {
			printf("EM7180 EEPROM not loaded!!!!!!!\n");
			break;
		}
    }
	
	// Enter EM7180 initialized state
    writeByte(EM7180_HostControl, 0x00);     // set SENtral in initialized state to configure registers
    writeByte(EM7180_PassThruControl,0x00);  // make sure pass through mode is off
    writeByte(EM7180_HostControl, 0x01);     // Force initialize
    writeByte(EM7180_HostControl, 0x00);     // set SENtral in initialized state to configure registers
    
    //Setup LPF bandwidth (BEFORE setting ODR's)
    writeByte(EM7180_ACC_LPF_BW, 0x03);      // 41Hz
    writeByte(EM7180_GYRO_LPF_BW, 0x03);     // 41Hz
    
    // Set accel/gyro/mage desired ODR rates
    writeByte(EM7180_QRateDivisor, 0x02);    // 100 Hz
    writeByte(EM7180_MagRate, 0x64);         // 100 Hz
    writeByte(EM7180_AccelRate, 0x14);       // 200/10 Hz
    writeByte(EM7180_GyroRate, 0x14);        // 200/10 Hz

    // Configure operating mode
    writeByte(EM7180_AlgorithmControl, 0x00); // read scale sensor data
    
    // Enable interrupt to host upon certain events
    //writeByte(EM7180_EnableEvents, 0x07);
    
    // Enable EM7180 run mode
    writeByte(EM7180_HostControl, 0x01);      // set SENtral in normal run mode
    sleep(1);
    
    //Disable stillness mode
    EM7180_set_integer_param (0x49, 0x00);
    
    //Write desired sensor full scale ranges to the EM7180
    EM7180_set_mag_acc_FS (0x3E8, 0x08); // 1000 uT, 8 g
    EM7180_set_gyro_FS (0x7D0); // 2000 dps
    
    // Put the Sentral in pass-thru mode
    WS_PassThroughMode();
    
    // Fetch the WarmStart data from the M24512DFM I2C EEPROM
    readSenParams();
    
    /*
    // Print WS matrix
    for (int paramnum = 0; paramnum < 35; paramnum++) {// 35 parameters
		for (int i= 0; i < 4; i++) {
			printf("0x%02x ", WS_params.Sen_param[paramnum][i]);
		}
		printf("\n");
	}
	printf("\n");
    */
    
    // Take Sentral out of pass-thru mode and re-start algorithm
    WS_Resume();
    usleep(500 * 1000);

    EM7180_set_WS_params();
    
    /*
    usleep(500 * 1000);
    EM7180_set_WS_params();
    // Print WS matrix
    for (int paramnum = 0; paramnum < 35; paramnum++) {// 35 parameters
		for (int i= 0; i < 4; i++) {
			printf("0x%02x ", WS_params.Sen_param[paramnum][i]);
		}
		printf("\n");
	}
	printf("\n");
    */
    

	printf("%s \n", "Init EM7180 OK!");

// ******************  end of Init EM7180  **********************
}


int pollEM7180(float *yaw, float *w) {
	
	short int gyroCount[3];// Stores the 16-bit signed gyro sensor output
	float Quat[4] = {0, 0, 0, 0}; 	// quaternion data register
	float gz = 0;		// Gyro rate
	float gzLp = 0;		// Low pass filtered
	float kgz = 1;		// Filter constant
	float Yaw;
	int newData = 0;	

	// Check event status register, way to chech data ready by polling rather than interrupt
	char eventStatus = readByte(EM7180_EventStatus); // reading clears the register

	// Check for errors
	if(eventStatus & 0x02) { // error detected, what is it?
		char errorStatus = readByte(EM7180_ErrorRegister);
		if(errorStatus != 0x00) { // non-zero value indicates error, what is it?
			printf(" EM7180 sensor status = "); printf("0x%02x \n", errorStatus);
			if(errorStatus == 0x11) printf("Magnetometer failure!\n");
			if(errorStatus == 0x12) printf("Accelerometer failure!\n");
			if(errorStatus == 0x14) printf("Gyro failure\n!");
			if(errorStatus == 0x21) printf("Magnetometer initialization failure!\n");
			if(errorStatus == 0x22) printf("Accelerometer initialization failure!\n");
			if(errorStatus == 0x24) printf("Gyro initialization failure!\n");
			if(errorStatus == 0x30) printf("Math error!\n");
			if(errorStatus == 0x80) printf("Invalid sample rate!\n");
		}
	// Handle errors ToDo
	}

	if(eventStatus & 0x20) { // new gyro data available
		readSENtralGyroData(gyroCount);      
		// Calculate the gyro value into actual dps   
		gz = (float)gyroCount[2]*0.153;
		*w = gz;
		gzLp = kgz * gz + (1 - kgz) * gzLp;     // Low pass filter angle rate
		//printf("gz ready: %f \n", gz);
		newData += 1;				 
	}

	if(eventStatus & 0x04) { // new quaternion data available
	readSENtralQuatData(Quat);
	//printf("Q ready: %f  loop: %d\n", Quat[0], loop);

	// Calculate Yaw
	Yaw   = atan2(2.0f * (Quat[0] * Quat[1] + Quat[3] * Quat[2]), Quat[3] * Quat[3] + Quat[0] * Quat[0] - Quat[1] * Quat[1] - Quat[2] * Quat[2]);   
	Yaw   *= 180.0f / M_PI; 
	//Yaw   += 6.3f; // Declination at Dalarö, Sweden
	if(Yaw < 0) Yaw   += 360.0f ; // Ensure yaw stays between 0 and 360
	
	float newYaw = yawDev(Yaw);
	
	// *yaw = newYaw;
	*yaw = Yaw;
	
	newData += 2;

	//printf("q ready: %f gz: %f loop: %d\n", Yaw, gz, loop);
	}
	
	return newData;
}


///////////////////////////////////////////////////////////////
//////		Functions                               ///////////
///////////////////////////////////////////////////////////////

float yawDev(float yaw) {
	
	// Deviation table to interpolate in
	float emY[14] = { -20, 7.8, 35.5, 59, 83.5, 108, 133, 160.5, 193, 231, 270, 309, 340, 367.8};
	float dev_emY[14] = { 10, 7.8, 5.5, -1, -6.5, -12, -17, -19.5, -17, -9, 0, 9, 10, 7.8};
	float kemY[14] = { -0.07914, -0.08303, -0.2766, -0.22449, -0.22449, -0.2, -0.09091, 0.076923, 0.210526, 0.230769, 0.230769, 0.032258, -0.07914, 0};
	
	int i = 0;
	while(emY[i] <= yaw) i++;
	i--;
	
	float newYaw = yaw - dev_emY[i] - (yaw - emY[i]) * kemY[i];
	
	if(newYaw < 0) newYaw += 360.0f;
	if(newYaw > 360) newYaw -= 360.0f;
	
	//printf("%f %f %d\n", newYaw, yaw, i);
	
	return newYaw;
}

void writeByte(char subAddress, char byte) {
	char buf[2] = {0};			// Buffer for write
	buf[0] = subAddress;
	buf[1] = byte;  
    write(file, buf, 2);		// Send slave register address and data
}

char readByte(char subAddress) {
	char buf[1] = {0};				// Buffer for read and write
	buf[0] = subAddress; 
    write(file, buf, 1);			// Send slave register address
    read(file, buf, 1);				// Read
    return buf[0];					// Return data read
}

void readBytes(char subAddress, char count, char *dest) {
	char abuf[1] = {0};					// Buffer for write
	abuf[0] = subAddress; 
    write(file, abuf, 1);				// Send slave register address
    char rbuf[100] = {0};				// Buffer for read
    read(file, rbuf, count);			// Read
    for(int i = 0; i < count; i++) {	// Return data read
		dest[i] = rbuf[i];
	}
}

void EM7180_set_integer_param (char param, unsigned int param_val) {
    char bytes[4], STAT;
    bytes[0] = param_val & (0xFF);
    bytes[1] = (param_val >> 8) & (0xFF);
    bytes[2] = (param_val >> 16) & (0xFF);
    bytes[3] = (param_val >> 24) & (0xFF);
    param = param | 0x80; //Parameter is the decimal value with the MSB set high to indicate a paramter write processs
    writeByte(EM7180_LoadParamByte0, bytes[0]); //Param LSB
    writeByte(EM7180_LoadParamByte1, bytes[1]);
    writeByte(EM7180_LoadParamByte2, bytes[2]);
    writeByte(EM7180_LoadParamByte3, bytes[3]); //Param MSB
    writeByte(EM7180_ParamRequest, param);
    writeByte(EM7180_AlgorithmControl, 0x80); //Request parameter transfer procedure
    STAT = readByte(EM7180_ParamAcknowledge); //Check the parameter acknowledge register and loop until the result matches parameter request byte
    while(!(STAT==param)) {
        STAT = readByte(EM7180_ParamAcknowledge);
    }
    writeByte(EM7180_ParamRequest, 0x00); //Parameter request = 0 to end parameter transfer process
    writeByte(EM7180_AlgorithmControl, 0x00); // Re-start algorithm
}

void EM7180_set_gyro_FS (unsigned short int gyro_fs) {
    char bytes[4], STAT;
    bytes[0] = gyro_fs & (0xFF);
    bytes[1] = (gyro_fs >> 8) & (0xFF);
    bytes[2] = 0x00;
    bytes[3] = 0x00;
    writeByte(EM7180_LoadParamByte0, bytes[0]); //Gyro LSB
    writeByte(EM7180_LoadParamByte1, bytes[1]); //Gyro MSB
    writeByte(EM7180_LoadParamByte2, bytes[2]); //Unused
    writeByte(EM7180_LoadParamByte3, bytes[3]); //Unused
    writeByte(EM7180_ParamRequest, 0xCB); //Parameter 75; 0xCB is 75 decimal with the MSB set high to indicate a paramter write processs
    writeByte(EM7180_AlgorithmControl, 0x80); //Request parameter transfer procedure
    STAT = readByte(EM7180_ParamAcknowledge); //Check the parameter acknowledge register and loop until the result matches parameter request byte
    while(!(STAT==0xCB)) {
        STAT = readByte(EM7180_ParamAcknowledge);
    }
    writeByte(EM7180_ParamRequest, 0x00); //Parameter request = 0 to end parameter transfer process
    writeByte(EM7180_AlgorithmControl, 0x00); // Re-start algorithm
}

void EM7180_set_mag_acc_FS (unsigned short int mag_fs, unsigned short int acc_fs) {
    char bytes[4], STAT;
    bytes[0] = mag_fs & (0xFF);
    bytes[1] = (mag_fs >> 8) & (0xFF);
    bytes[2] = acc_fs & (0xFF);
    bytes[3] = (acc_fs >> 8) & (0xFF);
    writeByte(EM7180_LoadParamByte0, bytes[0]); //Mag LSB
    writeByte(EM7180_LoadParamByte1, bytes[1]); //Mag MSB
    writeByte(EM7180_LoadParamByte2, bytes[2]); //Acc LSB
    writeByte(EM7180_LoadParamByte3, bytes[3]); //Acc MSB
    writeByte(EM7180_ParamRequest, 0xCA); //Parameter 74; 0xCA is 74 decimal with the MSB set high to indicate a paramter write processs
    writeByte(EM7180_AlgorithmControl, 0x80); //Request parameter transfer procedure
    STAT = readByte(EM7180_ParamAcknowledge); //Check the parameter acknowledge register and loop until the result matches parameter request byte
    while(!(STAT==0xCA)) {
        STAT = readByte(EM7180_ParamAcknowledge);
    }
    writeByte(EM7180_ParamRequest, 0x00); //Parameter request = 0 to end parameter transfer process
    writeByte(EM7180_AlgorithmControl, 0x00); // Re-start algorithm
}

void WS_PassThroughMode() {
  char stat = 0;
  
  // First put SENtral in standby mode
  writeByte(EM7180_AlgorithmControl, 0x01);
  usleep(5*1000);
  
  // Place SENtral in pass-through mode
  writeByte(EM7180_PassThruControl, 0x01);
  usleep(5*1000);
  stat = readByte(EM7180_PassThruStatus);
  while(!(stat & 0x01))
  {
    stat = readByte(EM7180_PassThruStatus);
    usleep(5*1000);
  }
}

void WS_Resume() {
  char stat = 0;
  
  // Cancel pass-through mode
  writeByte(EM7180_PassThruControl, 0x00);
  usleep(5*1000);
  stat = readByte(EM7180_PassThruStatus);
  while((stat & 0x01))
  {
    stat = readByte(EM7180_PassThruStatus);
    usleep(5*1000);
  }

  // Re-start algorithm
  writeByte(EM7180_AlgorithmControl, 0x00);
  usleep(5*1000);
  stat = readByte(EM7180_AlgorithmStatus);
  while((stat & 0x01))
  {
    stat = readByte(EM7180_AlgorithmStatus);
    usleep(5*1000);
  }
}

void readSenParams() {
	char data[140] = {0};
	int paramnum;
	M24512DFMreadBytes(M24512DFM_DATA_ADDRESS, 0x7f, 0x80, 12, &data[128]); // Page 255
	usleep(100*1000);
	M24512DFMreadBytes(M24512DFM_DATA_ADDRESS, 0x7f, 0x00, 128, &data[0]); // Page 254
	for (paramnum = 0; paramnum < 35; paramnum++) // 35 parameters
	{
		for (int i= 0; i < 4; i++)
		{
			WS_params.Sen_param[paramnum][i] = data[(paramnum*4 + i)];
		}
	}
}

/*
void writeSenParams() {
  char data[140];
  int paramnum;
  for (paramnum = 0; paramnum < 35; paramnum++) // 35 parameters
  {
    for (int i= 0; i < 4; i++)
    {
      data[(paramnum*4 + i)] = WS_params.Sen_param[paramnum][i];
    }
  }
  M24512DFMwriteBytes(M24512DFM_DATA_ADDRESS, 0x7f, 0x80, 12, &data[128]); // Page 255
  usleep(100*1000);
  M24512DFMwriteBytes(M24512DFM_DATA_ADDRESS, 0x7f, 0x00, 128, &data[0]); // Page 254
}
*/

void M24512DFMreadBytes(char device_address, char data_address1, char data_address2, char count, char * dest) {  

	int i1 = ioctl(file, I2C_SLAVE, device_address);
	//printf("ret ioctl: %d \n", i1);
	i1 = i1;
	
	char txbuf[2] = {0};
	txbuf[0] = data_address1;
	txbuf[1] = data_address2;
	
	int i2 = write(file, txbuf, 2);
	//printf("ret write: %d \n", i2);
	i2 = i2;
	
	//printf("count: %d \n", count);
	char rxbuf[140] = {0};
	int i3 = read(file, rxbuf, count);
	//printf("ret read: %d \n", i3);
	i3 = i3;
	for(int rx = 0; rx < count; rx++) {
		//printf("0x%02x ", rxbuf[rx]);
	}
	//printf("\n");	
	
	int i = 0; 
	while (i < count)
	{
		dest[i] = rxbuf[i];
		//printf("i: %d \n", i);
		i++;
	}
  
	int i4 = ioctl(file, I2C_SLAVE, EM7180_ADDRESS);
	//printf("ret ioctl2: %d \n\n", i4);
	i4 = i4;
}


void EM7180_set_WS_params()
{
  char param = 1;
  char STAT;
  
  // Parameter is the decimal value with the MSB set high to indicate a paramter write processs
  param = param | 0x80;
  writeByte(EM7180_LoadParamByte0, WS_params.Sen_param[0][0]);
  writeByte(EM7180_LoadParamByte1, WS_params.Sen_param[0][1]);
  writeByte(EM7180_LoadParamByte2, WS_params.Sen_param[0][2]);
  writeByte(EM7180_LoadParamByte3, WS_params.Sen_param[0][3]);
  writeByte(EM7180_ParamRequest, param);

  // Request parameter transfer procedure
  writeByte(EM7180_AlgorithmControl, 0x80);

  // Check the parameter acknowledge register and loop until the result matches parameter request byte
  STAT = readByte(EM7180_ParamAcknowledge);
  while(!(STAT==param))
  {
    STAT = readByte(EM7180_ParamAcknowledge);
  }
  for(int i=1; i<35; i++)
  {
    param = (i+1) | 0x80;
    writeByte(EM7180_LoadParamByte0, WS_params.Sen_param[i][0]);
    writeByte(EM7180_LoadParamByte1, WS_params.Sen_param[i][1]);
    writeByte(EM7180_LoadParamByte2, WS_params.Sen_param[i][2]);
    writeByte(EM7180_LoadParamByte3, WS_params.Sen_param[i][3]);
    writeByte(EM7180_ParamRequest, param);
    
    // Check the parameter acknowledge register and loop until the result matches parameter request byte
    STAT = readByte(EM7180_ParamAcknowledge);
    while(!(STAT==param))
    {
      STAT = readByte(EM7180_ParamAcknowledge);
    }
  }
  // Parameter request = 0 to end parameter transfer process
  writeByte(EM7180_ParamRequest, 0x00);
  
  // Re-start algorithm
  writeByte(EM7180_AlgorithmControl, 0x00);
}

void EM7180_get_WS_params() {
  char param = 1;
  char STAT;

  writeByte(EM7180_ParamRequest, param);
  usleep(10 * 1000);
  
  // Request parameter transfer procedure
  writeByte(EM7180_AlgorithmControl, 0x80);
  usleep(10 * 1000);

   // Check the parameter acknowledge register and loop until the result matches parameter request byte
  STAT = readByte(EM7180_ParamAcknowledge);
  while(!(STAT==param))
  {
    STAT = readByte(EM7180_ParamAcknowledge);
  }
  
  // Parameter is the decimal value with the MSB set low (default) to indicate a paramter read processs
  WS_params.Sen_param[0][0] = readByte(EM7180_SavedParamByte0);
  WS_params.Sen_param[0][1] = readByte(EM7180_SavedParamByte1);
  WS_params.Sen_param[0][2] = readByte(EM7180_SavedParamByte2);
  WS_params.Sen_param[0][3] = readByte(EM7180_SavedParamByte3);

  for(int i=1; i<35; i++)
  {
    param = (i+1);
    writeByte(EM7180_ParamRequest, param);
    usleep(10 * 1000);
    
    // Check the parameter acknowledge register and loop until the result matches parameter request byte
    STAT = readByte(EM7180_ParamAcknowledge);
    while(!(STAT==param))
    {
      STAT = readByte(EM7180_ParamAcknowledge);
    }
    WS_params.Sen_param[i][0] = readByte(EM7180_SavedParamByte0);
    WS_params.Sen_param[i][1] = readByte(EM7180_SavedParamByte1);
    WS_params.Sen_param[i][2] = readByte(EM7180_SavedParamByte2);
    WS_params.Sen_param[i][3] = readByte(EM7180_SavedParamByte3);
  }
  // Parameter request = 0 to end parameter transfer process
  writeByte(EM7180_ParamRequest, 0x00);

  // Re-start algorithm
  writeByte(EM7180_AlgorithmControl, 0x00);
}

void readSENtralQuatData(float * destination) {
    char rawData[16];  // x/y/z quaternion register data stored here
    readBytes(EM7180_QX, 16, &rawData[0]);       // Read the sixteen raw data registers into data array
    destination[0] = uint32_reg_to_float (&rawData[0]);
    destination[1] = uint32_reg_to_float (&rawData[4]);
    destination[2] = uint32_reg_to_float (&rawData[8]);
    destination[3] = uint32_reg_to_float (&rawData[12]);  // SENtral stores quats as qx, qy, qz, q0!
}

void readSENtralGyroData(short int * destination)
{
    char rawData[6];  // x/y/z gyro register data stored here
    readBytes(EM7180_GX, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
    destination[0] = (short int) (((short int)rawData[1] << 8) | rawData[0]);   // Turn the MSB and LSB into a signed 16-bit value
    destination[1] = (short int) (((short int)rawData[3] << 8) | rawData[2]);  
    destination[2] = (short int) (((short int)rawData[5] << 8) | rawData[4]); 
}

float uint32_reg_to_float (char *buf)
{
    union {
        unsigned int ui32;
        float f;
    } u;
    
    u.ui32 =   (((unsigned int)buf[0]) +
               (((unsigned int)buf[1]) <<  8) +
               (((unsigned int)buf[2]) << 16) +
               (((unsigned int)buf[3]) << 24));
    return u.f;
}





/* Code snippets for later use

	clock_t last, now;
	unsigned long timeStamp;
	double course, angleRate;
		
	last = clock();
	int i = 0;
	while (i < 20) {
		i++;

		now = clock();
		//double dtms = (double)(now - last) / CLOCKS_PER_SEC * 1000;


	}
	
	//printf("%x \n", readByte(EM7180_ACC_LPF_BW));
	
	char rbuf[40] = {0};
	readBytes(0x70, 3, rbuf);
	printf("rbuf = 0x%02x 0x%02x 0x%02x \n\n", rbuf[0], rbuf[1], rbuf[2]);
	
	    
    printf("0x%02x \n", readByte(EM7180_LoadParamByte0));

*/

		
