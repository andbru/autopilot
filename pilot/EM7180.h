
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <time.h>
#include <math.h>

#include <linux/types.h>
#include <linux/i2c-dev.h>
//#include <i2c/smbus.h>
#include <sys/ioctl.h>


#define EM7180_ADDRESS            0x28
#define EM7180_HostControl        0x34
#define EM7180_PassThruStatus     0x9E 
#define EM7180_PassThruControl    0xA0
#define EM7180_ACC_LPF_BW         0x5B  //Register GP36
#define EM7180_GYRO_LPF_BW        0x5C  //Register GP37
#define EM7180_QRateDivisor       0x32  // uint8_t 
#define EM7180_MagRate            0x55
#define EM7180_AccelRate          0x56
#define EM7180_GyroRate           0x57
#define EM7180_AlgorithmControl   0x54
#define EM7180_SentralStatus      0x37
#define EM7180_AlgorithmStatus    0x38
#define EM7180_ResetRequest       0x9B
#define EM7180_LoadParamByte0     0x60
#define EM7180_LoadParamByte1     0x61
#define EM7180_LoadParamByte2     0x62
#define EM7180_LoadParamByte3     0x63
#define EM7180_SavedParamByte0    0x3B
#define EM7180_SavedParamByte1    0x3C
#define EM7180_SavedParamByte2    0x3D
#define EM7180_SavedParamByte3    0x3E
#define EM7180_ParamRequest       0x64
#define EM7180_ParamAcknowledge   0x3A
#define M24512DFM_DATA_ADDRESS    0x50   // Address of the 500 page M24512DRC EEPROM data buffer, 1024 bits (128 8-bit bytes) per page
#define EM7180_LoadParamByte0     0x60
#define EM7180_LoadParamByte1     0x61
#define EM7180_LoadParamByte2     0x62
#define EM7180_LoadParamByte3     0x63
#define EM7180_EventStatus        0x35
#define EM7180_ErrorRegister      0x50
#define EM7180_QX                 0x00  // this is a 32-bit normalized floating point number read from registers 0x00-03
#define EM7180_GX                 0x22  // int16_t from registers 0x22-23


struct Sentral_WS_params
{
  char Sen_param[35][4];
};

struct Sentral_WS_params WS_params;

void initEM7180();
int pollEM7180(float *yaw, float *w);
float yawDev(float yaw);

void writeByte(char subAddress, char byte);
char readByte(char subAddress);
void readBytes(char subAddress, char count, char *dest);
void EM7180_set_integer_param (char param, unsigned int param_val);
void EM7180_set_gyro_FS (unsigned short int gyro_fs);
void EM7180_set_mag_acc_FS (unsigned short int mag_fs, unsigned short int acc_fs);
void WS_PassThroughMode();
void WS_Resume();
void readSenParams();
void writeSenParams();
void M24512DFMreadBytes(char device_address, char data_address1, char data_address2, char count, char * dest);
void EM7180_set_WS_params();
void EM7180_get_WS_params();
void readSENtralQuatData(float * destination);
void readSENtralGyroData(short int * destination);
float uint32_reg_to_float (char *buf);
