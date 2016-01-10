//======================================================================== 
// Package: Motion Mind DC Motor Controller
// Authors: Nitendra Nath
//          Vilas Chitrakaran
// Start Date: Sun Jan 15 15:36:46 EST 2006
// Compiler: GNU GCC 2.95.3 and above
// Operating System: QNX RTP, Linux
// ----------------------------------------------------------------------  
// File: MotionMind.hpp
// Implementation of the class MotionMind.
// See MotionMind.hpp for more details.
//========================================================================  
 
#include "MotionMind.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>

//#define DEBUG

struct MMRegInfo // struct to hold register length and whether the contents
                 // can be either positive or negative
{
 int8_t bytes;
 bool isPositiveRangeOnly;
};

#ifdef FIRMWARE_REVISION_5UP
 #define NUM_REGS 28
#else
 #define NUM_REGS 23
#endif

static struct MMRegInfo regsInfo[NUM_REGS]
 = {
    {4,false},// MMReg_position
    {2,true}, // MMReg_velocityLimit
    {1,true}, // MMReg_velocityFf
    {2,true}, // MMReg_function
    {2,true}, // MMReg_pTerm
    {2,true}, // MMReg_iTerm
    {2,true}, // MMReg_dTerm
    {1,true}, // MMReg_address
    {1,true}, // MMReg_pidScalar
    {1,true}, // MMReg_timer
    {2,true}, // MMReg_rcMax
    {2,true}, // MMReg_rcMin
    {2,true}, // MMReg_rcBand
    {2,true}, // MMReg_rcCount
    {2,false},// MMReg_velocity
    {4,false},// MMReg_time
    {2,true}, // MMReg_status
    {1,true}, // MMReg_revision
    {1,true}, // MMReg_mode
    {2,true}, // MMReg_analogCon
    {2,true}, // MMReg_analogFbck
    {2,false},// MMReg_pwmOut
    {4,false} // MMReg_indexPos
#ifdef FIRMWARE_REVISION_5UP
    ,
    {4,false},// MMReg_vnLimit
    {4,false},// MMReg_vpLimit
    {2,true}, // MMReg_pwmLimit
    {2,true}, // MMReg_deadband
    {4,false},// MMReg_desiredPosition
#endif
   };

//======================================================================== 
// MotionMind::MotionMind: Constructor of the MotionMind class
//========================================================================  
MotionMind::MotionMind ()
{
 d_isInit = false;
 d_port = -1;
}


//======================================================================== 
// MotionMind::~MotionMind: Destructor of the MotionMind class
//========================================================================  
MotionMind::~MotionMind ()
{
 if(d_isInit)
  reset(); 
 if(d_port != -1) {
  close(d_port);
  d_port = -1;
  d_isInit = false;
 }
}


//======================================================================== 
// MotionMind::init
//======================================================================== 
int MotionMind::init(const char *port, int baud, int address, bool verbose)
{
 d_isInit = false;
 d_port = open(port, O_RDWR | O_NOCTTY);
 if(d_port == -1) {
  fprintf(stderr, "[MotionMind::init] ERROR opening serial port %s\n",
          port);
  return -1;
 }
 
 struct termios termio_options;
 
 tcflush(d_port, TCIOFLUSH);
 tcgetattr(d_port, &termio_options);
 if(baud == 19200) {
  cfsetispeed(&termio_options, B19200);
  cfsetospeed(&termio_options, B19200);
 }else{
  cfsetispeed(&termio_options, B9600);
  cfsetospeed(&termio_options, B9600);
 }
 cfmakeraw(&termio_options);     
 termio_options.c_lflag = 0;
 termio_options.c_cflag &= ~PARENB; 
 termio_options.c_cflag &= ~CSTOPB;
 termio_options.c_cflag &= ~CSIZE;
 termio_options.c_cflag |= (CS8 | CLOCAL | CREAD);
 termio_options.c_iflag &= ~(IXON|IXOFF|IXANY);
 termio_options.c_oflag &= ~OPOST;
 termio_options.c_cc[VMIN] = 0;
 termio_options.c_cc[VTIME] = 0x14;
 tcsetattr(d_port, TCSADRAIN, &termio_options);

 // ensure default R/W response is 0x06, and not position, velocity
 // or time.
 d_address = address;
 int16_t tmp;
 if( (tmp = readRegister(MMReg_function)) == -1 ) {
  fprintf(stderr, "<-[MotionMind::init]\n");
  return -1;
 }
 if( writeRegister(MMReg_function, (tmp & 0xFFFFFFF1)) == -1) {
  fprintf(stderr, "<-[MotionMind::init]\n");
  return -1;
 }

 d_isInit = true;

 // try to read revision and mode on the device
 if( (tmp = (int8_t) readRegister(MMReg_revision)) == -1 ) {
  fprintf(stderr, "<-[MotionMind::init]\n");
  return -1;
 }
 tmp = tmp << 8;
 if( (tmp += (int8_t) readRegister(MMReg_mode)) == -1 ) {
  fprintf(stderr, "<-[MotionMind::init]\n");
  return -1;
 }

 if(verbose){
  fprintf(stdout, "%s\n* %s %s (%d baud)\n* Revision: %d, Mode: %d\n%s\n",
  "===================================================================",
  "Motion Mind Controller on serial port", port, baud,
  ((tmp >> 8) & 0xFF), (tmp & 0xFF),
  "===================================================================");
 }

 return 0;
}


//======================================================================== 
// MotionMind::changeSpeed
//======================================================================== 
int MotionMind::changeSpeed(int16_t speed)
{ 
 computeTwosComplement(speed, speed);
 d_buffer[0] = 0x14;
 d_buffer[1] = 0x01;
 d_buffer[2] = speed & 0xFF;
 d_buffer[3] = (speed >> 8) & 0xFF;
 computeCheckSum(d_buffer, 4, d_buffer[4]);

 if( send(d_buffer, 5) == -1 ) {
  fprintf(stderr, "<-[MotionMind::changeSpeed]\n");
  return -1;
 }

 return 0;
}


//======================================================================== 
// MotionMind::moveToAbsolute
//======================================================================== 
int MotionMind::moveToAbsolute(int32_t position)
{
 computeTwosComplement(position, position);
 d_buffer[0] = 0x15;
 d_buffer[1] = 0x01;
 d_buffer[2] = position & 0xFF;
 d_buffer[3] = (position >> 8) & 0xFF;
 d_buffer[4] = (position >> 16) & 0xFF;
 d_buffer[5] = (position >> 24) & 0xFF;
 computeCheckSum(d_buffer, 6, d_buffer[6]);

 if( send(d_buffer, 7) == -1) {
  fprintf(stderr, "<-[MotionMind::moveToAbsolute]\n");
  return -1;
 }

 return 0;
}


//======================================================================== 
// MotionMind::moveToRelative
//======================================================================== 
int MotionMind::moveToRelative(int32_t position)
{
 computeTwosComplement(position, position);
 d_buffer[0] = 0x16;
 d_buffer[1] = 0x01;
 d_buffer[2] = position & 0xFF;
 d_buffer[3] = (position >> 8) & 0xFF;
 d_buffer[4] = (position >> 16) & 0xFF;
 d_buffer[5] = (position >> 24) & 0xFF;
 computeCheckSum(d_buffer, 6, d_buffer[6]);

 if( send(d_buffer, 7) == -1) {
  fprintf(stderr, "<-[MotionMind::moveToRelative]\n");
  return -1;
 }

 return 0;
}


//======================================================================== 
// MotionMind::moveAtVelocity
//======================================================================== 
int MotionMind::moveAtVelocity(int16_t velocity)
{
 computeTwosComplement(velocity, velocity);
 d_buffer[0] = 0x17;
 d_buffer[1] = 0x01;
 d_buffer[2] = velocity & 0xFF;
 d_buffer[3] = (velocity >> 8) & 0xFF;
 computeCheckSum(d_buffer, 4, d_buffer[4]);

 if( send(d_buffer, 5) == -1) {
  fprintf(stderr, "<-[MotionMind::moveAtVelocity]\n");
  return -1;
 }
 
 return 0;
}


//======================================================================== 
// MotionMind::writeRegister
//======================================================================== 
int MotionMind::writeRegister(MMReg_t reg, int32_t val)
{
 int i;
#ifdef DEBUG
 fprintf(stderr, "DEBUG [MotionMind::readRegister] Attempting to write register %d\n", reg);
#endif 
 computeTwosComplement(val, val);
 d_buffer[0] = 0x18;
 d_buffer[1] = 0x01;
 d_buffer[2] = reg;
 
 for(i = 0; i < regsInfo[reg].bytes; ++i) {
  d_buffer[3+i] = (val >> (8 * i) ) & 0xFF;
 }
 i = regsInfo[reg].bytes + 3;
 computeCheckSum(d_buffer, i, d_buffer[i]);
 
 ++i;
 if( send(d_buffer, i) == -1) {
  fprintf(stderr, "<-[MotionMind::writeRegister]\n");
  return -1;
 }

 return 0;
}


//======================================================================== 
// MotionMind::writeStoreRegister
//======================================================================== 
int MotionMind::writeStoreRegister(MMReg_t reg, int32_t val)
{
 int i;
 computeTwosComplement(val, val);
 d_buffer[0] = 0x19;
 d_buffer[1] = 0x01;
 d_buffer[2] = reg;
 
 for(i = 0; i < regsInfo[reg].bytes; ++i) {
  d_buffer[3+i] = (val >> (8 * i) ) & 0xFF;
 }
 i = regsInfo[reg].bytes + 3;
 computeCheckSum(d_buffer, i, d_buffer[i]);
 
 ++i;
 if( send(d_buffer, i) == -1) {
  fprintf(stderr, "<-[MotionMind::writeStoreRegister]\n");
  return -1;
 }

 return 0;
}


//======================================================================== 
// MotionMind::readRegister
//======================================================================== 
int32_t MotionMind::readRegister(MMReg_t reg)
{
 int32_t i = (0x1 << reg);
#ifdef DEBUG
 fprintf(stderr, "DEBUG [MotionMind::readRegister] Attempting to read register 0x%x\n", i);
#endif 
 d_buffer[0] = 0x1A;
 d_buffer[1] = 0x01;
 d_buffer[2] = i & 0xFF;
 d_buffer[3] = (i >> 8) & 0xFF;
 d_buffer[4] = (i >> 16) & 0xFF;
 d_buffer[5] = (i >> 24) & 0xFF;
 computeCheckSum(d_buffer, 6, d_buffer[6]);

 if( sendAndReceive(d_buffer, 7, d_buffer, regsInfo[reg].bytes + 2) == -1 ) {
  fprintf(stderr, "<-[MotionMind::readRegister]\n");
  return -1;
 }

 // returned data format: [address][data0..][checksum]
 i = 0;
 for(int j = 0; j < regsInfo[reg].bytes; ++j) {
  i += (d_buffer[1+j] & 0xFF) << (8 * j);
 }
 return i;
}

/*
//======================================================================== 
// MotionMind::readAllRegisters
//======================================================================== 
int MotionMind::readAllRegisters(MMData_t &data)
{
 d_buffer[0] = 0x1A;
 d_buffer[1] = 0x01;
 d_buffer[2] = 0xFF;
 d_buffer[3] = 0xFF;
 d_buffer[4] = 0xFF;
 d_buffer[5] = 0xFF;
 computeCheckSum(d_buffer, 6, d_buffer[6]);

 if( sendAndReceive(d_buffer, 7, (char *)(&data), sizeof(MMData_t)+2) == -1 ) {
  fprintf(stderr, "<-[MotionMind::readAllRegisters]\n");
  return -1;
 }

 return 0;
}
*/

//======================================================================== 
// MotionMind::restore
//======================================================================== 
int MotionMind::restore()
{
 d_buffer[0] = 0x1B;
 d_buffer[1] = 0x01;
 d_buffer[2] = 0x1C;

 if( send(d_buffer, 3) == -1 ) {
  fprintf(stderr, "<-[MotionMind::restore]\n");
  return -1;
 }
 
 return 0;
}


//======================================================================== 
// MotionMind::reset
//======================================================================== 
int MotionMind::reset()
{
 d_buffer[0] = 0x1C;
 d_buffer[1] = 0x01;
 d_buffer[2] = 0x1D;

 if( send(d_buffer, 3) == -1) {
  fprintf(stderr, "<-[MotionMind::reset]\n");
  return -1;
 }
 
 return 0;
}


//======================================================================== 
// MotionMind::send
//======================================================================== 
int MotionMind::send(char *data, int len)
{
 int ret, timeout;
 
#ifdef DEBUG
 fprintf(stderr, "DEBUG [MotionMind::send] sending %d byte long data.\n", len);
#endif

 timeout = 0;
 ret = 0;
 while(ret < len && timeout < 5){
  ret += write(d_port, &(data[ret]), len-ret);
  ++timeout;
 }
 tcdrain(d_port);
 if( ret != len) {
  fprintf(stderr, "*-[MotionMind::send]: ERROR writing to device\n");
  return -1;
 }
 
#ifdef DEBUG
 fprintf(stderr, "DEBUG [MotionMind::send] receiving command response.\n");
#endif
 
 ret = 0x0;
 if( (read(d_port, (char *)(&ret), 1) != 1) || (ret != 0x06) ) {
  fprintf(stderr, "*-[MotionMind::send] ERROR reading from device\n");
  return -1;
 }

#ifdef DEBUG
 fprintf(stderr, "DEBUG [MotionMind::send] received response 0x%x.\n", (char) ret);
#endif
 
 return 0;
}
   

//======================================================================== 
// MotionMind::sendAndReceive
//======================================================================== 
int MotionMind::sendAndReceive(char *inData, int inLen, char *outData, int outLen)
{
 int ret, timeout;
 int numtries = 10;
#ifdef DEBUG
 fprintf(stderr, "DEBUG [MotionMind::sendAndReceive] sending %d byte long data.\n", inLen);
#endif

 timeout = 0;
 ret = 0;
 while(ret < inLen && timeout < numtries){
  ret += write(d_port, &(inData[ret]), inLen-ret);
  ++timeout;
 }
 tcdrain(d_port);
 if( ret != inLen ) {
  fprintf(stderr, "*-[MotionMind::sendAndReceive] ERROR writing to device.\n");
  return -1;
 }

#ifdef DEBUG
 fprintf(stderr, "DEBUG [MotionMind::sendAndReceive] receiving %d byte long response.\n",
         outLen);
#endif

 timeout = 0;
 ret = 0;
 while(ret < inLen && timeout < numtries){
  ret += read(d_port, &(outData[ret]), outLen-ret);
  ++timeout;
 }
 if( ret != outLen ) {
  fprintf(stderr, "*-[MotionMind::sendAndReceive] ERROR reading from device (%d / %d).\n", 
          ret, outLen);
  return -1;
 }
 
#ifdef DEBUG
 fprintf(stderr, "DEBUG [MotionMind::sendAndReceive] received response 0x");
 for(ret = 0; ret < outLen; ++ret) {
  fprintf(stderr, "%x ", (unsigned char)outData[ret]);
 }
 fprintf(stderr, "\n");
#endif

 return 0;
}


//======================================================================== 
// MotionMind::computeCheckSum
//======================================================================== 
void MotionMind::computeCheckSum(char *value, int numBytes, char &sum)
{
 sum = 0x0;
 for(int i = 0; i < numBytes; ++i) 
  sum += value[i];
}

