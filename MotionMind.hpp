//======================================================================== 
// Package: Motion Mind DC Motor Controller 
// Authors: Nitendra Nath
//          Vilas Chitrakaran
// Start Date: Sun Jan 15 15:36:46 EST 2006
// Compiler: GNU GCC 2.95.3 and above
// Operating System: QNX RTP, Linux
// ----------------------------------------------------------------------  
// File: MotionMind.hpp
// Interface of the class MotionMind.
//========================================================================  
 
#ifndef INCLUDED_MotionMind_hpp
#define INCLUDED_MotionMind_hpp
 
#include <inttypes.h>
#include <stdlib.h>

//#define FIRMWARE_REVISION_5UP // uncomment for firmware revision 5 and above

//======================================================================== 
// MMReg_t: Register indices and description of contents, with range and 
// default in brackets) (copied from user manual).
//======================================================================== 
typedef enum MMReg
{
 MMReg_position= 0,		// current position count from encoder x 4 
						// i.e. A single rotation from a 500 CPR
						// encoder would result in a position of
						// 2000, (-2,147,483,648 to + 2,147,483,647),
						// (0).
								
 MMReg_velocityLimit,	// Limit on motor speed changes per step or
 						// update cycle (+1 to +1023), (1023).
 								
 MMReg_velocityFf,		// In mode 4 (serial closed-loop mode), if 
 						// MMFunc_VelocityLimit bit in function register is  
 						// set then this register provides a proportional 
 						// gain outside of the PID loop. The extra 
 						// proportional gain is based on the desired 
 						// velocity times contents of this register
 						// divided by 255 (velocity feedforward gain),
 						// (0 to +255), (128).
 								
 MMReg_function,			// Contain a number of bits that enable/disable
 						// specific functions. See description for 
 						// MMFunc_t, (default: 0).
 								
 MMReg_pTerm,			// proportional gain (0 to +65,535), (6000).
 
 MMReg_iTerm	,			// integral gain (0 to +65,535), (35).
 
 MMReg_dTerm,			// derivative gain (0 to +65,535), (0).
 
 MMReg_address,			// device address (0 to +255), (1).
 
 MMReg_pidScalar,		// number of "divide-by-twos" the output goes  
 						// through before being used to determine the 
 						// output drive signal in MMReg_pwmOut,
 						// (0 to +32), (14). 
 								
 MMReg_timer,			// debounce and motor speed update timer, in 
 						// increments of 5ms. To increase motor  
 						// responsiveness to commands (in open-loop  
 						// modes) reduce this number and increase value  
 						// in MMReg_velocityLimit. This debounce rate 
 						// is also used to monitor the POS_LIM, and  
 						// NEG_LIM input pins. The default setting is 
 						// 50ms (10 x 5ms) and could be reduced for 
 						// faster response to limit switches, (+1 to 
 								// +255), (10).
 								
 MMReg_rcMax,			// R/C pulse width associated with maximum   
 						// forward speed in increments of 814ns. The   
 						// default is 2ms and pulse widths from RCMAX  
 						// to RCMAX + 0.5ms are treated as RCMAX in   
 						// width. Pulse widths greater than RCMAX  
 						// + 0.5ms are considered bad signals, 
 						// (0 to +65,535), (2457).
 								
 MMReg_rcMin,			// R/C pulse width associated with maximum 
  						// reverse speed in increments of 814ns. 
 						// The default setting is 1ms and pulse widths 
						// from RCMIN to RCMIN - 0.5ms are treated as 
						// RCMIN in width. Pulse widths less than RCMIN 
						// - 0.5ms are considered bad signals.
						// (0 to +65,535), (1229).

 MMReg_rcBand,			// Pulse widths of 1.5ms +/- RCBAND (in 814ns 
 						// increments) are treated as 1.5ms signals 
 						// (stopped). The default puts pulse widths from 
 						// 1.467ms to 1.533ms are treated as 1.5ms signals
 						// (0 to +65,535), (40).
 								
 MMReg_rcCount,			// raw R/C pulse width measured while in mode 0, 
 						// expressed in increments of 814ns. (0 to 
 						// +65,535).
 								
 MMReg_velocity,			// in mode 4 (serial closed-loop mode) this  
 						// register contains an average of the last 64 
 						// velocity measurements. Each measurement occurs  
 						// over a 5ms period and equals 4X the actual 
 						// encoder count. Therefore an average velocity  
 						// of 200 relates to 50 encoder counts during a 
 						// 5ms period, (-32,768 to +32,767).
 								
 MMReg_time,				// Elapsed time since power up in 5ms increments. 
 						// (0 to +4,294,967,295).
 
 MMReg_status,			// contains bits related to input pins or 
 						// other status conditions, (0 to 65,535).
 								
 MMReg_revision,			// firmware revision. This may be compared 
 						// to online errata sheets if known bugs exist 
 						// in the firmware, (0 to +255).
 								
 MMReg_mode,				// current operating mode, (0 to +7).
 
 MMReg_analogCon,		// Raw analog measurement at J4 P4. The value will 
 						// range from 0 to +1023 in increments of 5VOUT/1023,
 						// (typically 4.89mV).
 								
 MMReg_analogFbck,		// Raw analog measurement at J4 P3. The value 
 						// will range from 0 to 1023 in increments 
 						// of 5VOUT/1023, (typically 4.89mV).
 								
 MMReg_pwmOut,			// actual PWM drive signal where the sign 
 						// determines motor direction. (-1023 to +1023,
 						// 0 when stopped).
 								
 MMReg_indexPos			// 2's compliment number, relates to the last 
 						// position measured before the index input is
 						// asserted. Using this value the user can locate 
 						// the approximate index position. Then using the
 						// MMStatus_index bit in status register a controller  
 						// can home in on the actual index position. The index 
 						// pulse comes from a 3rd channel on many quadrature 
 						// encoders, and occurs with each encoder revolution.
 						// (-2,147,483,648 to + 2,147,483,647).
 								
#ifdef FIRMWARE_REVISION_5UP
 ,MMReg_vnLimit,			// (Rev5+ firmware only) 32-bit 2's compliment 
 						// number is used with the MMFunc_virtLimit bit of the 
 						// function register setting to establish a virtual 
 						// negative stop limit for closed loop position 
 						// control modes of operation. The VNLIMIT represents a
 						// lower boundary for negative moving positions. 
 						// For example if the VNLIMIT is set for -10000, 
 						// and the controller is directed to move to -20000, 
 						// it will be forced to stop at -10000. MMStatus_vnLimit 
 						// bit in status register will be set to 1 when this 
 						// boundary is reached, (-2,147,483,648 to 
 						// +2,147,483,647), (-100,000).
 								
 MMReg_vpLimit,			// (Rev5+ firmware only) 32-bit 2's compliment 
 						// number is used with the MMFunc_virtLimit bit of the 
 						// function register setting to establish a virtual
 						// positive stop limit for closed loop position 
 						// control modes of operation. The VPLIMIT represents 
 						// an upper boundary for positive moving positions. 
 						// For example if the VPLIMIT is set for +10000, and 
 						// the controller is directed to move to +20000, it 
 						// will be forced to stop at +10000. MMStatus_vpLimit bit
 						// in status register will be set to 1 when this 
 						// boundary is reached, (-2,147,483,648 to 
 						// +2,147,483,647), (+100,000).
 								
 MMReg_pwmLimit,			// (Rev5+ firmware only) 16 bit register (limited 
 						// to +1 to +1023) restricts the PWM h-bridge 
 						// drive signal. At the default value (1023) the PWM 
 						// output can range from -1023 to +1023 (-100% and + 100% 
 						// duty cycle respectively). Setting the PWMLIMIT to 
 						// 512 would keep the drive signal to the h-bridge 
 						// between -50% and +50%. For closed loop modes this 
 						// register can be used to current limit the h-bridge 
 						// by preventing higher duty cycles from occurring,
 						// (default 1023).

 MMReg_deadband,			// (Rev5+ firmware only) 16-bit register (limited 0  
 						// to +1023) is used to establish a dead band 
 						// (stopped motor) around 2.5V DC for bi-directional 
 						// analog control mode. Each bit represents a 4.88mV 
 						// step. So in order to create a dead band from 2.4V 
 						// to 2.6V this register would be set to 20 (4.88mV *
 						// 20 = 0.98V). When enableDb is set in the function 
 						// register, this register will also be applied to 
 						// closed loop modes to create a dead band around 
 						// the commanded position (default 15).
 								
 MMReg_desiredPosition	// (Rev5+ firmware only) 32-bit 2's compliment 
						// register holds the commanded position internally 
						// used by the controller. This is a read only 
 						// register. In closed-loop modes this is the 
 						// position the controller has been commanded to 
 						// move to. In some modes of operation this register 
 						// may be useful to have access (specifically mode 0 
 						// - RC Position Control Mode).
#endif
}MMReg_t;


//======================================================================== 
// MMFunc_t: Function register bit definitions. 
//======================================================================== 
typedef enum MMFunc
{
 MMFunc_posPwUp = 0x0001,     // When set: The position register will be loaded 
 						      // from EEPROM on power up. This can be used in 
 						      // conjunction with the savePos mode set using 
 						      // function register to restore the last position 
 						      // to the system after power is removed and then 
 						      // reapplied. An external controller could also use 
 						      // the writeStore() command to store current position 
 						      // data to EEPROM and then setting this bit would 
 						      // restore that position on power-up. 
 						      // When clear: The POSITION register defaults to 0 
 						      // on power up.
 						
 MMFunc_satProt = 0x0010,     // When set: The integral summation is limited to 
 						      // +/-4096, this severely limits the ability of the
 						      // integral to build up over time to account for 
 						      // small errors. But it may also prevent large errors 
 						      // from accumulating and swamping out the proportional 
 						      // part of the PID. When clear: The integral summation 
 						      // can build up to +/-120,000.
 						
 MMFunc_savePos = 0x0020,     // When set: If operating in mode 4 (serial closed-loop), 
 						      // the position register will be stored in EEPROM 
 					          // (non-volatile memory) if the motor velocity stays 
 						      // at 0 continuously for 5 minutes. When clear: no 
 						      // effect.
 						
 MMFunc_velLimit = 0x0040,    // When set: If operating in mode 4 (serial closed-loop), 
 						      // and using position control commands (not 
 						      // moveAtVelocity()) the absolute value of the motor 
 						      // velocity is limited to the value in velocityLimit 
 						      // register. When clear: Motor movements are at the 
 						      // highest speed possible.
 						
 MMFunc_activeStop = 0x0080,	  // When set: When motor control reaches a stop condition 
 						      // both leads of the motor are tied to ground. When 
 						      // clear: Motor leads float when stop occurs	.
 						
 MMFunc_lastRc = 0x0100,      // When set: In mode 0 (R/C open loop) if a bad R/C pulse 
 						      // is received (no pulse, pulse too long, or pulse too 
 						      // short) the average of the last 64 valid pulses will 
 						      // be used to determine motor speed. Turning on/off an 
 						      // RC transmitter can still cause erroneous signals of 
 						      // valid duration to be received. This function bit 
 						      // reduces but does not remove this possibility. When 
 						      // clear: An invalid R/C pulse will be treated as a 
 						      // 1.5ms (stopped) signal.

 MMFunc_adStep = 0x0200,	      // When set: Modes 0,1, and 2 use an analog input to 
 						      // select a motor speed change limit. It may be desirable 
 						      // to do away with this analog input. if that's the case 
 						      // setting this bit forces the controller to take it's 
 						      // step limit from the contents of the velocityLimit 
 						      // register. When clear: Modes 0,1, and 2 use an analog 
 						      // input to determine motor step changes allowed from one 
 						      // update to the next.

 MMFunc_adSerial = 0x0400,   // When set: In Mode 5 the source of the desired position 
 						     // is serial commands moveToAbsolute() or moveToRelative().
 						     // Care should be taken to ensure that the commanded 
 						     // position is a value between 0 and 1023. When clear: 
 						     // In Mode 5 the source of the desired position is the 
 						     // analog control signal (0-5V) provided at ANA_CON 
 						     // (J4 P4).

 MMFunc_enableDb = 0x0800,   // When set: In modes 4 and 5 the contents of the 
 						     // deadband register are used to set a deadband around 
 						     // the desired position. If (position - deadband) <= 
 						     // Desired Position <= (position + deadband) then the 
 						     // pwmOut signal is forced to 0. Not for use with 
 						     // moveAtVelocity() command. When clear: No dead band 
 						     // exists in modes 4 and 5.
 						     // Note - In firmware 2,3,4 the register rcBand was used 
 						     // to set the dead band, and was shared with the rcBand 
 						     // setting.

 MMFunc_selectFbck = 0x1000,	 // Firmware revision 4.0, Mode 0: 
 						     // When set: Uses the analog feedback input as the actual 
 						     // position signal and converts the 1-2ms R/C signal to a 
 						     // desired position value. When clear: Mode 0 operates in 
 						     // speed control mode.
 						     // Firmware revision 5.0, Mode 5:
 						     // When set: The desired position is derived from the 
 						     // analog input (J4 P4) 0-5V 0-1023 positions, but the 
 						     // feedback comes from an encoder attached to J6. When 
 						     // clear: The desired position is derived from the analog 
 						     // control input (J4 P4) and the position feedback is 
 						     // derived from the analog feedback input (J4 P3).

 MMFunc_virtLimit = 0x2000	 // Virtual limit settings are used to restrict movement 
 						     // in closed loop modes. vnLimit and vpLimit registers 
 						     // determine the position limits. When clear: Virtual 
 						     // position limits are not used.
}MMFunc_t;


//======================================================================== 
// MMStatus_t: Status register bit definitions
//======================================================================== 
typedef enum MMStatus
{
 MMStatus_negLimit = 0x1,	// Set when the input NEG_LIM (J4 P16) is at 0V.
 
 MMStatus_posLimit = 0x02,	// Set when the input POS_LIM (J4 P17) is at 0V.
 
 MMStatus_brake = 0x04,		// Set when the input _BRAKE (J4 P12) is at 0V.
 
 MMStatus_index = 0x08,		// Set when the input IND/RC (J6 P2) is at 0V.
 
 MMStatus_badRc = 0x10,		// Set when the R/C pulse is not received within 
 							// 50ms, is shorter than RCMIN - 0.5ms, or is 
 							// longer than RCMAX + 0.5ms.
 							
 MMStatus_vnLimit = 0x20,	// Set when the negative virtual limit position 
 							// is reached.
 							
 MMStatus_vpLimit = 0x40		// Set when the positive virtual limit position 
 							// is reached.
}MMStatus_t;


//======================================================================== 
// MMData_t: A structure to hold contents of all registers in the device.
//======================================================================== 
typedef struct MMData
{
 int32_t position;
 int16_t velocityLimit;
 int8_t  velocityFf;
 int16_t function;
 int16_t pTerm;
 int16_t iTerm;
 int16_t dTerm;
 int8_t  address;
 int8_t  pidScalar;
 int8_t  timer;
 int16_t rcMax;
 int16_t rcMin;
 int16_t rcBand;
 int16_t rcCount;
 int16_t velocity;
 int32_t time;
 int16_t status;
 int8_t  revision;
 int8_t  mode;
 int16_t analogCon;
 int16_t analogFbck;
 int16_t pwmOut;
 int32_t indexPos;
#ifdef FIRMWARE_REVISION_5UP
 int32_t vnLimit;
 int32_t vpLimit;
 int16_t pwmLimit;
 int16_t deadband;
 int32_t desiredPosition;
#endif
}MMData_t;


//======================================================================== 
// class MotionMind
// ----------------------------------------------------------------------
// \brief
// A driver for the Solutions Cubed Motion Mind DC motor controller 
// Revision 4.
//
// The documentation in this file supplements the technical 
// manual for the motion mind controller which can be obtained 
// from http://www.solutions-cubed.com/solutions%20cubed/MM1_2005.htm . 
// This driver is designed for the controller released with 
// revision 4 of the above technical manual (late 2005). This driver class 
// provides a RS232 communication interface to the controller, and 
// is most useful for modes 3 (serial open-loop control) and 4 (serial 
// PID position control) specified in the controller manual. The user 
// must select the mode of operation by setting jumpers J5, and RS232 
// baud rate through J2.
//
// <b>Example Program:</b>
// \include MotionMind.t.cpp
//========================================================================  
 
class MotionMind
{
 public:
  MotionMind();
   // The default constructor does nothing. Call init() to connect
   // to the device.
   
  ~MotionMind();
   // The default destructor. Frees resources.
   
  int init(const char *port, int baud, int address, bool verbose=true);
   // Connects to the device through a specified serial port. Only 
   // RS232 binary data communication is supported. NOTE: This method
   // must be called before calling any other method to interact with the
   // hardware.
   // <ul>
   // <li> Make sure jumper pins in J2 are set to 19.2 KBPS baud rate 
   //      and Binary mode.
   // <li> Make sure that the PC's serial port is hooked to RS232 
   //      lines of the device.
   // <li> Ensure that mode of operation is selected through J5.
   // </ul>
   //  port     The serial port name (example /dev/ttyS0, /dev/ser1)
   //  baud     The baud select and motor direction lines are common in 
   //           connector J4. When the desired operating mode is 2 
   //           (button mode), the baud rate is selected based on 
   //           motor direction. Otherwise, it is advised to set baud
   //           rate to 19.2 KBPS. Specify either 9600 or 19200 for
   //           the selected baud rate here.
   //  address  Multiple contollers may be daisy chained. Specify which 
   //           device is being controlled by an instance of this object
   //           using this parameter. (address=1 if only one device present).
   //  verbose  Print device info if set to true.
   //  return   0 on success, -1 in case of error.
   
  int changeSpeed(int16_t speed);
   // Modifies speed and direction of motor when controller is 
   // operated in mode 3 (serial open loop control).
   // The device takes between 5 - 10 ms to respond to this command. 
   //  speed   Value in the range -1023 to +1023. The direction is 
   //          determined by the sign (- reverse, + forward) and the
   //          the absolute value sets the duty cycle of the input
   //          voltage (for example -512 => motor is driven in reverse 
   //          with 50% duty cycle). The timer register (MMReg_timer) 
   //          and the velocity limit register (MMReg_velocityLimit)  
   //          determines the ramp up time required to achieve the new speed. 
   //  return  0 on success, -1 on error.

  int moveToAbsolute(int32_t position);
   // Specify an absolute motor position when operating in mode 4 
   // (serial closed-loop). Ensure that the PID filter is tuned, and 
   // note that the encoder is decoded on a 4:1 ratio (i.e., if 
   // the encoder gives 500 pulses for 1 rotation of the shaft, send 
   // 2000 to rotate motor shaft by one full rotation.
   // The device takes between 5 - 10 ms to respond to this command. 
   //  position  Absolute position as a 32 bit 2's compliment number.
   //            Set velocity limit bit in the function register 
   //            (MMFunc_velocityLimit) and load velocity limit 
   //            register (mmcreg_velocityLimit) to limit average  
   //            velocity during motion.
   //  return    0 on success, -1 on error.

  int moveToRelative(int32_t position);
   // Specify movement to a position relative to current position, when 
   // operating in mode 4 (serial closed loop). Ensure that the PID filter  
   // is tuned, and note that the encoder is decoded on a 4:1 ratio 
   // (i.e., if the encoder gives 500 pulses for 1 rotation of the shaft, 
   // send 2000 to rotate motor shaft by one full rotation.
   // The device takes between 5 - 10 ms to respond to this command. 
   //  position  Relative position as a 32 bit 2's compliment number.
   //            Set velocity limit bit in the function register 
   //            (MMFunc_velocityLimit) and load velocity limit 
   //            register (MMReg_velocityLimit) to limit average velocity 
   //            during motion.
   //  return    0 on success, -1 on error.

  int moveAtVelocity(int16_t velocity);
   // Specify velocity when operating in mode 4 (serial closed loop).
   // The PID filter must be tuned to get best results.
   // The device takes between 5 - 10 ms to respond to this command.
   //  velocity  The desired velocity as a 2's compliment integer. 
   //            The velocity measurement in the controller occurs
   //            over a 5 ms period. Hence, the desired velocity
   //            setting can be computed from the following equation:
   //            velocity = (MotorShaftRotations/Second * 
   //                       EncoderCountPerRotation * GearRatio)/50.
   //            Use negative values for motion in reverse direction.
   //  return    0 on success, -1 on error.

  int writeRegister(MMReg_t reg, int32_t val);
   // This command can be used to set internal registers in any mode 
   // of operation. Register values set using this call are not retained
   // after power cycling the controller. Use writeStoreRegister() to make 
   // permanent changes to register contents. The registers are 1, 2 or
   // 4 bytes in length, and some of them expect 2's compliment data 
   // format. The user must take care to send properly formatted data to this
   // call. The function and status registers (MMReg_function, MMReg_status)
   // contain bit fields where each bit is related to a functionality.
   // See Section 6.0 (Register Definitions - Function/Status Bits) 
   // of the user manual for more informaiton.
   // The device takes between 5 - 10 ms to respond to this command. 
   // To enable a particular bit (example MMFunc_posPwUp) in the function
   // register, use this method as follows
   // \code
   // int16_t func = readRegister(MMReg_function);
   // writeRegister(MMReg_function, func | MMFunc_posPwUp);
   // \endcode   
   // In order to disable a functionality, the user code would look like the 
   // following
   // \code
   // int16_t func = readRegister(MMReg_function);
   // writeRegister(MMReg_function, func & ~MMFunc_posPwUp);
   // \endcode   
   //  reg       The register index.
   //  val       the desired value.
   //  return    0 on success, -1 on error (including attempted write 
   //            into a read-only register).

  int writeStoreRegister(MMReg_t reg, int32_t val);
   // This command can be used to set internal registers in any mode 
   // of operation. Register values set using this call are retained
   // after power cycling the controller, and hence, used to modify
   // default settings. Use writeRegister() to make temporary changes to 
   // register contents. The registers are 1, 2 or 4 bytes in length, and 
   // some of them expect 2's compliment data format. The user must take care 
   // to send properly formatted data to this call. The function register 
   // (MMReg_function) contains bit fields where each bit is related to a 
   // functionality. See Section 6.0 (Register Definitions - Function/Status 
   // Bits) of the user manual for more informaiton.
   // NOTE: The device takes up to 40 ms to respond to this command. 
   // Upon reception of a valid command, the motor is automatically 
   // stopped. The settings are made permanent by writing to the internal 
   // EEPROM which has a life time of about 1,000,000 write cycles. Avoid
   // overuse of this function call.
   //  reg       The register index.
   //  val       array with desired value (1,2, or 4 bytes).
   //  return    0 on success, -1 on error (including attempted write 
   //            into a read-only register).

  int32_t readRegister(MMReg_t reg);
   // Read contents of a register. The returned values may have 1, 2 or
   // 4 bytes of valid data, and may be in 2's compliment data 
   // format. See Section 6.0 (Register Definitions - Function/Status Bits) 
   // of the user manual for more informaiton.
   // The device takes between 5 - 10 ms to respond to this command. 
   // To check whether a particular bit (example MMFunc_posPwUp) is set in the 
   // function register, use this method as follows
   // \code
   // int16_t func = readRegister(MMReg_function);
   // func &= MMFunc_posPwUp;
   // if(func) {
   // cout << "Position will be restored from EEPROM on power up" << endl;
   // }
   // \endcode   
   // The status register can be read in the same manner.
   //  reg     The register index.
   //  return  register value. The return value is undefined if the call
   //          didn't successfully execute.
  
  int restore();
   // Restore factory defaults. The motor is automatically stopped 
   // after successful execution of this command
   // NOTE: The device takes up to 40 ms to respond to this command. 
   //  return    0 on success, -1 on error.

  int reset();
   // Stops the motor and does a software reset. Allow a few seconds before 
   // calling any other method after a call to this function.
   //  return    0 on success, -1 on error.

 protected:

  // ========== END OF INTERFACE ==========
 private:
  int send(char *data, int len);
   // Send command in 'data', 'len' bytes long to the controller
   //  return  0 on success, -1 on error.
   
  int sendAndReceive(char *inData, int inLen, char *outData, int outLen);
   // send command and receive a non-default response.
   //  inData   input data buffer.
   //  inLen    input data length.
   //  outData  output data buffer.
   //  outLen   length of data to read as response.
   //  return   0 on success, -1 on error.
   
  template <class T>
  void computeTwosComplement(T &in, T &out);
   // Compute the 2's complement of 'in' and place it in 'out'.
  
  void computeCheckSum(char *value, int numBytes, char &sum);
   // Compute check sum for 'value' array 'numBytes' long and 
   // place it in sum.

  bool d_isInit;         // true if init() returned successfully
  int8_t d_address;      // device address
  int d_port;            // serial port descriptor
  char d_buffer[8];      // buffer to hold commands for write..()
};


//======================================================================== 
// MotionMind::computeTwosComplement
//======================================================================== 
template <class T>
void MotionMind::computeTwosComplement(T &in, T &out)
{
 (in >= 0) ? (out = in) : (out =  (~abs(in) + 1));
}


#endif

