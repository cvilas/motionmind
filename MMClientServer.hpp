//======================================================================== 
// Project: Motion Mind DC Motor Controller
// ---------------------------------------------------------------------- 
// Package: MMClient: Client class for MMServer
// Authors: Vilas Chitrakaran
// Start Date: Fri Mar 03 14:23:02 EST 2006
// Compiler: GNU GCC 2.95.3 and above
// Operating System: QNX RTP, Linux
// ----------------------------------------------------------------------  
// File: MMClientServer.hpp
// Interface of the class MMClient and MMServer.
//========================================================================  
 

#ifndef INCLUDED_MMCLIENTSERVER_HPP
#define INCLUDED_MMCLIENTSERVER_HPP

#include "MotionMind.hpp"
#include "putils/ShMem.hpp"
#include "putils/StatusReport.hpp"
#include <time.h>
#include <signal.h>
#include <semaphore.h>

#define MMCHANGESPEED 0x01
#define MMMOVETOABS   0x02
#define MMMOVETOREL   0x03
#define MMMOVEATVEL   0x04

struct MMCommand
{
 sem_t guard;      // semaphore
 char command;     // command byte
 int32_t data;     // data byte
 int32_t position; // motor position update
 char status;      // server status, 0 if OK, -1 if error
};

//======================================================================== 
// class MMClient
// ----------------------------------------------------------------------
// \brief
// A non-blocking client to MMServer
//
// Use an object of this class to command the Motion Mind Controller
// through a server. The functionality is restricted to setting speed and
// position using modes 3 or 4 of the controller (serial open loop, and
// serial closed loop PID, respectively)
//
// <b>Example Program:</b>
// \include MMClientServer.t.cpp
//========================================================================  
 
class MMClient
{
 public:
  MMClient(char *shmName = "mmc0");
   // The default constructor. Initiates connection with server
   //  shmName  The server (MMServer class) identifier
   
  ~MMClient ();
   // The default destructor. Disconnects from server.
   
  int isServerStatusOk();
   //  return  0 if server status is OK, else -1.

  void changeSpeed(int16_t speed);
   // Modifies speed and direction of motor when controller is 
   // operated in mode 3 (serial open loop control).
   //  speed   Value in the range -1023 to +1023. The direction is 
   //          determined by the sign (- reverse, + forward) and the
   //          the absolute value sets the duty cycle of the input
   //          voltage (for example -512 => motor is driven in reverse 
   //          with 50% duty cycle). The timer register (MMReg_timer) 
   //          and the velocity limit register (MMReg_velocityLimit)  
   //          determines the ramp up time required to achieve the new speed. 

  void moveToAbsolute(int32_t position);
   // Specify an absolute motor position when operating in mode 4 
   // (serial closed-loop). Ensure that the PID filter is tuned, and 
   // note that the encoder is decoded on a 4:1 ratio (i.e., if 
   // the encoder gives 500 pulses for 1 rotation of the shaft, send 
   // 2000 to rotate motor shaft by one full rotation.
   //  position  Absolute position as a 32 bit 2's compliment number.
   //            Set velocity limit bit in the function register 
   //            (MMFunc_velocityLimit) and load velocity limit 
   //            register (mmcreg_velocityLimit) to limit average  
   //            velocity during motion.

  void moveToRelative(int32_t position);
   // Specify movement to a position relative to current position, when 
   // operating in mode 4 (serial closed loop). Ensure that the PID filter  
   // is tuned, and note that the encoder is decoded on a 4:1 ratio 
   // (i.e., if the encoder gives 500 pulses for 1 rotation of the shaft, 
   // send 2000 to rotate motor shaft by one full rotation.
   //  position  Relative position as a 32 bit 2's compliment number.
   //            Set velocity limit bit in the function register 
   //            (MMFunc_velocityLimit) and load velocity limit 
   //            register (MMReg_velocityLimit) to limit average velocity 
   //            during motion.

  void moveAtVelocity(int16_t velocity);
   // Specify velocity when operating in mode 4 (serial closed loop).
   // The PID filter must be tuned to get best results.
   //  velocity  The desired velocity as a 2's compliment integer. 
   //            The velocity measurement in the controller occurs
   //            over a 5 ms period. Hence, the desired velocity
   //            setting can be computed from the following equation:
   //            velocity = (MotorShaftRotations/Second * 
   //                       EncoderCountPerRotation * GearRatio)/50.
   //            Use negative values for motion in reverse direction.

  int32_t getPosition();
   // The client continously receives an update for actual motor 
   // position. This position data is updated only as fast as the 
   // server can provide this information, and hence, not guaranteed
   // to be exactly correct. Use this method to get a rough estimate
   // for motor position.
   
 protected:
 private:
  ShMem d_shm;
  struct MMCommand *d_data;
  int d_serverStatus;
  int32_t d_position;
};


//======================================================================== 
// class MMServer
// ----------------------------------------------------------------------
// \brief
// Provides a server for motion mind controller
//
// A client (MMClient class) can connect to an object of this class to 
// set desired velocities and positions through a shared memory interface.
// Use a separate program to set internal registers in the board.
//
// <b>Example Program:</b>
// See example program for MMClient
//========================================================================  
 
class MMServer
{
 public:
  MMServer(const char *serverName, const char *port, int baud, int address, bool verbose=true);
   // Default constructor
   //  serverName  Name of the server
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
   
  ~MMServer ();
   // Destructor.
  
  int getStatusCode() const;
   //  return  0 if server status is OK, else -1.
  
  const char *getStatusMessage() const;
   //  return  status message.
  
  int setPIDGains(int p, int i, int d);
   // Set the P, I and D gains
   //  return  0 on success, -1 on error.
    
  void doMainLoop(int updateDelay);
   // Main loop of the server.
   //  updateDelay  Delay (ms) between every update
   //               cycle.
	 
 protected:
 private:
  static void signalHandler(int signal);
   // Receives SIGINT, resets motor controller.
  static bool d_terminate;
  ShMem d_shm;
  MotionMind d_controller;
  struct MMCommand *d_data;
  StatusReport d_status;
};


#endif // INCLUDED_MMCLIENTSERVER_HPP


