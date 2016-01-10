//======================================================================== 
// Package: Motion Mind DC Motor Controller
// Authors: Vilas Kumar Chitrakaran, Nitendra Nath
// Start Date: Sun Jan 15 15:36:46 EST 2006
// ----------------------------------------------------------------------  
// File: MotionMind.t.cpp
// Example program for the class MotionMind.
//========================================================================  
 

#include <stdio.h>
#include <unistd.h>
#include "MotionMind.hpp"

//======================================================================== 
// main - An example PID control program
//========================================================================  
int main()
{
 MotionMind controller;
 int ret = 0;
 
 // connect to the controller
 ret = controller.init("/dev/ser1", 19200, 1);
 if(ret == -1) {
  fprintf(stderr, "Connection error\n");
  return(-1);
 }
 
 // set gains p = 6000, I = 35, D = 200
 if( controller.writeRegister(MMReg_pTerm, 6000) == -1 ) {
  fprintf(stderr, "[main] ERROR setting P gain\n");
 }
 if( controller.writeRegister(MMReg_iTerm, 35) == -1 ) {
  fprintf(stderr, "[main] ERROR setting I gain\n");
 }
 if( controller.writeRegister(MMReg_dTerm, 200) == -1 ) {
  fprintf(stderr, "[main] ERROR setting D gain\n");
 }
 
 // read current position
 fprintf(stdout, "current position: %d\n", (unsigned int) controller.readRegister(MMReg_position));

 // send desired position (in encoder counts x 4)
 if( controller.moveToAbsolute(5000) == -1 ) {
  fprintf(stderr, "[main] ERROR sending desired position\n");
 }
  
 // wait
 sleep(4);
 
 // bye (controller automatically resets and stops motor on exit)
 
 return 0;
}

