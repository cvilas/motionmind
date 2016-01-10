//======================================================================== 
// Package: Motion Mind DC Motor Controller
// Authors: Vilas Kumar Chitrakaran, Nitendra Nath
// Start Date: Sun Jan 15 15:36:46 EST 2006
// ----------------------------------------------------------------------  
// File: MMClientServer.t.cpp
//========================================================================  

#include <stdio.h>
#include "MMClientServer.hpp"
#include <pthread.h>
#include <unistd.h>
#include <math.h>

 
//======================================================================== 
// client 
//========================================================================  
int client()
{
 int t = 0;
 MMClient mmClient("/mmc0");
 if(mmClient.isServerStatusOk() != 0) {
  fprintf(stderr, "[client]: ERROR in server.\n");
  return -1;
 }

 while(1) { 
  if(mmClient.isServerStatusOk() != 0) {
   fprintf(stderr, "[client]: ERROR in server.\n");
   return -1;
  }
  mmClient.moveAtVelocity((int)(sin((float)t) * 100.0));
  fprintf(stdout, "%d\n", mmClient.getPosition());
  t+=1;
  sleep(1);
 }
 return 0;
}

//======================================================================== 
// server
//========================================================================  
void *server(void *arg)
{ 
 MMServer mmServer("/mmc0", "/dev/ser1", 19200, 1);
 if(mmServer.getStatusCode() != 0) {
  fprintf(stderr, "[server]: %s\n", mmServer.getStatusMessage());
  return (void *)(-1);
 }
 mmServer.doMainLoop(10);
 return 0;
}


//==============================================================================
// main function
//==============================================================================
int main()
{
 pthread_t threadId;
 pthread_create(&threadId, NULL, &server, NULL);
 sleep(1);
 client();
 return 0;
}
