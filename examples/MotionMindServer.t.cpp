//======================================================================== 
// Package: Motion Mind DC Motor Controller
// Authors: Vilas Kumar Chitrakaran, Nitendra Nath
// Start Date: Sun Jan 15 15:36:46 EST 2006
// ----------------------------------------------------------------------  
// File: MMServer.t.cpp
// Example program for the class MMServer.
//========================================================================  
 

#include "MotionMind/MMClientServer.hpp"
#include <unistd.h>
#include <stdio.h>
#include <pthread.h>
 
//======================================================================== 
// usage
//========================================================================  
void usage(char *name)
{
 fprintf(stderr, "Usage: %s -s <serial_port> -b <baud> -n <name> \n \
 -a <address> -m <loop_ms_delay> -p <p gain> -i <i gain> -d <d gain> \n\
 -f <priority> \n", name);
}

//======================================================================== 
// main
//========================================================================  
int main(int argc, char *argv[])
{ 
 struct sched_param param;
 int policy;
 int p = 6000;
 int i = 35;
 int d = 200;
 
 char iobsname[80];
 int opt;
 char port[80];
 char shmName[80];
 int priority = 10;
 int address = 1;
 int baud = 19200;
 int msDelay = 500;
 snprintf(port, 80, "/dev/ser1");
 snprintf(shmName, 80, "/mmc0");
 
 // get command line options
 while( (opt = getopt(argc, argv, "s:b:n:a:m:p:i:d:f:")) != -1) {
  switch(opt) {
   case 's':
    snprintf(port, 80, optarg);
    break;
   case 'b':
    baud = atoi(optarg);
    break;
   case 'a':
    address = atoi(optarg);
    break;
   case 'm':
    msDelay = atoi(optarg);
    break;
   case 'n':
    snprintf(shmName, 80, optarg);
    break;
   case 'f':
    priority = atoi(optarg);
    break;
   case 'p':
    p = atoi(optarg);
    break;
   case 'i':
    i = atoi(optarg);
    break;
   case 'd':
    d = atoi(optarg);
    break;
   case ':':
   case '?':
    usage(argv[0]);
    return -1;
  }
 }
 
 fprintf(stdout, "[%s] starting server %s for device on port %s, (address %d, baud %d).\n", 
         argv[0], shmName, port, address, baud);

 // set priority
 pthread_getschedparam(pthread_self(), &policy, &param);
 policy = SCHED_FIFO;
 param.sched_priority = priority;
 opt = pthread_setschedparam(pthread_self(), policy, &param); 
 if(opt != 0) {
  fprintf(stderr, "[%s] ERROR setting priority: %s\n", argv[0], strerror(opt));
 }
 
 MMServer server(shmName, port, baud, address);
 server.setPIDGains(p,i,d);
 if(server.getStatusCode() != 0) {
  fprintf(stderr, "[%s]: ERROR in server: %s\n", argv[0], server.getStatusMessage());
  return -1;
 }
 server.doMainLoop(msDelay);
 return 0;
}

