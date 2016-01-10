//======================================================================== 
// Project: Motion Mind DC Motor d_controller
// ---------------------------------------------------------------------- 
// Package: MMClient: Client class for MMServer
// Authors: Vilas Chitrakaran
// Start Date: Fri Mar 03 14:23:02 EST 2006
// ----------------------------------------------------------------------  
// File: MMClient.cpp
// Implementation of the class MMClient.
// See MMClient.hpp for more details.
//========================================================================  
 

#include "MMClientServer.hpp"
#include <stdio.h> 

//#define DEBUG

//======================================================================== 
// MMClient::MMClient
//========================================================================  
MMClient::MMClient (char *shmName)
{
 // open existing shared memory
 d_data = (struct MMCommand *)d_shm.open( shmName, sizeof(struct MMCommand) );
 if( d_data == NULL ) {
  fprintf(stderr, "[MMClient::MMClient]: Unable to connect to server: %s\n", 
  strerror(d_shm.getErrnoError()));
  d_serverStatus = -1;
  return;
 }
 sem_wait(&d_data->guard);
 d_serverStatus = d_data->status;
 sem_post(&d_data->guard);
}


//======================================================================== 
// MMClient::~MMClient
//========================================================================  
MMClient::~MMClient ()
{
 moveAtVelocity(0);
 d_shm.close();
}


//======================================================================== 
// MMClient::isServerStatusOk
//========================================================================  
int MMClient::isServerStatusOk()
{
 if(!d_data)
  return -1;
 sem_wait(&d_data->guard);
 d_serverStatus = d_data->status;
 sem_post(&d_data->guard);
 return d_serverStatus;
}


//======================================================================== 
// MMClient::changeSpeed
//========================================================================  
void MMClient::changeSpeed(int16_t speed)
{
 if(!d_data)
  return;
 sem_wait(&d_data->guard);
 d_data->command = MMCHANGESPEED;
 d_data->data = speed & 0xFFFF;
 sem_post(&d_data->guard);
}


//======================================================================== 
// MMClient::moveToAbsolute
//========================================================================  
void MMClient::moveToAbsolute(int32_t position)
{
 if(!d_data)
  return;
 sem_wait(&d_data->guard);
 d_data->command = MMMOVETOABS;
 d_data->data = position;
 sem_post(&d_data->guard);
}


//======================================================================== 
// MMClient::moveToRelative
//========================================================================  
void MMClient::moveToRelative(int32_t position)
{
 if(!d_data)
  return;
 sem_wait(&d_data->guard);
 d_data->command = MMMOVETOREL;
 d_data->data = position;
 sem_post(&d_data->guard);
}


//======================================================================== 
// MMClient::moveAtVelocity
//========================================================================  
void MMClient::moveAtVelocity(int16_t velocity)
{
 if(!d_data)
  return;
 sem_wait(&d_data->guard);
 d_data->command = MMMOVEATVEL;
 d_data->data = velocity;
 sem_post(&d_data->guard);
}


//======================================================================== 
// MMClient::getPosition
//========================================================================  
int32_t MMClient::getPosition()
{
 if(!d_data)
  return d_position;
 sem_wait(&d_data->guard);
 d_position = d_data->position;
 sem_post(&d_data->guard);
 return d_position;
}


//======================================================================== 
// MMServer::MMServer
//========================================================================  
bool MMServer::d_terminate = false; 
MMServer::MMServer (const char *shmName, const char *port, int baud, int address, bool verbose)
{
 d_terminate = false;
 d_status.setReport(-1, "Not initialized");
 
 // create shared memory
 d_data = (struct MMCommand *)d_shm.create( shmName, sizeof(struct MMCommand) );
 if( d_data == NULL ) {
  d_status.setReport(-1, "[MMServer::MMServer]: Unable to create shared memory");
  fprintf(stderr, "%s: %s\n", d_status.getReportMessage(), strerror(d_shm.getErrnoError()));
  return;
 }

 // initialize semaphore
 d_data->status = -1;
 if( sem_init(&d_data->guard, 1, 1) < 0 ) {
  d_status.setReport(-1, "[MMServer::MMServer] ERROR initializing semaphore");
  fprintf(stderr, "%s: %s\n", d_status.getReportMessage(), strerror(errno));
  return;
 }

 // initialize controller
 if( (d_controller.init(port, baud, address, verbose)) == -1) {
  d_status.setReport(-1, "[MMServer::MMServer] ERROR connecting to board");
  fprintf(stderr, "%s\n", d_status.getReportMessage());
  return;
 }
 
 // set gains p = 6000, I = 35, D = 200
 if ( setPIDGains(6000, 35, 200) == -1) {
  return;
 }
 
 // A signal handler for SIGINT
 signal(SIGINT, signalHandler);

 d_status.setReport(0, "No errors");
 d_data->status = 0;
 d_data->position = 0;
}


//======================================================================== 
// MMServer::~MMServer
//========================================================================  
MMServer::~MMServer ()
{
 sem_destroy(&d_data->guard);
 d_shm.unlink();
}


//======================================================================== 
// MMServer::getStatusCode
//========================================================================  
int MMServer::getStatusCode () const
{
 return d_status.getReportCode();
}


//======================================================================== 
// MMServer::getStatusMessage
//========================================================================  
const char *MMServer::getStatusMessage() const
{
 return d_status.getReportMessage();
}


//======================================================================== 
// MMServer::setPIDGains
//========================================================================  
int MMServer::setPIDGains(int p, int i, int d)
{
 int ret = 0;
 if( d_controller.writeRegister(MMReg_pTerm, p) == -1 ) {
  d_status.setReport(-1, "[MMServer::setPIDGains] ERROR setting P gain");
  fprintf(stderr, "%s\n", d_status.getReportMessage());
  ret = -1;
 }
 if( d_controller.writeRegister(MMReg_iTerm, i) == -1 ) {
  d_status.setReport(-1, "[MMServer::setPIDGains] ERROR setting I gain");
  fprintf(stderr, "%s\n", d_status.getReportMessage());
  ret = -1;
 }
 if( d_controller.writeRegister(MMReg_dTerm, d) == -1 ) {
  d_status.setReport(-1, "[MMServer::setPIDGains] ERROR setting D gain");
  fprintf(stderr, "%s\n", d_status.getReportMessage());
  ret = -1;
 }
 return ret;
}


//======================================================================== 
// MMServer::doMainLoop
//========================================================================  
void MMServer::doMainLoop(int msDelay)
{
 if(!d_data)
  return;
 struct timespec napTime;
 napTime.tv_sec = 0;
 napTime.tv_nsec = (long int)(msDelay * 1e6);

 int32_t dTmp, cTmp, sTmp = 0;

 while(!d_terminate) {

  sem_wait(&d_data->guard);
  dTmp = d_data->data;
  cTmp = d_data->command;
  d_data->command = 0x0;
  sem_post(&d_data->guard);

#ifdef DEBUG
  fprintf(stderr, "command %d\n", cTmp);
#endif

  switch(cTmp) {
   case MMCHANGESPEED:
    if( d_controller.changeSpeed(dTmp & 0xFFFF) == -1 ) {
     d_status.setReport(-1, "[MMServer::doMainLoop] ERROR changing speed");
     fprintf(stderr, "%s\n", d_status.getReportMessage());
     sTmp = -1;
    } else
     sTmp = 0;
   break;
   case MMMOVETOABS: 
    if( d_controller.moveToAbsolute(dTmp) == -1 ) {
     d_status.setReport(-1, "[MMServer::doMainLoop] ERROR changing absolute position");
     fprintf(stderr, "%s\n", d_status.getReportMessage());
     sTmp = -1;
    } else
     sTmp = 0;
   break;
   case MMMOVETOREL:
    if( d_controller.moveToRelative(dTmp) == -1 ) {
     d_status.setReport(-1, "[MMServer::doMainLoop] ERROR changing relative position");
     fprintf(stderr, "%s\n", d_status.getReportMessage());
     sTmp = -1;
    } else
     sTmp = 0;
   break;
   case MMMOVEATVEL:
    if( d_controller.moveAtVelocity(dTmp & 0xFFFF) == -1 ) {
     d_status.setReport(-1, "[MMServer::doMainLoop] ERROR changing velocity");
     fprintf(stderr, "%s\n", d_status.getReportMessage());
     sTmp = -1;
    } else
     sTmp = 0;
   break;
  }
  
  // read position
  dTmp = d_controller.readRegister(MMReg_position);
  cTmp = 0;
  
  sem_wait(&d_data->guard);
  d_data->status = sTmp;
  d_data->position = dTmp;
  sem_post(&d_data->guard);

  nanosleep(&napTime, NULL);
 } // end while()
 
 if( d_controller.reset() != 0) {
  d_status.setReport(-1, "[MMServer::doMainLoop] ERROR reseting controller");
  fprintf(stderr, "%s\n", d_status.getReportMessage());
 }
}


//======================================================================== 
// MMServer::signalHandler
//======================================================================== 
void MMServer::signalHandler(int signo)
{
 if(signo != SIGINT)
  return;

 fprintf(stderr, "%s\n", "[MMServer::signalHandler] SIGINT received");
 MMServer::d_terminate = true;
 signal(SIGINT, SIG_DFL);

 return;
}

