#======================================================================== 
# Project: Motion Mind DC Motor Controller
# ---------------------------------------------------------------------- 
# Package: MotionMind
# Authors: Nitendra Nath, Vilas Chitrakaran  
# Start Date: Sun Jan 15 15:36:46 EST 2006
# ----------------------------------------------------------------------  
# File: makefile (makefile of examples)
#========================================================================  

# ---- compiler options ----
os=linux
CC = g++
LD = g++
ifeq ($(os),qnx)
 CFLAGS += -DNTO
endif
CFLAGS += -Wall -fexceptions -fno-builtin -D_REENTRANT -O2 -fpic -c
LDFLAGS = -fexceptions -O2 -o
INCLUDEHEADERS = -I ../ -I /usr/local/include -I /opt/include/SDL \
                 -I /usr/qrts/include
INCLUDELIBS = -L ../ -L /usr/local/lib -L /opt/lib -L /usr/qrts/lib \
	          -lMotionMind -lputils

ifeq ($(os),qnx)
	INCLUDELIBS += -lsocket
else 
	INCLUDELIBS += -lpthread -lrt
endif

OBJ = 
TARGET = MotionMind.t MMClientServer.t MotionMindServer
CLEAN = rm -rf *.o lib* *.dat $(TARGET)


# ========== Targets ==========
targets: $(TARGET)

# ----- MotionMind -----
MotionMind.t: MotionMind.t.cpp
	$(CC) $(CFLAGS) MotionMind.t.cpp $(INCLUDEHEADERS)
	$(LD) $(LDFLAGS) MotionMind.t MotionMind.t.o $(INCLUDELIBS)

# ----- MMClientServer -----
MMClientServer.t: MMClientServer.t.cpp
	$(CC) $(CFLAGS) MMClientServer.t.cpp $(INCLUDEHEADERS)
	$(LD) $(LDFLAGS) MMClientServer.t MMClientServer.t.o $(INCLUDELIBS)

# ----- MotionMindServer -----
MotionMindServer: MotionMindServer.t.cpp
	$(CC) $(CFLAGS) MotionMindServer.t.cpp $(INCLUDEHEADERS)
	$(LD) $(LDFLAGS) MotionMindServer MotionMindServer.t.o $(INCLUDELIBS)

clean:
	$(CLEAN)

