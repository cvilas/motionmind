#======================================================================== 
# Project: Motion Mind DC Motor Controller
# ---------------------------------------------------------------------- 
# Package: MotionMind 
# Authors: Nitendra Nath, Vilas Chitrakaran  
# Start Date: Sun Jan 15 15:36:46 EST 2006
# ----------------------------------------------------------------------  
# File: makefile
#========================================================================  

#Name of the package
PKG = MotionMind

# ----- Directories -----
INSTALLDIR = /usr/local
INSTALLHEADERPATH= $(INSTALLDIR)/include/$(PKG)
INSTALLLIBPATH= $(INSTALLDIR)/lib
INSTALLBINPATH =
INSTALLBSRCPATH = 

# ----- Doxygen documentation parameters -----
DOCNAME = Motion Mind DC Motor Controller
DOCSOURCE = *.hpp
DOCTARGET = 

# Libraries, headers, and binaries that will be installed.
os=linux
LIBS = lib$(PKG).a lib$(PKG).so
HDRS = MotionMind.hpp MMClientServer.hpp
#SRC = *.cpp

# ---- compiler options ----
CC = g++
LD = g++
ifeq ($(os),qnx)
 CFLAGS += -DNTO
endif
CFLAGS += -W -Wall -fexceptions -fno-builtin -O2 -fpic -D_REENTRANT
LDFLAGS = 
INCLUDEHEADERS = -I ./ -I /usr/local/include -I /usr/qrts/include 
INCLUDELIBS = -L /usr/local/lib -lputils
OBJ = MotionMind.o MMClientServer.o
TARGET = $(LIBS)
CLEAN = rm -rf *.o *.dat $(TARGET)


# ========== Targets ==========
targets: $(TARGET)

# ----- lib -----
lib$(PKG).a: $(OBJ)
	ar cr $@ $(OBJ)
	ranlib $@

lib$(PKG).so: $(OBJ)
	$(LD) -shared -o $@ $(OBJ)

.cpp.o:
	$(CC) $(CFLAGS) -c $< $(INCLUDEHEADERS)

# ---- make rules ----
clean:
	@echo
	@echo ----- Package $(PKG), Cleaning -----
	@echo
	$(CLEAN)
	if (test -d examples) ; then (cd examples; make clean);fi

install:
	@echo
	@echo ----- Package $(PKG), Installing to $(INSTALLDIR) -----
	@echo
	if ! test -d $(INSTALLLIBPATH) ; then (mkdir $(INSTALLLIBPATH)); fi
	for i in ${LIBS}; do (cp $$i $(INSTALLLIBPATH)); done
	if ! test -d $(INSTALLHEADERPATH) ; then (mkdir $(INSTALLHEADERPATH)); fi
	for i in ${HDRS}; do (cp $$i $(INSTALLHEADERPATH)); done

