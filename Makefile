# Copyright (c) 2015 Qualcomm Technologies, Inc.  All Rights Reserved.
# Qualcomm Technologies Proprietary and Confidential.

CC = arm-linux-gnueabihf-gcc
CXX = arm-linux-gnueabihf-g++

CFLAGS = -Wall -Wno-unused-variable -Wno-unused-but-set-parameter -Wno-unused-but-set-variable
CXXFLAGS = -Wall -Wno-unused-variable -Wno-unused-but-set-parameter -Wno-unused-but-set-variable -std=c++0x

LIB_DIR = ./lib
INCLUDES	= -I ./inc
			
LIBS	=	-lsnav_arm \
				-ladsprpc \
				-lsensor_imu \
				-ladreno_utils \
				-lCB \
				-lEGL_adreno \
				-lGLESv2_adreno \
				-lgsl \
				-lmv \
				-lOpenCL \
				-lsc-a3xx \
				-L./lib \
				
LIBS	+=	$(LIB_DIR)/libcamera.so.0 \
					$(LIB_DIR)/libcamparams.so.0 \
					$(LIB_DIR)/libcrypto.so.1.0.0 \
					$(LIB_DIR)/libssl.so.1.0.0 \
					$(LIB_DIR)/libstdc++.so.6 \
					$(LIB_DIR)/libz.so.1 \
					$(LIB_DIR)/libglib-2.0.so.0 \
										

TARGET = flightctrl_proxy  uf1_calibration_test ota

all: $(TARGET)

% : %.cpp 
	$(CXX) $(CXXFLAGS) $< -o $@ $(INCLUDES) -lpthread   $(LIBS)	
	
% : %.c 
	$(CC) $(CFLAGS) $< -o $@ $(INCLUDES) -lpthread   $(LIBS)

clean:
	rm -f $(TARGET)

