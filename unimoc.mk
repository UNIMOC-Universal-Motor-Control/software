# Base directory of Project
BASEDIR := ../../

# UAVCAN Lib files
include ${BASEDIR}/libuavcan.mk 

# List of all the Project related hardware independent files.
# C code
UNIMOCSRC := 
# Cpp code
UNIMOCCPPSRC := ${BASEDIR}/src/main.cpp \
                $(LIBUAVCAN_SRC) \

# Required include directories
UNIMOCINC := ${BASEDIR}/inc \
             $(LIBUAVCAN_INC)

# Shared variables
ALLCPPSRC += ${UNIMOCCPPSRC}
ALLCSRC += $(UNIMOCSRC)
ALLINC  += $(UNIMOCINC)