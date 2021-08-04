# Base directory of Project
BASEDIR := ../../

# included modules
include ${BASEDIR}/uavcan/uavcan.mk

# List of all the Project related hardware independent files.
# C code
UNIMOCSRC := ${UAVCANSRC}

# Cpp code
UNIMOCCPPSRC := ${BASEDIR}/src/main.cpp \
                ${BASEDIR}/src/controller.cpp \
                ${BASEDIR}/src/filter.cpp \
                ${BASEDIR}/src/observer.cpp \
                ${BASEDIR}/src/settings.cpp \
                ${BASEDIR}/src/systems.cpp \
                ${BASEDIR}/src/values.cpp \
                ${BASEDIR}/src/management.cpp \
                ${BASEDIR}/src/control_thread.cpp \
                ${BASEDIR}/src/measurement.cpp \
                ${UAVCANCPPSRC} \

# Required include directories
UNIMOCINC := ${BASEDIR}/inc \
             ${BASEDIR}/hardware/interface \
             ${UAVCANINC} \

# Shared variables
ALLCPPSRC += ${UNIMOCCPPSRC}
ALLCSRC += $(UNIMOCSRC)
ALLINC  += $(UNIMOCINC)