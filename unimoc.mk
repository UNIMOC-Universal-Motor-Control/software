# Base directory of Project
BASEDIR := ../../

# included modules
include ${BASEDIR}/libcanard/libcanard.mk  

# List of all the Project related hardware independent files.
# C code
UNIMOCSRC := ${LIBCANARDSRC}  

# Cpp code
UNIMOCCPPSRC := ${LIBCANARDCPPSRC} \
                ${BASEDIR}/src/main.cpp \
                ${BASEDIR}/src/controller.cpp \
                ${BASEDIR}/src/filter.cpp \
                ${BASEDIR}/src/observer.cpp \
                ${BASEDIR}/src/settings.cpp \
                ${BASEDIR}/src/systems.cpp \
                ${BASEDIR}/src/values.cpp \
                ${BASEDIR}/src/uavcan.cpp \

# Required include directories
UNIMOCINC := ${LIBCANARDINC} \
             ${BASEDIR}/inc \
             ${BASEDIR}/hardware/interface \

# Shared variables
ALLCPPSRC += ${UNIMOCCPPSRC}
ALLCSRC += $(UNIMOCSRC)
ALLINC  += $(UNIMOCINC)