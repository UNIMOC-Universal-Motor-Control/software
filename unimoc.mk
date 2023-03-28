# Base directory of Project
BASEDIR := ../../

# included modules
include ${BASEDIR}/cyphal/cyphal.mk

# List of all the Project related hardware independent files.
# C code
UNIMOCSRC := ${CYPHALSRC}

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
                ${BASEDIR}/src/display.cpp \
                ${BASEDIR}/src/pas.cpp \
                ${CYPHALCPPSRC} \

# Required include directories
UNIMOCINC := ${BASEDIR}/inc \
             ${BASEDIR}/hardware/interface \
             ${CYPHALINC} 
             
UNIMOCXASM := 

# Shared variables
ALLCPPSRC += ${UNIMOCCPPSRC}
ALLCSRC += $(UNIMOCSRC)
ALLINC  += $(UNIMOCINC)
ALLXASMSRC  += ${UNIMOCXASM}