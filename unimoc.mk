# Base directory of Project
BASEDIR := ../../

# included modules
include ${BASEDIR}/modules/freemaster/freemaster.mk 

# List of all the Project related hardware independent files.
# C code
UNIMOCSRC := ${FREEMASTERSRC}

# Cpp code
UNIMOCCPPSRC := ${FREEMASTERCPPSRC} \
                ${BASEDIR}/src/main.cpp \

# Required include directories
UNIMOCINC := ${FREEMASTERINC} \
             ${BASEDIR}/inc \
             ${BASEDIR}/hardware/interface \

# Shared variables
ALLCPPSRC += ${UNIMOCCPPSRC}
ALLCSRC += $(UNIMOCSRC)
ALLINC  += $(UNIMOCINC)