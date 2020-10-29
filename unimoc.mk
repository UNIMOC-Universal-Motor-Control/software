# Base directory of Project
BASEDIR := ../../

# included modules
include ${BASEDIR}/modules/freemaster/freemaster.mk

# List of all the Project related hardware independent files.
# C code
UNIMOCSRC := ${FREEMASTERSRC} \

# Cpp code
UNIMOCCPPSRC := ${FREEMASTERCPPSRC} \
                ${BASEDIR}/src/main.cpp \
                ${BASEDIR}/src/controller.cpp \
                ${BASEDIR}/src/filter.cpp \
                ${BASEDIR}/src/observer.cpp \
                ${BASEDIR}/src/settings.cpp \
                ${BASEDIR}/src/systems.cpp \
                ${BASEDIR}/src/values.cpp \
                ${BASEDIR}/src/management.cpp \
                ${BASEDIR}/src/pas.cpp \

# Required include directories
UNIMOCINC := ${FREEMASTERINC} \
             ${BASEDIR}/inc \
             ${BASEDIR}/hardware/interface \

# Shared variables
ALLCPPSRC += ${UNIMOCCPPSRC}
ALLCSRC += $(UNIMOCSRC)
ALLINC  += $(UNIMOCINC)