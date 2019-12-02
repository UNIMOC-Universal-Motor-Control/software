# Base directory of Project
BASEDIR := ../../

# List of all the Project related hardware independent files.
# C code
UNIMOCSRC := ${BASEDIR}/libcanard/canard.c \
             ${BASEDIR}/libcanard/drivers/stm32/canard_stm32.c \
# Cpp code
UNIMOCCPPSRC := ${BASEDIR}/src/main.cpp \

# Required include directories
UNIMOCINC := ${BASEDIR}/inc \
             ${BASEDIR}/libcanard \
             ${BASEDIR}/libcanard/drivers/stm32 \
                        

# Shared variables
ALLCPPSRC += ${UNIMOCCPPSRC}
ALLCSRC += $(UNIMOCSRC)
ALLINC  += $(UNIMOCINC)