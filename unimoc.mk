# Base directory of Project
BASEDIR = ../../

# List of all the Project related hardware independent files.
# C code
UNIMOCSRC = 
# Cpp code
UNIMOCCPPSRC = ${BASEDIR}/src/main.cpp \

# Required include directories
UNIMOCINC = ${BASEDIR}/inc \

# Shared variables
ALLCPPSRC += ${UNIMOCCPPSRC}
ALLCSRC += $(UNIMOCSRC)
ALLINC  += $(UNIMOCINC)