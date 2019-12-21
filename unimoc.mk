# Base directory of Project
BASEDIR := ../../

# List of all the Project related hardware independent files.
# C code
UNIMOCSRC := ${BASEDIR}/src/main.c \

# Cpp code
UNIMOCCPPSRC := 

# Required include directories
UNIMOCINC := ${BASEDIR}/inc \

# Shared variables
ALLCPPSRC += ${UNIMOCCPPSRC}
ALLCSRC += $(UNIMOCSRC)
ALLINC  += $(UNIMOCINC)