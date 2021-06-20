# included modules
include ${BASEDIR}/uavcan/can/can.mk
include ${BASEDIR}/uavcan/libcanard/libcanard.mk
include ${BASEDIR}/uavcan/types/types.mk

# List of all the related files.
UAVCANSRC =     ${LIBCANARDCANSRC} \
                ${LIBCANARDSRC} \
				
UAVCANCPPSRC = ../../uavacan/uavcan.cpp


# Required include directories
UAVCANINC =     ${LIBCANARDCANINC} \
				${LIBCANARDINC} \
				${UAVCANTYPESINC} \
		   