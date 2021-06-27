# included modules
include ${BASEDIR}/uavcan/can/can.mk
include ${BASEDIR}/uavcan/libcanard/libcanard.mk
include ${BASEDIR}/uavcan/types/types.mk
include ${BASEDIR}/uavcan/01heap/o1heap.mk

# List of all the related files.
UAVCANSRC =     ${LIBCANARDCANSRC} \
                ${LIBCANARDSRC} \
                ${O1HEAPSRC} \
				
UAVCANCPPSRC = ../../uavacan/uavcan.cpp


# Required include directories
UAVCANINC =     ${LIBCANARDCANINC} \
				${LIBCANARDINC} \
				${UAVCANTYPESINC} \
				${O1HEAPINC} \
		   