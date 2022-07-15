# included modules
include ${BASEDIR}/cyphal/libcanard/libcanard.mk
include ${BASEDIR}/cyphal/types/types.mk
include ${BASEDIR}/cyphal/o1heap/o1heap.mk

# List of all the related files.
CYPHALSRC =     ${LIBCANARDSRC} \
                ${O1HEAPSRC} \
				
CYPHALCPPSRC = ../../cyphal/cyphal.cpp


# Required include directories
CYPHALINC =     ${LIBCANARDCANINC} \
				${LIBCANARDINC} \
				${CYPHALTYPESINC} \
				${O1HEAPINC} \
				../../cyphal/ \
		   