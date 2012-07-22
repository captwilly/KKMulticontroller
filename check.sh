#!/bin/bash

# This checks build for every supported copter type

for COPTER_TYPE in SINGLE_COPTER DUAL_COPTER TWIN_COPTER TRI_COPTER QUAD_COPTER QUAD_X_COPTER Y4_COPTER HEX_COPTER Y6_COPTER
do
	echo -ne "Building $COPTER_TYPE\t\t"
	make -j COPTER_TYPE=$COPTER_TYPE clean > /dev/null
	make -j COPTER_TYPE=$COPTER_TYPE all > /dev/null
	if [[ 0 == $? ]] 
	then
		echo OK;
	else
		echo Fail;
	fi;
done

echo -e $OUT
