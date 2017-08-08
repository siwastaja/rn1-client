#!/bin/bash
while true
do
	echo "syncing."
	rsync -z proto4:/home/hrst/rn1-host/*.map .
	sleep 1
done
