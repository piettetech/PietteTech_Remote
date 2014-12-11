#!/bin/bash
c=1
while [ $c -le 5000 ]
do
	echo "Sending sensor data $c times"
	(( c++ ))
	date
	nc 192.168.1.104 5005 < sensor_define.txt
	sleep 5
done
