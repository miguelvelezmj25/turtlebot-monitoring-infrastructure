#/bin/bash

while true
do
	./wattsup ttyUSB0 -c 1 watts >> Power_Consumption.txt
done
