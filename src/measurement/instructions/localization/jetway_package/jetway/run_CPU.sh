#/bin/bash

while true
do
	mpstat 1 1 | grep -o M..all........ | sed -e 's/M  all   //' >> ~/cpu.log
done
