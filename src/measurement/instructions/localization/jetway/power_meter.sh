
if [ -e "save_pid.txt" ] ; then   # Checks if save_pid.txt exists
	echo stopping power meter
	kill -9 "$(cat save_pid.txt)"  # Kills the PID stored in save_pid.txt
	rm save_pid.txt # Deletes the file storing the temporary PID
else
	echo starting power meter
	nohup ./run_wattsup.sh > /dev/null 2>&1& # Activates the Power Meter
	echo $! > save_pid.txt # Creates a temporary file containing the PID
fi
