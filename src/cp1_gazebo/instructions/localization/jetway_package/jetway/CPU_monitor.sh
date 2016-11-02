

case "$1" in
  start)
	if [ -e "cpumonitor.pid" ] ; then # Checks if the file CPU-PID.txt exists
		echo cpu monitor is already running
	else
		echo starting CPU monitor
		nohup ./run_CPU.sh > /dev/null 2>&1 & 	# Activates the CPU Monitor
		echo $! > cpumonitor.pid	# Stores the PID in a temporary file CPU-PID.txt
	fi	
	;;
  stop)
	if [ -e "cpumonitor.pid" ] ; then # Checks if the file CPU-PID.txt exists
		echo stopping CPU monitor
		kill -9 "$(cat cpumonitor.pid)"	# Kills the PID stored in CPU-PID.txt
		rm cpumonitor.pid	# Deletes the temporary file CPU-PID.txt containing the PID
	else
		echo cpu monitor is not running
	fi
	;;	
  status)
	if [ -e "cpumonitor.pid" ] ; then # Checks if the file CPU-PID.txt exists
		echo cpu monitor is running
	else
		echo cpu monitor is not running
	fi
	;;
  *)
    echo "Usage: CPU_monitor.sh {start|stop|status}" >&2
    exit 3
    ;;
esac
exit 0
