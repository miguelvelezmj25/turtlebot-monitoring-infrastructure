import subprocess
import sys


print("Video Codecs: \n 1) Xvid \n 2) libavcodec \n 3) x264")
vcodec = input("Which video codec would you like to use? ")
while not (vcodec == 1 or vcodec == 2 or vcodec == 3):
    print("Invalid Input")
    input("Which video codec would you like to use? ")


if vcodec == 1:
    print("Starting up...\n")
    print("Initializing Power Meter...\n")
    subprocess.call("./power_meter.sh", shell=True) # Activate Power Meter
    print("Initializing CPU Monitor...\n")
    subprocess.call("./CPU_monitor.sh", shell=True) # Activate CPU Monitor
    print("Beginning Encoding Process...\n")
    subprocess.call("echo Time > time_log.txt", shell=True) #Create Time Log File

    for x in range(0, 512):
        command = "time -o time_log.txt -a ./xvid-encoder.sh " # Begin building command
        option = bin(x)[2:] # Convert x to a binary string

        while len(option) != 9:
            option = "0" + option # Insert leading 0's

        output = "~/Jetway/output-files/output" + option + ".m4v" # Name of output file

        for y in range(0, 9):
            command = command + option[y] + " " # Adding options to the command

        command += output  # Adding output to the command
        # subprocess.call("echo " + option + ">> [CPU FILE]", shell=True)
        # subprocess.call("echo " + option + ">> [POWER FILE]", shell=True)
        subprocess.call("echo " + option + ">> time_log.txt", shell=True) # Write command options to time log
        subprocess.call(command, shell=True) # Execute command
        subprocess.call(["scp Power_Consumption.txt power.txt"], shell=True)
        subprocess.call(["scp CPU-Utilization.txt CPU_usage.txt"], shell=True)
        subprocess.call(["grep -o M..all........ CPU_usage.txt | sed -e 's/M  all   //' > CPU.txt"], shell=True)
        subprocess.call(["rm CPU_usage.txt"], shell=True)
        f = open("CPU.txt")
        f_line = f.readline()
        f_total = 0
        f_count = 0

        while f_line:
            f_count += 1
            f_total += float(f_line)
            f_line = f.readline()

        g = open("power.txt")
        g_line = g.readline()
        g_total = 0
        g_count = 0

        while g_line:
            g_count += 1
            g_total += float(g_line)
            g_line = g.readline()

        subprocess.call("echo " + str(f_total / f_count) + " >> CPU_average.txt", shell=True)
        subprocess.call("echo " + str(g_total / g_count) + " >> power_average.txt", shell=True)
        subprocess.call("sleep 10", shell=True) # Sleep for 10 seconds
        subprocess.call(["rm power.txt"], shell=True)
        subprocess.call(["rm CPU.txt"], shell=True)
        subprocess.call(["rm Power_Consumption.txt"], shell=True)
        subprocess.call(["rm CPU-Utilization.txt"], shell=True)

    print("All tasks complete.\n")
    print("Turning Off Power Meter...\n")
    subprocess.call(["./power_meter.sh"], shell=True) #Deactivate Power Meter
    print("Turning Off CPU Monitor...\n")
    subprocess.call(["./CPU_monitor.sh"], shell=True) #Deactivate CPU Monitor
    subprocess.call(["grep -o 0:..... time_log.txt | sed -e 's/0://' > times.txt"], shell=True)


if vcodec == 2:
    import math
    print("Starting up...\n")
    print("Intializing Power Meter...\n")
    subprocess.call("./power_meter.sh", shell=True) # Activate Power Meter
    print("Initializing CPU Monitor...\n")
    subprocess.call("./CPU_monitor.sh", shell=True) # Activate CPU Monitor
    print("Beginning Encoding Process...\n")
    subprocess.call("echo Time > time_log.txt", shell=True) # Create Time Log File
    option = [[1, 2], [0, 1], [0, 1], [0, 1], [0, 1], [-1, 1, 4], [1, 2, 4], [0, 3, 6], [1, 3, 5], [0, 0.5, 1], [1, 4, 9]]

    for a in xrange(int(math.pow(2, 5))):
        for b in xrange(int(math.pow(3, 6))):
            x = bin(a)[2:]

            while len(x) != 5:
                x = "0" + x

            y = ""
            temp = b

            while temp != 0:
                y += str(temp % 3)
                temp /= 3

            y = y[::-1]

            while len(y) != 6 :
                y = "0" + y

            z = x + y
            command = "time -o time_log.txt -a ./libavcodec.sh " # Begin building command
            output = "~/Jetway/output-files/output" + z + ".m4v" # Name of output file

            for c in range(0, 11):
                command = command + str(option[c][int(z[c])]) + " " # Adding options to the command

            command += output
            subprocess.call("echo " + z + ">> time_log.txt", shell=True) # Write command options to time log
            subprocess.call(command, shell=True) # Execute command
            subprocess.call(["scp Power_Consumption.txt power.txt"], shell=True)
            subprocess.call(["scp CPU-Utilization.txt CPU_usage.txt"], shell=True)
            subprocess.call(["grep -o M..all........ CPU_usage.txt | sed -e 's/M  all   //' > CPU.txt"], shell=True)
            subprocess.call(["rm CPU_usage.txt"], shell=True)
            f = open("CPU.txt")
            f_line = f.readline()
            f_total = 0
            f_count = 0

            while f_line:
                f_count += 1
                f_total += float(f_line)
                f_line = f.readline()

            g = open("power.txt")
            g_line = g.readline()
            g_total = 0
            g_count = 0

            while g_line:
                g_count += 1
                g_total += float(g_line)
                g_line = g.readline()
            subprocess.call("echo " + str(f_total / f_count) + " >> CPU_average.txt", shell=True)
            subprocess.call("echo " + str(g_total / g_count) + " >> power_average.txt", shell=True)
            subprocess.call("sleep 10", shell=True) # Sleep for 10 seconds
            subprocess.call(["rm power.txt"], shell=True)
            subprocess.call(["rm CPU.txt"], shell=True)
            subprocess.call(["rm Power_Consumption.txt"], shell=True)
            subprocess.call(["rm CPU-Utilization.txt"], shell=True)
        print("All tasks complete.\n")
        print("Turning Off Power Meter...\n")
        subprocess.call(["./power_meter.sh"], shell=True)
        print("Turning Off CPU Monitor...\n")
        subprocess.call(["./CPU_monitor.sh"], shell=True)
        subprocess.call(["grep -o 0:..... time_log.txt | sed -e 's/0://' > times.txt"], shell=True)


if vcodec == 3:
    import math
    print("Starting up...\n")
    print("Intializing Power Meter...\n")
    subprocess.call("./power_meter.sh", shell=True) # Activate Power Meter
    print("Initializing CPU Monitor...\n")
    subprocess.call("./CPU_monitor.sh", shell=True) # Activate CPU Monito
    print("Beginning Encoding Process...\n")
    subprocess.call("echo Time > time_log.txt", shell=True) # Create Time Log File
    option = [[0, 1], [0, 1], [0, 1], [1, 3, 7], [1, 3, 6], ["hex", "umh", "dia"], [0, 1, 4]]

    for c in xrange(int(math.pow(2, 4))):
        for d in xrange(int(math.pow(3, 4))):
            g = bin(c)[2:]

            while len(g) != 4:
                g = "0" + g

            h = ""
            temp = d

            while temp != 0:
                h += str(temp % 3)
                temp /= 3

            h = h[::-1]

            while len(h) != 4:
                h = "0" + h

            i = g + h
            command = "time -o time_log.txt -a ./x264.sh " # Begin building command
            output = "~/Jetway/output-files/output" + i + ".m4v" # Name of output file

            for e in range(0, 8):
                command = command + str(option[e][int(i[e])]) + " " # Adding options to the command

            command += output
            subprocess.call("echo " + i + ">> time_log.txt", shell=True) # Write command options to time log
            subprocess.call(command, shell=True) # Execute command
            subprocess.call(["scp Power_Consumption.txt power.txt"], shell=True)
            subprocess.call(["scp CPU-Utilization.txt CPU_usage.txt"], shell=True)
            subprocess.call(["grep -o M..all........ CPU_usage.txt | sed -e 's/M  all   //' > CPU.txt"], shell=True)
            subprocess.call(["rm CPU_usage.txt"], shell=True)
            f = open("CPU.txt")
            f_line = f.readline()
            f_total = 0
            f_count = 0

            while f_line:
                f_count += 1
                f_total += float(f_line)
                f_line = f.readline()

            g = open("power.txt")
            g_line = g.readline()
            g_total = 0
            g_count = 0

            while g_line:
                g_count += 1
                g_total += float(g_line)
                g_line = g.readline()

            subprocess.call("echo " + str(f_total / f_count) + " >> CPU_average.txt", shell=True)
            subprocess.call("echo " + str(g_total / g_count) + " >> power_average.txt", shell=True)
            subprocess.call("sleep 10", shell=True) # Sleep for 10 seconds
            subprocess.call(["rm power.txt"], shell=True)
            subprocess.call(["rm CPU.txt"], shell=True)
            subprocess.call(["rm Power_Consumption.txt"], shell=True)
            subprocess.call(["rm CPU-Utilization.txt"], shell=True)

        print("All tasks complete.\n")
        print("Turning Off Power Meter...\n")
        subprocess.call(["./power_meter.sh"], shell=True)
        print("Turning Off CPU Monitor...\n")
        subprocess.call(["./CPU_monitor.sh"], shell=True)