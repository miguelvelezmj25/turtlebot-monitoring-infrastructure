import subprocess
import sys
import os.path
import os
import binascii
import time
import hashlib

timeout = "20m"


def ex(p):
    subprocess.call(p, shell=True)


if len(sys.argv) == 2:
    run_id = sys.argv[1]
else:
    run_id = binascii.hexlify(os.urandom(4))

result_dir = "sac_results/" + run_id + "/"
ex("mkdir -p " + result_dir)

print("reporting results in " + result_dir + "\n")


def start():
    print("Starting up...\n")
    print("Initializing Power Meter...\n")
    subprocess.call("./power_meter.sh -start", shell=True) # Activate Power Meter
    print("Initializing CPU Monitor...\n")
    subprocess.call("./CPU_monitor.sh -start", shell=True) # Activate CPU Monitor


def stop():
    print("All tasks complete.\n")
    print("Turning Off Power Meter...\n")
    subprocess.call(["./power_meter.sh -stop"], shell=True) # Deactivate Power Meter
    print("Turning Off CPU Monitor...\n")
    subprocess.call(["./CPU_monitor.sh -stop"], shell=True) # Deactivate CPU Monitor


def measure(key, cmd):
    command = "time -f \"real:%e\nuser:%U\nsys:%S\nexit:%x\nioin:%I\nioout:%O\nmaxmem:%M\navgmem:%K\" -o " + result_dir + key + " -a timeout " + timeout + " " + cmd

    subprocess.call(">Power_Consumption.txt; > CPU-Utilization.txt", shell=True)
    subprocess.call("echo " + key + ">> " + result_dir + key, shell=True)

    exit_code = subprocess.call(command, shell=True)

    subprocess.call("cp Power_Consumption.txt _power.txt", shell=True)
    subprocess.call("cp CPU-Utilization.txt _CPU_usage.txt", shell=True)
    subprocess.call(["grep -o M..all........ _CPU_usage.txt | sed -e 's/M  all   //' > _CPU.txt"], shell=True)
    f = open("_CPU.txt")
    f_line = f.readline()
    f_total = 0
    f_count = 0

    while f_line:
        f_count += 1
        f_total += float(f_line)
        f_line = f.readline()

    g = open("_power.txt")
    g_line = g.readline()
    g_total = 0
    g_count = 0

    while g_line:
        g_count += 1
        g_total += float(g_line)
        g_line = g.readline()

    if g_count > 0:
        power = str( g_total / g_count)
    else:
        power = "-1"

    if f_count > 0:
        cpu = str( f_total / f_count)
    else:
        cpu = "-1"

    subprocess.call("echo cpu:" + cpu + " >> " + result_dir + key, shell=True)
    subprocess.call("echo power:" + power + " >> " + result_dir + key, shell=True)
    return exit_code


def should_skip(key):
    return os.path.isfile(result_dir + "compile-" + key)


def analyze(key, param):
    print("***** running " + key +" /" + run_id + "\n")

    if os.path.isfile("1024_bodies_dynamic.sacbugreport"):
        subprocess.call("rm 1024_bodies_dynamic.sacbugreport", shell=True)

    exitcode = measure("compile-"+key, "./sac-compile.sh "  + param)

    if os.path.isfile("1024_bodies_dynamic.sacbugreport") or (exitcode != 0):
        print("compilation failed.\n")
        ex("mv 1024_bodies_dynamic.sacbugreport " + result_dir + "compile-" + key + ".sacbugreport")
    else:
        ex("wc -c nbody/tmp.out >> " + result_dir + "compile-" + key)
        sys.stdout.write("done"),
        time.sleep(5)
        print(".\n")
        measure("run-"+key, "./sac-run.sh")
        sys.stdout.write("done"),\

    time.sleep(5)
    print(".\n")


def run_all(todos):
    """
    run a list of tasks (each a tuple of key and parameter)
    """
    num = len(todos)
    count = 0
    start_time = time.time()

    for todo in todos:
        if should_skip(todo[0]):
            print("*** skipping "+todo[0])
            num -= 1
        else:
            t1 = time.time()
            analyze(todo[0],todo[1])
            t2 = time.time()
            count += 1
            estimatedtotaltime = (t2-start_time)/count*num
            print("analysis time: "+str(t2-t1)+"s, estimated remaining: "+str((estimatedtotaltime - (t2-start_time))/60/60)+"h")


todos = []
with open("sac.list") as f:
    lines = f.readlines()
    for idx in range(0, (len(lines)/3 - 1)):
        key = lines[idx*3+1].strip()

        if len(key) > 255:
            new_key = hashlib.md5(key).hexdigest()
            ex("echo \"" + key +"\" > " + result_dir + "/" + new_key + ".key")
            key = new_key

        param = lines[idx*3+2].strip()
        todos.append((key, param))

start()
run_all(todos)
print("All tasks complete.\n")
stop()