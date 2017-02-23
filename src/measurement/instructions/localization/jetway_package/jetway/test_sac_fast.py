import subprocess
import sys
import os.path
import os
import binascii
import time
import hashlib

timeout = "5m"


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


def stop():
    print("All tasks complete.\n")


def hashfile(f):
    return hashlib.md5(open(f, 'rb').read()).hexdigest()


def measure(key, cmd):
    command = "time -f \"real:%e\nuser:%U\nsys:%S\nexit:%x\nioin:%I\nioout:%O\nmaxmem:%M\navgmem:%K\" -o " \
              + result_dir + key + " -a timeout " + timeout + " " + cmd

    subprocess.call("echo " + key + ">> " + result_dir + key, shell=True)
    exit_code = subprocess.call(command, shell=True)

    return exit_code


def should_skip(key):
    return os.path.isfile(result_dir + "compile-" + key)


def analyze(key, param):
    print("***** running " + key +" /" + run_id + "\n")

    if os.path.isfile("1024_bodies_dynamic.sacbugreport"):
        subprocess.call("rm 1024_bodies_dynamic.sacbugreport", shell=True)

    exit_code = measure("compile-"+key, "./sac-compile.sh "  + param)

    if os.path.isfile("1024_bodies_dynamic.sacbugreport") or (exit_code != 0):
        print("compilation failed.\n")
        ex("mv 1024_bodies_dynamic.sacbugreport " + result_dir + "compile-" + key + ".sacbugreport")
    else:
        ex("wc -c nbody/tmp.out")
        ex("wc -c nbody/tmp.out >> " + result_dir + "compile-" + key)
        hash_c = hashfile("nbody/tmp.out")
        ex("echo bin-hash:" + hash_c +" >> " + result_dir + "compile-" + key)
        sys.stdout.write("done"),
        time.sleep(0.1)
        print(".\n")
        print(hash_c)

        if os.path.isfile(result_dir+ "run-"+hash_c):
            print("already measured binary "+hash_c)
        else:
            measure("run-"+hash_c, "./sac-run.sh")

        sys.stdout.write("done"),\

    time.sleep(0.1)
    print(".\n")


def runall(todos):
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
            estimated_total_time = (t2-start_time)/count*num
            print("analysis time: "+str(t2-t1)+"s, estimated remaining: "+str((estimated_total_time - (t2-start_time))/60/60)+"h")


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
runall(todos)
print("All tasks complete.\n")
stop()
