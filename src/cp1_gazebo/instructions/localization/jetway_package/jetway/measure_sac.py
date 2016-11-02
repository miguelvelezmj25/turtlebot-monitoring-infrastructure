from mdb import *
import subprocess
import sys
import os.path
import os
import binascii
import time
from mcontrol import m_control
import hashlib
 
# Main measurement util for SaC
#
# assumes one or more measurement series as parameter
# will process series sequentially


def ex(p):
    return subprocess.call(p, shell=True)


# use symbolic links to map powermeter readings to local files if necessary
home = os.path.expanduser("~")+"/"
working_dir = home + "energy/"
power_meter_path = working_dir + "energy.log"
cpu_meter_path = home + "/cpu.log"

tmp_power_meter_path = working_dir + ".energy.log.tmp"
tmp_cpu_meter_path = working_dir + ".cpu.log.tmp"

timeout = "20m"

msr_dir = working_dir + "sac_results/"

SACOUT = working_dir + "nbody/tmp.out"
SACCOMPILE = working_dir + "sac-compile.sh"
SACRUN = working_dir + "sac-run.sh"

ex("sh CPU_monitor.sh start")
ex("mkdir -p " + working_dir)
ex("mkdir -p " + msr_dir)
os.chdir(working_dir)
assert os.path.isfile(SACCOMPILE)
assert os.path.isfile(SACRUN)
print "starting measuring process. make sure power readings are activated and directed to " + power_meter_path


def read_avg_log_value(log_file):
    f = open(log_file)
    f_line = f.readline()
    f_total = 0
    f_count = 0

    while f_line:
        f_count += 1
        f_total += float(f_line)
        f_line = f.readline()

    if f_count > 0:
        cpu = str(f_total / f_count)
    else:
        cpu = "-1"

    return cpu


def read_log_file(log_file):
    my_vars = {}
    assert os.path.isfile(log_file)

    with open(log_file) as my_file:
        for line in my_file:
            if ":" in line:
                name, var = line.partition(":")[::2]
                my_vars[name.strip()] = var.strip()

    return my_vars


def hash_file(f):
    return hashlib.sha1(open(f, 'rb').read()).hexdigest()


def measure(log_file, cmd, extra_gather_results=None):
    command = "time -f \"real:%e\nuser:%U\nsys:%S\nexit:%x\nioin:%I\nioout:%O\nmaxmem:%M\navgmem:%K\" -o " \
              + log_file + " -a timeout " + timeout + " " + cmd

    # discard old powermeter measurements
    ex(">{0}; >{1}; >{2}".format(power_meter_path, cpu_meter_path, log_file))

    exitcode = ex(command)

    ex("sed -e 's/^.*: \([0-9]*\)$/\\1/' < {0} | grep \"^[0-9]*$\" > {1}".format(power_meter_path, tmp_power_meter_path))
    ex("cat " + tmp_power_meter_path)
    ex("cp {0} {1}".format(cpu_meter_path, tmp_cpu_meter_path))
    avg_cpu=read_avg_log_value(tmp_cpu_meter_path)
    avg_power=read_avg_log_value(tmp_power_meter_path)
    results = {"cpu": avg_cpu, "power":avg_power, "exit": exitcode}

    if extra_gather_results is not None:
        results.update(cmd)

    results.update(read_log_file(log_file))

    return results


def measure_sa_c(series_name, config_id):
    """
    perform a measurement for a specific configuration (in a series) and
    returns a list of measurement results (map from NFP name to value)

    the method is called by the mcontrol infrastructure
    the actual measurement depends heavily on the actual program
    being measured.
    in this case, both the compilation and the running of the compiled
    program are measured. it used measurements from `time` as well as
    measurement from external CPU meters and power meters
    """
    param = get_config_params(config_id)
    print "\n*** measuring {0} ({1})".format(config_id, param)

    time.sleep(5)
    compile_results = measure(msr_dir + ".compilelog." + series_name, SACCOMPILE + " " + param)

    if compile_results["exit"] != "0":
        print "compilation failed."
        run_results = {}
    else:
        compile_results["size"]=os.path.getsize(SACOUT)
        hash_v = hash_file(SACOUT)
        compile_results["hash"] = hash_v
        run_log_file = msr_dir + ".runlog." + series_name + "." + hash_v

        if os.path.isfile(run_log_file):
            print "same binary measured earlier, skipping"
            run_results = {}
        else:
            time.sleep(5)
            run_results = measure(run_log_file, SACRUN)

    results = {}
    results.update(dict(("compile-"+k, v) for (k,v) in compile_results.items()))
    results.update(dict(("run-"+k, v) for (k,v) in run_results.items()))
    print str(results)

    return results


if len(sys.argv) <= 1:
    print "expecting measurement series as parameter"
    sys.exit(1)

m_control(sys.argv[1:], measure_sa_c)