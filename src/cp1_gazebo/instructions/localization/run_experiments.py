#!/usr/bin/python

import turtlebot_remote
import ConfigParser
import signal
import math
import socket
import jetway.mdb as mdb


def signal_handler(signum, frame):
    raise SystemError("Time out")


def get_cpu_utilization(id, time_range, nfp, utilization_data):
    cpu_data = utilization_data[:]
    filter_cpu(cpu_data)
    start_item = [item for item in cpu_data if item[0] == time_range[0]][0]
    end_item = [item for item in cpu_data if item[0] == time_range[1]][0]
    cpu_data = cpu_data[cpu_data.index(start_item):cpu_data.index(end_item)+1]

    nfp_id = mdb.get_nfp_id(nfp)
    for i in range(0, len(cpu_data)):
        mdb.insert('measurements_verbose', 'configuration_id, simulator, host, nfp_id, value, time', '"' + id + '", "'
                   + remote_host + '", "' + socket.gethostname() + '", ' + str(nfp_id) + ", " + str(cpu_data[i][1])
                   + ", " + str(cpu_data[i][0]))

    mean = 0

    for entry in cpu_data:
        mean += entry[1]

    mean /= len(cpu_data)

    mdb.insert('measurements', 'configuration_id, simulator, host, nfp_id, value', '"' + id + '", "' + remote_host + '", "'
               + socket.gethostname() + '", ' + str(nfp_id) + ", " + str(mean))


def get_localization_uncertainty(id, duration, result, ground_truth_data, estimate_monitor_data):
    ground_data = ground_truth_data[:]
    estimate_data = estimate_monitor_data[:]
    filter_data(estimate_data)
    filter_data(ground_data, estimate_data[0][0])

    # Trim the ground data to have the same number of data points for both arrays
    ground_data = ground_data[0:len(estimate_data)]

    if result == 'success':
        ground_data = ground_data[:int(float(duration)) + 1]
        estimate_data = estimate_data[:int(float(duration)) + 1]

    uncertainty = []

    time_range = (ground_data[0][0], ground_data[-1][0])
    nfp_id = mdb.get_nfp_id('mean_localization_error')
    for i in range(0, len(ground_data)):
        calculation = math.sqrt(math.pow(ground_data[i][1] - estimate_data[i][1], 2)
                                + math.pow(ground_data[i][2] - estimate_data[i][2], 2))
        mdb.insert('measurements_verbose', 'configuration_id, simulator, host, nfp_id, value, time', '"' + id + '", "'
                   + remote_host + '", "' + socket.gethostname() + '", ' + str(nfp_id) + ", " + str(calculation) + ", "
                   + str(estimate_data[i][0]))
        uncertainty.append((estimate_data[i][0], calculation))

    # The ground truth recorded less values than the estimate
    if len(estimate_data) == len(ground_data):
        mean = 0

        for current_time, data in uncertainty:
            mean += data

        mean /= len(uncertainty)

        mdb.insert('measurements', 'configuration_id, simulator, host, nfp_id, value', '"' + id + '", "' + remote_host + '", "'
                   + socket.gethostname() + '", ' + str(nfp_id) + ", " + str(mean))

    else:
        mdb.insert('measurements', 'configuration_id, simulator, host, nfp_id', '"' + id + '", "' + remote_host + '", "'
                   + socket.gethostname() + '", ' + str(nfp_id))

    if result != 'success':
        duration = max_run_time

    nfp_id = mdb.get_nfp_id('time')
    mdb.insert('measurements', 'configuration_id, simulator, host, nfp_id, value', '"' + id + '", "' + remote_host + '", "'
               + socket.gethostname() + '", ' + str(nfp_id) + ", " + str(duration))

    return time_range


def filter_cpu(data):
    current_time = data[0][0] - 1
    filtered_data = []

    i = 0

    while i < len(data):
        element = data[i]

        if element[0] > current_time + 1:
            current_time += 1

            if len(filtered_data) == 0:
                filtered_data.append((current_time, (element[1] + data[i-1][1])/2))
            else:
                filtered_data.append((current_time, (element[1] + filtered_data[-1][1])/2))

            i -= 1
        elif current_time < element[0]:
            current_time = element[0]
            filtered_data.append((current_time, element[1]))

        i += 1

    del data[:]
    data.extend(filtered_data)


def filter_data(data, current_time=-1):
    if current_time == -1:
        current_time = data[0][0]

    current_time -= 1
    filtered_data = []

    i = 0

    while i < len(data):
        element = data[i]

        if element[0] > current_time + 1:
            current_time += 1

            if len(filtered_data) == 0:
                filtered_data.append((current_time, (element[1] + data[i-1][1])/2, (element[2] + data[i-1][2])/2))
            else:
                filtered_data.append((current_time, (element[1] + filtered_data[-1][1])/2, (element[2]
                                                                                            + filtered_data[-1][2])/2))

            i -= 1
        elif current_time < element[0]:
            current_time = element[0]
            filtered_data.append((current_time, element[1], element[2]))

        i += 1

    del data[:]
    data.extend(filtered_data)


def run(id, configurations):
    turtlebot_remote.restart(configurations['environment'])

    measurements = {}
    try:
        signal.alarm(max_run_time)

        measurements = turtlebot_remote.measure(id, configurations)

    except SystemError, e:
        duration = max_run_time
        measurements['duration'] = duration
        measurements['ground_truth_pose'] = turtlebot_remote.gazebo_pose_data
        measurements['estimate_pose'] = turtlebot_remote.amcl_pose_data
        measurements['result'] = 'fail'
        measurements['cpu_monitor'] = turtlebot_remote.cpu_monitor_data
        measurements['amcl_monitor'] = turtlebot_remote.amcl_cpu_monitor_data
        measurements['move_base_monitor'] = turtlebot_remote.move_base_cpu_monitor_data

    finally:
        signal.alarm(0)

    turtlebot_remote.shutdown()

    return measurements


def filter_environment_configurations(configurations):
    settings = {}

    for configuration in environment_configurations:
        if configurations.has_key(configuration):
            settings[configuration] = configurations.pop(configuration)

    return settings


def build_configurations(options):
    configurations = {}

    for option in options.split(','):
        option = option.strip()
        key, value = option.split(' ')

        str(key)
        if '"' in value or "'" in value:
            configurations[key] = value[1:-1]
        elif "." in value:
            configurations[key] = float(value)
        elif value == 'True':
            configurations[key] = True
        elif value == 'False':
            configurations[key] = False
        else:
            configurations[key] = int(value)

    return configurations


def measure(id, environment_configurations, amcl_configurations):
    settings = {'navigation': navigation_configuration, 'environment': environment_configurations,
                'amcl': amcl_configurations}

    measurements = run(id, settings)

    # Trim the first entry since amcl does not start publishing until the robot start moving
    try:
        time_range = get_localization_uncertainty(id, measurements['duration'], measurements['result'],
                                                  measurements[ground_truth_pose], measurements[estimate_pose][1:])
    except:
        print 'Error when processing localization uncertainty'
        time_range = [measurements[cpu_monitor][0][0], measurements[cpu_monitor][-1][0]]
        print 'Attempting to continue with time_range = {}'.format(time_range)

    try:
        get_cpu_utilization(id, time_range, 'mean_cpu_utilization', measurements[cpu_monitor][1:])
    except:
        print 'Error when processing mean cpu utilization'

        if time_range is None:
            time_range = [measurements[amcl_monitor][0][0], measurements[amcl_monitor][-1][0]]
            print 'Attempting to continue with time_range = {}'.format(time_range)

    try:
        get_cpu_utilization(id, time_range, 'mean_amcl_cpu_utilization', measurements[amcl_monitor][1:])
    except:
        print 'Error when processing mean amcl cpu utilization'


signal.signal(signal.SIGALRM, signal_handler)
max_run_time = 100
max_experiment_time = 300
ground_truth_pose = 'ground_truth_pose'
estimate_pose = 'estimate_pose'
cpu_monitor = 'cpu_monitor'
amcl_monitor = 'amcl_monitor'
move_base_monitor = 'move_base_monitor'
navigation_configuration = {'target_x': 7, 'target_y': -10.5}
environment_configurations = ['laser_miscalibration', 'laser_noise', 'odometry_miscalibration', 'odometry_noise']

if __name__ == '__main__':
    try:
        signal.alarm(max_experiment_time)

        config_parser = ConfigParser.RawConfigParser()
        config_file_path = r'.serverconfig'
        config_parser.read(config_file_path)
        remote_host = config_parser.get(socket.gethostname(), 'simulator')

        mdb.startup('turtlebot-explore')
        id = mdb.get_next_todo('worker', '"' + str(socket.gethostname()) + '"')

        if id is None:
            id = mdb.get_next_todo()

        if id is not None:
            options = mdb.exec_sql_one('select options from configurations where id = "{0}"'.format(id))
            configurations = build_configurations(options)
            environment_configurations = filter_environment_configurations(configurations)
            measure(id, environment_configurations, configurations)
    except SystemError, e:
        print "The experiment could not be completed"
    finally:
        signal.alarm(0)
        mdb.shutdown()
