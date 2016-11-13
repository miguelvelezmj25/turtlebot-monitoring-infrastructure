#!/usr/bin/python

import turtlebot_remote
import ConfigParser
import signal
import math
import socket
import jetway.mdb as mdb
import load_configurations

config_parser = ConfigParser.RawConfigParser()
config_file_path = r'.serverconfig'
config_parser.read(config_file_path)
remote_host = config_parser.get(socket.gethostname(), 'simulator')

MAX_RUN_TIME = 100
MAX_EXPERIMENT_TIME = 300
navigation_configuration = {'target_x': 7, 'target_y': -10.5}


def signal_handler(signum, frame):
    raise SystemError("Time out")


def get_cpu_utilization(id, nfp, utilization_data, time_range):
    """

    :param time_range:
    :param id:`
    :param nfp:
    :param utilization_data:
    :return:
    """
    cpu_data = utilization_data[:]
    filter_data(cpu_data)

    start_item = [item for item in cpu_data if item[0] == time_range[0]][0]
    end_item = [item for item in cpu_data if item[0] == time_range[1]][0]
    cpu_data = cpu_data[cpu_data.index(start_item):cpu_data.index(end_item)+1]

    nfp_id = mdb.get_nfp_id(nfp)

    for i in range(0, len(cpu_data)):
        mdb.insert('measurements_verbose', 'configuration_id, simulator, host, nfp_id, value, time',
                   '"{}", "{}", "{}", {}, {}, {}'.format(id, remote_host, socket.gethostname(), nfp_id,
                                                         cpu_data[i][1], cpu_data[i][0]))

    mean = 0

    for entry in cpu_data:
        mean += entry[1]

    mean /= len(cpu_data)

    mdb.insert('measurements', 'configuration_id, simulator, host, nfp_id, value',
               '"{}", "{}", "{}", {}, {}'.format(id, remote_host, socket.gethostname(), nfp_id, mean))


def get_localization_uncertainty(id, duration, result, ground_truth_data, estimate_monitor_data):
    """

    :param id:
    :param duration:
    :param result:
    :param ground_truth_data:
    :param estimate_monitor_data:
    :return:
    """
    ground_data = ground_truth_data[:]
    estimate_data = estimate_monitor_data[:]

    filter_data(estimate_data)
    filter_data(ground_data, estimate_data[0][0])

    # Trim the ground data to have the same number of data points for both arrays
    ground_data = ground_data[0:len(estimate_data)]

    uncertainty = []
    nfp_id = mdb.get_nfp_id('mean_localization_error')

    for i in range(0, len(ground_data)):
        calculation = math.sqrt(math.pow(ground_data[i][1] - estimate_data[i][1], 2)
                                + math.pow(ground_data[i][2] - estimate_data[i][2], 2))

        try:
            mdb.insert('measurements_verbose', 'configuration_id, simulator, host, nfp_id, value, time',
                       '"{}", "{}", "{}", {}, {}, {}'.format(id, remote_host, socket.gethostname(), nfp_id,
                                                             calculation, estimate_data[i][0]))
        except mdb.connector.Error as error:
            print "Error when inserting data: {}".format(error)

        uncertainty.append((estimate_data[i][0], calculation))

    # Both arrays have the same number of values
    if len(estimate_data) == len(ground_data):
        mean = 0

        for current_time, data in uncertainty:
            mean += data

        mean /= len(uncertainty)

        try:
            mdb.insert('measurements', 'configuration_id, simulator, host, nfp_id, value',
                       '"{}", "{}", "{}", {}, {}'.format(id, remote_host, socket.gethostname(), nfp_id, mean))
        except mdb.connector.Error as error:
            print "Error when inserting data: {}".format(error)
    else:
        try:
            mdb.insert('measurements', 'configuration_id, simulator, host, nfp_id',
                       '"{}", "{}", "{}", {}'.format(id, remote_host, socket.gethostname(), nfp_id))
        except mdb.connector.Error as error:
            print "Error when inserting data: {}".format(error)

    if result == turtlebot_remote.FAIL:
        duration = MAX_RUN_TIME

    nfp_id = mdb.get_nfp_id('time')
    mdb.insert('measurements', 'configuration_id, simulator, host, nfp_id, value',
               '"{}", "{}", "{}", {}, {}'.format(id, remote_host, socket.gethostname(), nfp_id, duration))


def filter_data(data, start_time=-1):
    """

    :param data:
    :param start_time:
    :return:
    """
    if start_time == -1:
        start_time = data[0][0]

    current_time = start_time - 1
    filtered_data = []

    i = 0

    while i < len(data):
        element = data[i]
        entry = []

        if element[0] > (current_time + 1):
            current_time += 1
            entry.append(current_time)
            j = 1

            if len(filtered_data) == 0:
                while j < len(element):
                    entry.append((element[j] + data[0][j])/2.0)
                    j += 1
            else:
                while j < len(element):
                    entry.append((element[j] + filtered_data[-1][j])/2.0)
                    j += 1

            i -= 1

            filtered_data.append(tuple(entry))

        elif element[0] > current_time:
            current_time = element[0]
            entry.append(current_time)
            j = 1

            while j < len(element):
                entry.append(element[j])
                j += 1

            filtered_data.append(tuple(entry))
        # TODO should we check if the current element has the same time as the current time?

        i += 1

    del data[:]
    data.extend(filtered_data)


def run(id, configurations):
    """

    :param id:
    :param configurations:
    :return:
    """
    turtlebot_remote.restart(configurations['environment'])

    measurements = {}
    try:
        signal.alarm(MAX_RUN_TIME)

        measurements = turtlebot_remote.measure(id, configurations)
    except SystemError as error:
        duration = MAX_RUN_TIME
        # measurements[turtlebot_remote.DURATION] = duration
        # measurements[turtlebot_remote.GROUND_TRUTH_POSE] = turtlebot_remote.gazebo_pose_data
        # measurements[turtlebot_remote.ESTIMATE_POSE] = turtlebot_remote.amcl_pose_data
        # measurements[turtlebot_remote.RESULT] = turtlebot_remote.FAIL
        # measurements[turtlebot_remote.CPU_MONITOR] = turtlebot_remote.cpu_monitor_data
        # measurements[turtlebot_remote.LOCALIZATION_CPU_MONITOR] = turtlebot_remote.amcl_cpu_monitor_data
        # measurements[turtlebot_remote.MOVE_BASE_CPU_MONITOR] = turtlebot_remote.move_base_cpu_monitor_data
    finally:
        signal.alarm(0)

    turtlebot_remote.shutdown()

    return measurements


def filter_environment_configurations(configurations):
    """

    :param configurations:
    :return:
    """
    settings = {}

    for configuration in load_configurations.environment_parameters:
        if configurations.has_key(configuration[0]):
            settings[configuration[0]] = configurations.pop(configuration[0])

    return settings


def build_configurations(options):
    """

    :param options:
    :return:
    """
    configurations = {}

    for option in options.split(','):
        option = option.strip()
        key, value = option.split(' ')

        key = str(key)

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
    """

    :param id:
    :param environment_configurations:
    :param amcl_configurations:
    :return:
    """
    settings = {'navigation': navigation_configuration, 'environment': environment_configurations,
                'amcl': amcl_configurations}

    measurements = run(id, settings)

    get_localization_uncertainty(id, measurements[turtlebot_remote.DURATION], measurements[turtlebot_remote.RESULT],
                                 measurements[turtlebot_remote.GROUND_TRUTH_POSE],
                                 measurements[turtlebot_remote.ESTIMATE_POSE])

    time_range = (measurements[turtlebot_remote.ESTIMATE_POSE][0][0],
                  measurements[turtlebot_remote.ESTIMATE_POSE][-1][0])

    # TODO These nfps should match the db
    get_cpu_utilization(id, 'mean_cpu_utilization', measurements[turtlebot_remote.CPU_MONITOR], time_range)
    get_cpu_utilization(id, 'mean_amcl_cpu_utilization', measurements[turtlebot_remote.LOCALIZATION_CPU_MONITOR], time_range)


signal.signal(signal.SIGALRM, signal_handler)

if __name__ == '__main__':
    try:
        signal.alarm(MAX_EXPERIMENT_TIME)

        mdb.startup('turtlebot-explore')
        id = mdb.get_next_todo('worker', '"' + str(socket.gethostname()) + '"')

        if id is None:
            id = mdb.get_next_todo()

        if id is not None:
            options = mdb.exec_sql_one('select options from configurations where id = "{}"'.format(id))
            configurations = build_configurations(options)
            environment_configurations = filter_environment_configurations(configurations)
            measure(id, environment_configurations, configurations)
    except SystemError, e:
        print "The experiment could not be completed"
    finally:
        signal.alarm(0)
        mdb.shutdown()
