#!/usr/bin/python

# import turtlebot_remote
import ConfigParser
import signal
import math
import socket
import jetway.mdb as mdb

config_parser = ConfigParser.RawConfigParser()
config_file_path = r'.serverconfig'
config_parser.read(config_file_path)
remote_host = config_parser.get(socket.gethostname(), 'simulator')

MAX_RUN_TIME = 100
MAX_EXPERIMENT_TIME = 300
navigation_configuration = {'target_x': 7, 'target_y': -10.5}
environment_configurations = ['laser_miscalibration', 'laser_noise', 'odometry_miscalibration', 'odometry_noise']


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
        measurements[turtlebot_remote.DURATION] = duration
        measurements[turtlebot_remote.GROUND_TRUTH_POSE] = turtlebot_remote.gazebo_pose_data
        measurements[turtlebot_remote.ESTIMATE_POSE] = turtlebot_remote.amcl_pose_data
        measurements[turtlebot_remote.RESULT] = turtlebot_remote.FAIL
        measurements[turtlebot_remote.CPU_MONITOR] = turtlebot_remote.cpu_monitor_data
        measurements[turtlebot_remote.AMCL_CPU_MONITOR] = turtlebot_remote.amcl_cpu_monitor_data
        measurements[turtlebot_remote.MOVE_BASE_CPU_MONITOR] = turtlebot_remote.move_base_cpu_monitor_data
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

    for configuration in environment_configurations:
        if configurations.has_key(configuration):
            settings[configuration] = configurations.pop(configuration)

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

    # These nfps should match the db
    get_cpu_utilization(id, 'mean_cpu_utilization', measurements[turtlebot_remote.CPU_MONITOR], time_range)
    get_cpu_utilization(id, 'mean_amcl_cpu_utilization', measurements[turtlebot_remote.AMCL_CPU_MONITOR], time_range)


signal.signal(signal.SIGALRM, signal_handler)

# if __name__ == '__main__':
#     try:
#         signal.alarm(MAX_EXPERIMENT_TIME)
#
#         mdb.startup('turtlebot-explore')
#         id = mdb.get_next_todo('worker', '"' + str(socket.gethostname()) + '"')
#
#         if id is None:
#             id = mdb.get_next_todo()
#
#         if id is not None:
#             options = mdb.exec_sql_one('select options from configurations where id = "{}"'.format(id))
#             configurations = build_configurations(options)
#             environment_configurations = filter_environment_configurations(configurations)
#             measure(id, environment_configurations, configurations)
#     except SystemError, e:
#         print "The experiment could not be completed"
#     finally:
#         signal.alarm(0)
#         mdb.shutdown()


#TODO why is are the different nfps giving me diffirent sizes?


estimate = [(9653, 0.0147192476383, -0.0067835151022), (9655, 0.260040746141, -0.161908081008), (9656, 0.724523471581, -0.223482507852), (9657, 1.18430058087, -0.20548518997), (9658, 1.67134888478, -0.243243821082), (9659, 2.14891936823, -0.323485932533), (9660, 2.60722532574, -0.478297889027), (9661, 3.04596800013, -0.838386133701), (9662, 3.28210936739, -1.33242123113), (9663, 3.43004389923, -1.7991672339), (9664, 3.57577355553, -2.29849055329), (9665, 3.66088155087, -2.73893341235), (9666, 3.68301559598, -3.19014174821), (9667, 3.67748906997, -3.66430373707), (9668, 3.69086055856, -4.13780056548), (9669, 3.69076261477, -4.56122480178), (9670, 3.68571026389, -5.03452172927), (9671, 3.67887848908, -5.48030901079), (9672, 3.65825960696, -6.42084642684), (9673, 3.65446170931, -6.92621288963), (9674, 3.65224693433, -7.37960918352), (9675, 3.65353748927, -7.86662748027), (9676, 3.65678474103, -8.33694566022), (9677, 3.66917937842, -8.85522543333), (9678, 3.75607488513, -9.30912108133), (9679, 4.17445233276, -9.76617654924), (9680, 4.60905960457, -9.91887137417), (9681, 5.06250764509, -10.0534520515), (9682, 5.50187018384, -10.155392768), (9683, 5.96412956975, -10.2528903151), (9684, 6.41698362519, -10.3599646373), (9686, 6.83804169616, -10.467753236)]
ground = [(9649, 0.000116760448502, 2.24296028344e-05), (9650, 0.000125751718255, 2.2497292911e-05), (9651, 0.000134274588226, 2.16022169599e-05), (9652, 0.000144104965296, 2.35246604566e-05), (9653, 0.000151850289062, 2.25286157525e-05), (9654, 0.0219476173589, -9.75098709019e-05), (9655, 0.217940553529, -0.00854283154471), (9656, 0.689747761218, -0.036009642913), (9657, 1.18981631833, -0.0241224230584), (9658, 1.67309942386, -0.0542691757511), (9659, 2.16832436515, -0.133603505809), (9660, 2.65464484849, -0.25050530413), (9661, 3.07991620887, -0.492299134115), (9662, 3.38074125331, -0.889926115999), (9663, 3.56351233429, -1.34838368287), (9664, 3.70743558178, -1.82039492475), (9665, 3.84077340894, -2.2935393009), (9666, 3.91033651442, -2.79000232141), (9667, 3.92248592757, -3.29138082714), (9668, 3.93707407437, -3.79243740029), (9669, 3.95299940862, -4.29368092667), (9670, 3.95739196178, -4.79513030723), (9671, 3.95553646593, -5.29659232139), (9672, 3.9520204875, -5.79800188839), (9673, 3.94978826606, -6.29942657398), (9674, 3.94941330864, -6.80085086954), (9675, 3.94800958199, -7.3022840093), (9676, 3.94628131759, -7.80370084597), (9677, 3.94527192186, -8.30513980777), (9678, 3.96783065228, -8.79747236778), (9679, 4.12412632481, -9.2587819305), (9680, 4.49577556396, -9.58464226213), (9681, 4.96996283179, -9.74439596656), (9682, 5.44237534449, -9.88170613885), (9683, 5.92363470809, -9.98539392308), (9684, 6.40263677871, -10.0855692622), (9685, 6.82404277381, -10.190563868), (9686, 7.01182184786, -10.2374118757), (9687, 7.19299867539, -10.2828887701), (9688, 7.19300427971, -10.2828898402)]
cpu = [('9649', 75.25), ('9650', 70.37), ('9651', 73.62), ('9652', 72.82), ('9653', 71.05), ('9654', 71.03), ('9655', 73.02), ('9656', 73.28), ('9657', 72.86), ('9658', 65.46), ('9659', 73.55), ('9660', 70.65), ('9661', 72.7), ('9662', 69.21), ('9663', 73.02), ('9664', 74.44), ('9665', 73.62), ('9666', 72.82), ('9667', 71.46), ('9668', 74.56), ('9669', 77.02), ('9670', 68.88), ('9671', 72.64), ('9672', 71.43), ('9673', 71.28), ('9674', 74.0), ('9675', 74.38), ('9676', 72.39), ('9677', 69.87), ('9678', 70.81), ('9679', 72.84), ('9680', 75.84), ('9681', 71.83), ('9682', 70.47), ('9683', 71.07), ('9684', 73.21), ('9685', 69.93), ('9686', 72.05), ('9687', 71.57)]


time_range = (estimate[0][0], estimate[-1][0])
get_cpu_utilization('a', 2, cpu, time_range)
