import collections
import operator
import subprocess
import load_configurations as configurations
import jetway.mdb as mdb

turtlebot_explore_db = 'turtlebot-explore'
data_folder = './data/'
file_extension = '_data.csv'
default_configuration = 'e26ab2de-93ae-11e6-bf5b-000c290c1bad'
compare_configurations_r_script = './graph_compare_options.R '
graph_configuration_options_r_script = './graph_configuration_options.R '


def graph_configuration_options(configuration, default=None, id_option_tuples=None):
    """
    graph_options(options, tuples=None)

    Graph all options for a specific configuration.

    :param default:
    :param configuration:
    :param id_option_tuples:
    """
    mdb.startup(turtlebot_explore_db)
    nfps_tuple = mdb.exec_sql('select name from nfps')

    nfps = []
    for nfp in nfps_tuple:
        nfps.append(str(nfp[0]))

    if id_option_tuples is None:
        id_option_tuples = mdb.exec_sql('select id, options from configurations where options like "{}%" '
                                        'ORDER BY options'.format(configuration))

    mdb.shutdown()

    # Use ordered dictionary to keep track of order
    options_ids = collections.OrderedDict()

    for element in id_option_tuples:
        option = str(element[1]).strip()
        key, value = option.split(',')[0].split(' ')

        if '"' in value or "'" in value:
            option = value[1:-1]
        elif "." in value:
            option = float(value)
        elif value == 'True':
            option = True
        elif value == 'False':
            option = False
        else:
            option = int(value)

        options_ids[option] = str(element[0])

    sorted_dict = sorted(options_ids.items(), key=operator.itemgetter(0))
    options_ids = collections.OrderedDict()

    for element in sorted_dict:
        options_ids[element[0]] = element[1]

    options_files = collections.OrderedDict()

    for nfp in nfps:
        for option, id in options_ids.items():
            options_files[option] = str(nfp + '_' + id + file_extension)

        servers = set()
        for option, data_file in options_files.items():
            with open(data_folder + data_file, 'r') as data:
                header = data.readline().strip().split(',')
                for word in header:
                    servers.add(word)

        r_arguments = configuration + ' ' + nfp + ' '

        if default is None:
            r_arguments += 'NA'
        else:
            r_arguments += str(default)

        r_arguments += ' '

        for server in servers:
            r_arguments += server
            r_arguments += ' '

        r_arguments += ','

        for option, data_file in options_files.items():
            r_arguments += str(option) + ' ' + data_file + ' '

        subprocess.call('Rscript ' + graph_configuration_options_r_script + r_arguments, shell=True)


def compare_configurations(nfps, configuration_ids, name):
    """
    compare_configurations(nfps, configuration_ids, name)

    :param nfps:
    :param configuration_ids:
    :param name:
    :return:
    """
    files = []

    for nfp in nfps:
        for configuration_id in configuration_ids:
            files.append(nfp + '_' + configuration_id + file_extension)

    servers = set()
    for data_file in files:
        with open(data_folder + data_file, 'r') as data:
            header = data.readline().strip().split(',')
            for word in header:
                servers.add(word)

    files = []
    configuration_ids.append(default_configuration)

    for nfp in nfps:
        files.append(nfp)
        for configuration_id in configuration_ids:
            files.append(nfp + '_' + configuration_id + file_extension)

    r_arguments = name + ' '

    for server in servers:
        r_arguments += server
        r_arguments += ' '

    r_arguments += ','

    for data_file in files:
        r_arguments += data_file
        r_arguments += ' '

    r_arguments = r_arguments[:-1]

    subprocess.call('Rscript ' + compare_configurations_r_script + r_arguments, shell=True)


def compare_configuration(configuration, min, max, name=None):
    """
    compare_configuration(configuration)

    :param name:
    :param max:
    :param min:
    :param configuration:
    """
    mdb.startup(turtlebot_explore_db)
    nfps_tuple = mdb.exec_sql('select name from nfps')

    nfps = []
    for nfp in nfps_tuple:
        nfps.append(str(nfp[0]))

    configuration_ids = [str(mdb.exec_sql('select id from configurations where options '
                                          'like "{} {}"'.format(configuration, min))[0][0]),
                         str(mdb.exec_sql('select id from configurations where options '
                                          'like "{} {}"'.format(configuration, max))[0][0])]

    mdb.shutdown()

    if name is None:
        name = configuration

    compare_configurations(nfps, configuration_ids, name)


def compare_custom_configuration(configuration_1, configuration_2, name):
    """
    compare_configuration(configuration)

    :param name:
    :param configuration_2:
    :param configuration_1:
    """
    mdb.startup(turtlebot_explore_db)
    nfps_tuple = mdb.exec_sql('select name from nfps')

    nfps = []
    for nfp in nfps_tuple:
        nfps.append(str(nfp[0]))

    configuration_ids = [str(mdb.exec_sql('select id from configurations where options '
                                          ' = "{}"'.format(configuration_1))[0][0]),
                         str(mdb.exec_sql('select id from configurations where options '
                                          ' = "{}"'.format(configuration_2))[0][0])]

    mdb.shutdown()
    compare_configurations(nfps, configuration_ids, name)


def compare_particle_filter_configurations():
    """
    compare_particle_filter_configurations()

    Graph configurations to compare the default values to the minimum and maximum values
    """
    for configuration in configurations.filter_parameters:
        compare_configuration(configuration[0], configuration[2], configuration[3])


def compare_laser_configurations():
    """
    compare_laser_configurations()

    Graph configurations to compare the default values to the minimum and maximum values
    """
    for configuration in configurations.laser_parameters:
        compare_configuration(configuration[0], configuration[2], configuration[3])


def compare_environment_configurations():
    """
    compare_environment_configurations()

    Graph configurations to compare the default values to the minimum and maximum values
    """
    for configuration in configurations.environment_parameters:
        compare_configuration(configuration[0], configuration[2], configuration[3])


def compare_particle_filter_configurations_combine():
    """
    compare_particle_filter_configurations_combine()

    Graph combine configurations to compare the default values to the minimum and maximum values
    """
    mdb.startup(turtlebot_explore_db)

    for options in configurations.filter_parameters_combine:
        name = options[0]
        options = options[1:]
        parameter = options[0][0]
        min = str(options[0][2]) + ', '
        for option in options[1:]:
            min += option[0] + ' ' + str(option[2]) + ', '

        min = min[:-2]

        max = str(options[0][3]) + ', '
        for option in options[1:]:
            max += option[0] + ' ' + str(option[3]) + ', '

        max = max[:-2]
        compare_configuration(parameter, min, max, name)

    mdb.shutdown()


def graph_particle_filter_configurations():
    """
    graph_particle_filter_configurations()

    Graph all options of all the configurations in the particle filter that are worth exploring
    """
    for option in configurations.filter_parameters_to_explore:
        default = [item for item in configurations.filter_parameters if item[0] == option][0][1]
        graph_configuration_options(option, default)


def graph_laser_configurations():
    """
    graph_laser_configurations()

    Graph all options of all the configurations in the laser that are worth exploring
    """
    for option in configurations.laser_parameters_to_explore:
        default = [item for item in configurations.laser_parameters if item[0] == option][0][1]
        graph_configuration_options(option, default)


def graph_environment_configurations():
    """
    graph_environment_configurations()

    """
    for configuration in configurations.environment_parameters_to_explore:
        default = [item for item in configurations.environment_parameters if item[0] == configuration][0][1]
        graph_configuration_options(configuration, default)


def graph_particle_filter_configurations_combine():
    """
    graph_particle_filter_configurations_combine()

    Graph all options of all the combine configurations in the particle filter that are worth exploring
    """
    mdb.startup(turtlebot_explore_db)

    for options in configurations.filter_parameters_combine_to_explore:
        name = options[0]
        options = options[1:]
        combine_option = ''
        for option in options:
            combine_option += option + '%, '

        combine_option = combine_option[:-3]
        tuples = mdb.exec_sql('select id, options from configurations where options like "{0}%" '
                              'and options not like "{0}%,%" ORDER BY options'.format(combine_option))

        analyze = []
        for tuple in tuples:
            element = tuple[1].split(',')
            element_1 = element[0].strip()
            element_2 = element[1].strip()

            if element_1.split(' ')[-1] == element_2.split(' ')[-1]:
                analyze.append(tuple)

        graph_configuration_options(name, id_option_tuples=tuples)

    mdb.shutdown()
