import collections
import operator
import subprocess
import load_configurations as configurations
import jetway.mdb as mdb

turtlebot_explore_db = 'turtlebot-explore'
data_folder = './data/'
file_extension = '_data.csv'
default_configuration = 'e26ab2de-93ae-11e6-bf5b-000c290c1bad'
compare_configurations_r_script = './compare_configurations.R '
graph_configuration_options_r_script = './graph_configuration_options.R '


def graph_configuration_options(configuration, id_option_tuples=None):
    """
    graph_options(options, tuples=None)

    Graph all options for a specific
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

        for server in servers:
            r_arguments += server
            r_arguments += ' '

        r_arguments += ','

        for option, data_file in options_files.items():
            r_arguments += str(option) + ' ' + data_file + ' '

        subprocess.call('Rscript ' + graph_configuration_options_r_script + r_arguments, shell=True)

    mdb.shutdown()


def compare_configuration(configuration):
    mdb.startup(turtlebot_explore_db)
    nfps_tuple = mdb.exec_sql('select name from nfps')

    nfps = []
    for nfp in nfps_tuple:
        nfps.append(str(nfp[0]))

    configuration_ids_tuples = mdb.exec_sql('select id from configurations where options like "{0}%"'.format(configuration))
    mdb.shutdown()

    configuration_ids = []

    for configuration_id in configuration_ids_tuples:
        configuration_ids.append(str(configuration_id[0]))

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

    r_arguments = configuration + ' '

    for server in servers:
        r_arguments += server
        r_arguments += ' '

    r_arguments += ','

    for data_file in files:
        r_arguments += data_file
        r_arguments += ' '

    r_arguments = r_arguments[:-1]

    subprocess.call('Rscript ' + compare_configurations_r_script + r_arguments, shell=True)


def compare_particle_filter_options():
    for configuration in configurations.filter_options:
        compare_configuration(configuration[0])


def compare_laser_options():
    for configuration in configurations.laser_options:
        compare_configuration(configuration[0])


def graph_particle_filter_options():
    for option in configurations.filter_options_to_explore:
        graph_configuration_options(option)


def graph_particle_filter_options_combine():
    mdb.startup(turtlebot_explore_db)

    for options in configurations.filter_options_combine_to_explore:
        name = options[0]
        options = options[1:]
        combine_option = ''
        for option in options:
            combine_option += option + '%, '

        combine_option = combine_option[:-3]
        print combine_option
        tuples = mdb.exec_sql('select id, options from configurations where options like "{0}%" '
                              'and options not like "{0}%,%" ORDER BY options'.format(combine_option))

        graph_configuration_options(name, tuples)

    mdb.shutdown()
