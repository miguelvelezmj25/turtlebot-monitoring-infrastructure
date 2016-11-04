import jetway.mdb as mdb

turtlebot_explore_db = 'turtlebot-explore'
data_folder = './data/'
file_extension = '_data.csv'


def save_data():
    """
    For all hosts and configurations, save the data in csv files
    """
    mdb.startup(turtlebot_explore_db)
    hosts_tuple = mdb.exec_sql('select DISTINCT host from measurements where host like "%.andrew%" order by host')

    hosts = []
    for host in hosts_tuple:
        hosts.append(str(host[0]))

    configuration_ids_tuples = mdb.exec_sql('select id from configurations ORDER BY created_at DESC')
    configuration_ids = []

    for configuration_id in configuration_ids_tuples:
        configuration_ids.append(str(configuration_id[0]))

    for id in configuration_ids:
        save(id, hosts, str(id) + file_extension)

    mdb.shutdown()


def save(configuration_id, hosts, file_name):
    """
    save(configuration_id, hosts, file_name)

    For each host, save the individuals values of a configuration in a csv file
    :param configuration_id:
    :param hosts:
    :param file_name:
    """
    nfps_tuple = mdb.exec_sql('select name from nfps')

    nfps = []
    for nfp in nfps_tuple:
        nfps.append(str(nfp[0]))

    for nfp in nfps:
        nfp_id = mdb.get_nfp_id(nfp)
        host_to_data = {}

        for host in hosts:
            data_tuple = mdb.exec_sql('select value from measurements where host = "' + host + '" and nfp_id = '
                                      + str(nfp_id) + ' and configuration_id = "' + configuration_id + '"')
            data = []

            for value in data_tuple:
                if value[0] != 'None':
                    data.append(float(value[0]))

            host_to_data[host] = data

        max_length = -1

        host_to_data = {k: v for k, v in host_to_data.items() if len(v) > 0}

        for list in host_to_data.values():
            if max_length < len(list):
                max_length = len(list)

        data_file = open(data_folder + nfp + '_' + file_name, "w", 0)
        i = 0

        for host in host_to_data:
            if i > 0:
                data_file.write(',')

            data_file.write(host.split('.')[0])
            i += 1

        data_file.write('\n')

        row = 0

        while row < max_length:
            i = 0

            for host in host_to_data:
                if i > 0:
                    data_file.write(',')

                if row < len(host_to_data[host]):
                    data_file.write(str(host_to_data[host][row]))

                i += 1

            data_file.write('\n')
            row += 1

        data_file.close()
