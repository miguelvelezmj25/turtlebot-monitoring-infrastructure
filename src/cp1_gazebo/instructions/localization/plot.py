import matplotlib.pyplot as pyplot
import jetway.mdb as mdb
import numpy
import os
import scipy.stats
import smtplib

plots_directory = 'plots/'

if not os.path.exists(plots_directory):
    os.mkdir(plots_directory)


def plot(graph, label, show=True, save=False):
    if save:
        graph.savefig(plots_directory + label + ".png")

    if show:
        graph.show()

    pyplot.close()


def plot_graph(x, y, x_label, y_label, axis, label, show=True, save=False):
    pyplot.plot(x, y, 'bs', x, y, 'b-')
    pyplot.xlabel(x_label)
    pyplot.ylabel(y_label)
    pyplot.axis(axis)

    if save:
        pyplot.savefig(plots_directory + label + ".png")

    if show:
        pyplot.show()

    pyplot.close()


def plot_box_plot(data, label, show=True, save=False):
    data.sort()
    print data
    figure, auxiliary = pyplot.subplots(1)
    auxiliary.boxplot(data, labels=[label])
    mean = float(numpy.mean(data))
    median = float(numpy.median(data))
    standard_deviation = float(numpy.std(data))
    min = data[0]
    max = data[-1]

    text = '$n=%d$\n$\mu=%.3f$\n$\mathrm{median}=%.3f$\n$\sigma=%.3f$\n$max=%.3f$\n$min=%.3f$' \
           % (len(data), mean, median, standard_deviation, max, min)
    props = dict(boxstyle='round', facecolor='wheat', alpha=0.5)
    auxiliary.text(0.7, 0.95, text, transform=auxiliary.transAxes, fontsize=14, verticalalignment='top', bbox=props)

    plot(pyplot, label, show, save)


def plot_histogram(x, x_label, y=None, y_label=None, label=None, show=True, save=False):
    if y is None:
        pyplot.hist(x)

    else:
        print x
        print y
        pyplot.hist(x, alpha=0.5, label='x')
        pyplot.hist(y, alpha=0.5, label='y')
        pyplot.legend(loc='upper right')
        pyplot.xlabel(x_label)

    if save:
        pyplot.savefig(plots_directory + label + ".png")

    if show:
        pyplot.show()

    pyplot.close()


def calculate_t_test():
    mdb.startup()

    x_configurations = mdb.select_where('amcl_configurations', 'options = "min_particles 500, '
                                                               'max_particles 2000" '
                                                               'and DATE(created_at) = "2016-10-05"')

    x_configuration_ids = []
    for element in x_configurations:
        x_configuration_ids.append(element[0])

    measurement = 'mean_localization_error'
    nfp_id = mdb.get_nfp_id(measurement)
    x_measurements = mdb.select_where('amcl_measurements', 'nfp_id = ' + str(nfp_id) + ' and id in '
                                    + str(tuple(x_configuration_ids)))

    x = []

    for element in x_measurements:
        # value column
        if element[4] != 'None':
            x.append(float(element[4]))

    y_configurations = mdb.select_where('amcl_configurations', 'options = "min_particles 500, '
                                                               'max_particles 2000" '
                                                               'and DATE(created_at) = "2016-10-06"')

    y_configuration_ids = []
    for element in y_configurations:
        y_configuration_ids.append(element[0])

    y_measurements = mdb.select_where('amcl_measurements', 'nfp_id = ' + str(nfp_id) + ' and id in '
                                    + str(tuple(y_configuration_ids)))

    y = []

    for element in y_measurements:
        # value column
        if element[4] != 'None':
            y.append(float(element[4]))

    x[:] = x[:-1]
    y.sort()
    y[:] = y[:-3]

    if len(x) != len(y):
        raise ValueError('The lengths of the two arrays are not the same')

    plot_histogram(x, measurement, label='t_test_' + measurement, y=y, save=False)
    print "Measurement: " + measurement
    t_statistic, p_value = scipy.stats.ttest_ind(x, y, equal_var=False)
    print "T-test t-statistic: " + str(t_statistic)
    print "T-test p-value: " + str(p_value)

    measurement = 'mean_cpu_utilization'
    nfp_id = mdb.get_nfp_id(measurement)
    x_measurements = mdb.select_where('amcl_measurements', 'nfp_id = ' + str(nfp_id) + ' and id in '
                                      + str(tuple(x_configuration_ids)))

    x = []

    for element in x_measurements:
        # value column
        if element[4] != 'None':
            x.append(float(element[4]))

    y_configuration_ids = []
    for element in y_configurations:
        y_configuration_ids.append(element[0])

    measurement = 'mean_cpu_utilization'
    y_measurements = mdb.select_where('amcl_measurements', 'nfp_id = ' + str(nfp_id) + ' and id in '
                                      + str(tuple(y_configuration_ids)))

    y = []

    for element in y_measurements:
        # value column
        if element[4] != 'None':
            y.append(float(element[4]))

    x[:] = x[:-1]
    y.sort()
    y[:] = y[:-3]

    if len(x) != len(y):
        raise ValueError('The lengths of the two arrays are not the same')

    plot_histogram(x, measurement, label='t_test_' + measurement, y=y, save=False)
    print "Measurement: " + measurement
    t_statistic, p_value = scipy.stats.ttest_ind(x, y, equal_var=False)
    print "T-test t-statistic: " + str(t_statistic)
    print "T-test p-value: " + str(p_value)

    measurement = 'time'
    nfp_id = mdb.get_nfp_id(measurement)
    x_measurements = mdb.select_where('amcl_measurements', 'nfp_id = ' + str(nfp_id) + ' and id in '
                                      + str(tuple(x_configuration_ids)))

    x = []

    for element in x_measurements:
        # value column
        if element[4] != 'None':
            x.append(float(element[4]))

    y_configuration_ids = []
    for element in y_configurations:
        y_configuration_ids.append(element[0])

    measurement = 'time'
    y_measurements = mdb.select_where('amcl_measurements', 'nfp_id = ' + str(nfp_id) + ' and id in '
                                      + str(tuple(y_configuration_ids)))

    y = []

    for element in y_measurements:
        # value column
        if element[4] != 'None':
            y.append(float(element[4]))

    x[:] = x[:-1]
    y.sort()
    y[:] = y[:-3]

    if len(x) != len(y):
        raise ValueError('The lengths of the two arrays are not the same')

    t_statistic, p_value = scipy.stats.ttest_ind(x, y, equal_var=False)

    plot_histogram(x, measurement, label='t_test_' + measurement, y=y, save=False)
    print "Measurement: " + measurement
    t_statistic, p_value = scipy.stats.ttest_ind(x, y, equal_var=False)
    print "T-test t-statistic: " + str(t_statistic)
    print "T-test p-value: " + str(p_value)

    mdb.shutdown()


def amcl_explore_option(option=0, value=0):
    option = 'update_min_d'
    value = 4.0
    mdb.startup()

    measurement = 'mean_localization_error'
    nfp_id = mdb.get_nfp_id(measurement)
    configurations = mdb.exec_sql('select am.id, am.value, ac.options '
                                  'from amcl_measurements as am '
                                  'join amcl_configurations as ac '
                                  'on am.id = ac.id '
                                  'where ac.options = "' + option + ' ' + str(value) + '" ' +
                                  'and am.created_at > (NOW() - INTERVAL 1 DAY) '
                                  'and am.nfp_id = ' + str(nfp_id))

    x = []

    for element in configurations:
        if element[1] is not None:
            x.append(float(element[1]))

    explore = 'explore'
    plot_box_plot(x, explore + '_' + measurement, show=True, save=False)

    measurement = 'mean_cpu_utilization'
    nfp_id = mdb.get_nfp_id(measurement)
    configurations = mdb.exec_sql('select am.id, am.value, ac.options '
                                  'from amcl_measurements as am '
                                  'join amcl_configurations as ac '
                                  'on am.id = ac.id '
                                  'where ac.options = "' + option + ' ' + str(value) + '" ' +
                                  'and am.created_at > (NOW() - INTERVAL 1 DAY) '
                                  'and am.nfp_id = ' + str(nfp_id))

    x = []

    for element in configurations:
        if element[1] is not None:
            x.append(float(element[1]))

    explore = 'explore'
    plot_box_plot(x, explore + '_' + measurement, show=True, save=False)

    measurement = 'time'
    nfp_id = mdb.get_nfp_id(measurement)
    configurations = mdb.exec_sql('select am.id, am.value, ac.options '
                                  'from amcl_measurements as am '
                                  'join amcl_configurations as ac '
                                  'on am.id = ac.id '
                                  'where ac.options = "' + option + ' ' + str(value) + '" ' +
                                  'and am.created_at > (NOW() - INTERVAL 1 DAY) '
                                  'and am.nfp_id = ' + str(nfp_id))

    x = []

    for element in configurations:
        if element[1] is not None:
            x.append(float(element[1]))

    explore = 'explore'
    plot_box_plot(x, explore + '_' + measurement, show=True, save=False)

    mdb.shutdown()


def amcl_min_particles(min_particles=500):
    mdb.startup()

    measurement = 'mean_localization_error'
    nfp_id = mdb.get_nfp_id(measurement)

    configurations = mdb.exec_sql('select am.id, am.value, ac.options, ac.created_at '
                                  'from amcl_measurements as am '
                                  'join amcl_configurations as ac '
                                  'on am.id = ac.id '
                                  'where ac.options like "min_particles ' + str(min_particles) + ',%" '
                                  'and am.created_at > (NOW() - INTERVAL 1 DAY) '
                                  'and am.nfp_id = ' + str(nfp_id))

    data_time = str(configurations[0][-1]).split(' ')[0].strip()
    print data_time

    particles_value = []

    for element in configurations:
        if element[1] is not None:
            particles_value.append((int(element[2].split(',')[1].strip().split(' ')[1]), float(element[1])))

    particles_value.sort()

    print particles_value
    x = []
    y = []

    for element in particles_value:
        x.append(element[0])
        y.append(element[1])

    plot_graph(x, y, 'max_particles', measurement, [0, 10000, min(y) - 0.02, max(y) + 0.02], data_time + "_"
               + str(min_particles) + '_min_particles_' + measurement, save=False)

    measurement = 'mean_cpu_utilization'
    nfp_id = mdb.get_nfp_id(measurement)
    configurations = mdb.exec_sql('select am.id, am.value, ac.options '
                                  'from amcl_measurements as am '
                                  'join amcl_configurations as ac '
                                  'on am.id = ac.id '
                                  'where ac.options like "min_particles ' + str(min_particles) + ',%" '
                                  'and am.created_at > (NOW() - INTERVAL 1 DAY) '
                                  'and am.nfp_id = ' + str(nfp_id))

    particles_value = []

    for element in configurations:
        if element[1] is not None:
            particles_value.append((int(element[2].split(',')[1].strip().split(' ')[1]), float(element[1])))

    particles_value.sort()

    x = []
    y = []

    for element in particles_value:
        x.append(element[0])
        y.append(element[1])

    plot_graph(x, y, 'max_particles', measurement, [0, 10000, min(y) - 0.05, max(y) + 0.05], data_time + "_" +
               str(min_particles) + '_min_particles_' + measurement, save=False)

    measurement = 'time'
    nfp_id = mdb.get_nfp_id(measurement)
    configurations = mdb.exec_sql('select am.id, am.value, ac.options '
                                  'from amcl_measurements as am '
                                  'join amcl_configurations as ac '
                                  'on am.id = ac.id '
                                  'where ac.options like "min_particles ' + str(min_particles) + ',%" '
                                  'and am.created_at > (NOW() - INTERVAL 1 DAY) '
                                  'and am.nfp_id = ' + str(nfp_id))

    particles_value = []

    for element in configurations:
        if element[1] is not None and element[1] != '100':
            particles_value.append((int(element[2].split(',')[1].strip().split(' ')[1]), float(element[1])))

    particles_value.sort()

    x = []
    y = []

    for element in particles_value:
        x.append(element[0])
        y.append(element[1])

    plot_graph(x, y, 'max_particles', measurement, [0, 10000, min(y) - 0.05, max(y) + 0.05], data_time + "_" +
               str(min_particles) + '_min_particles_' + measurement, save=False)

    mdb.shutdown()


def amcl_option():
    mdb.startup()

    option = 'laser_z_hit'

    measurement = 'mean_localization_error'
    nfp_id = mdb.get_nfp_id(measurement)

    configurations = mdb.exec_sql('select AVG(am.value), ac.options, ac.created_at '
                                  'from amcl_measurements as am '
                                  'join amcl_configurations as ac on am.id = ac.id '
                                  'where ac.options like "' + option + '%" '
                                  'and DATE(ac.created_at) = "2016-10-12" '
                                  'and am.nfp_id = ' + str(nfp_id) + ' '
                                  'group by ac.options')

    data_time = str(configurations[0][-1]).split(' ')[0].strip()

    x = []
    y = []

    for element in configurations:
        x.append(float(element[1].split(',')[0].strip().split(' ')[1]))
        y.append(element[0])

    x, y = (list(t) for t in zip(*sorted(zip(x, y))))

    plot_graph(x, y, option, measurement, [min(x), max(x), min(y) - 0.02, max(y) + 0.02],
               data_time + '_' + option + '_' + measurement, save=True)

    measurement = 'mean_cpu_utilization'
    nfp_id = mdb.get_nfp_id(measurement)

    configurations = mdb.exec_sql('select AVG(am.value), ac.options, ac.created_at '
                                  'from amcl_measurements as am '
                                  'join amcl_configurations as ac on am.id = ac.id '
                                  'where ac.options like "' + option + '%" '
                                  'and DATE(ac.created_at) = "2016-10-12" '
                                  'and am.nfp_id = ' + str(nfp_id) + ' '
                                  'group by ac.options')

    data_time = str(configurations[0][-1]).split(' ')[0].strip()

    x = []
    y = []

    for element in configurations:
        x.append(float(element[1].split(',')[0].strip().split(' ')[1]))
        y.append(element[0])

    x, y = (list(t) for t in zip(*sorted(zip(x, y))))

    plot_graph(x, y, option, measurement, [min(x), max(x), min(y) - 0.02, max(y) + 0.02],
               data_time + '_' + option + '_' + measurement, save=True)

    measurement = 'time'
    nfp_id = mdb.get_nfp_id(measurement)

    configurations = mdb.exec_sql('select AVG(am.value), ac.options, ac.created_at '
                                  'from amcl_measurements as am '
                                  'join amcl_configurations as ac on am.id = ac.id '
                                  'where ac.options like "' + option + '%" '
                                  'and DATE(ac.created_at) = "2016-10-12" '
                                  'and am.nfp_id = ' + str(nfp_id) + ' '
                                  'and am.value != "100" '
                                  'group by ac.options')

    data_time = str(configurations[0][-1]).split(' ')[0].strip()

    x = []
    y = []

    for element in configurations:
        x.append(float(element[1].split(',')[0].strip().split(' ')[1]))
        y.append(element[0])

    x, y = (list(t) for t in zip(*sorted(zip(x, y))))

    plot_graph(x, y, option, measurement, [min(x), max(x), min(y) - 0.02, max(y) + 0.02],
               data_time + '_' + option + '_' + measurement, save=True)

    mdb.shutdown()


def amcl_default_consistency(db):
    mdb.startup(db)
    host = 'feature12.andrew.cmu.edu'

    default_configuration = mdb.exec_sql('select configuration_id, created_at from measurements where host = "{0}" '
                                         'ORDER BY created_at DESC limit 1'.format(host))

    data_time = str(default_configuration[0][-1]).split(' ')[0].strip()
    configuration_id = str(default_configuration[0][0])

    measurement = 'mean_localization_error'
    nfp_id = mdb.get_nfp_id(measurement)
    measurements = mdb.exec_sql('select value from measurements where nfp_id = {0} and configuration_id = "{1}" '
                                'and host = "{2}" and DATE(measurements.created_at) >  DATE(NOW() -  INTERVAL 2 DAY)'.format(str(nfp_id), configuration_id, host))

    x = []

    for element in measurements:
        # value column
        if element[0] != 'None':
            x.append(float(element[0]))

    consistency = 'consistency'
    plot_box_plot(x, data_time + "_" + "_" + host + "_" + consistency + '_' + measurement, show=True, save=False)

    measurement = 'mean_cpu_utilization'
    nfp_id = mdb.get_nfp_id(measurement)
    measurements = mdb.exec_sql('select value from measurements where nfp_id = {0} and configuration_id = "{1}" '
                                'and host = "{2}" and DATE(measurements.created_at) >  DATE(NOW() -  INTERVAL 2 DAY)'.format(str(nfp_id), configuration_id, host))

    x = []

    for element in measurements:
        # value column
        if element[0] != 'None':
            x.append(float(element[0]))

    consistency = 'consistency'
    plot_box_plot(x, data_time + "_" + "_" + host + "_" + consistency + '_' + measurement, show=True, save=False)

    measurement = 'time'
    nfp_id = mdb.get_nfp_id(measurement)
    measurements = mdb.exec_sql('select value from measurements where nfp_id = {0} and configuration_id = "{1}" '
                                'and host = "{2}" and DATE(measurements.created_at) >  DATE(NOW() -  INTERVAL 2 DAY)'.format(str(nfp_id), configuration_id, host))

    x = []

    for element in measurements:
        # value column
        if element[0] != 'None' and element[0] != '100':
            x.append(float(element[0]))

    consistency = 'consistency'
    plot_box_plot(x, data_time + "_" + "_" + host + "_" + consistency + '_' + measurement, show=True, save=False)

    measurement = 'mean_amcl_cpu_utilization'
    nfp_id = mdb.get_nfp_id(measurement)
    measurements = mdb.exec_sql('select value from measurements where nfp_id = {0} and configuration_id = "{1}" '
                                'and host = "{2}" and DATE(measurements.created_at) >  DATE(NOW() -  INTERVAL 2 DAY)'.format(str(nfp_id), configuration_id, host))

    x = []

    for element in measurements:
        # value column
        if element[0] != 'None':
            x.append(float(element[0]))

    consistency = 'consistency'
    plot_box_plot(x, data_time + "_" + "_" + host + "_" + consistency + '_' + measurement, show=True, save=False)

    mdb.shutdown()
