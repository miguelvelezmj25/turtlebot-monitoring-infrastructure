import jetway.mdb as mdb

turtlebot_explore_db = 'turtlebot-explore'

servers = ['feature3.andrew.cmu.edu', 'feature6.andrew.cmu.edu', 'feature8.andrew.cmu.edu',
           'feature10.andrew.cmu.edu', 'feature4.andrew.cmu.edu', 'feature7.andrew.cmu.edu', 'feature9.andrew.cmu.edu',
           'feature11.andrew.cmu.edu']


# Note that whichever mixture weights are in use should sum to 1.
# The beam model uses all 4: z_hit, z_short, z_max, and z_rand.
# The likelihood_field model uses only 2: z_hit and z_rand.


def add_default_configuration(db, workers=None, priority=None, iterations=30):
    mdb.startup(db)

    configuration = ''

    for option in filter_parameters:
        configuration += str(option[0]) + ' ' + str(option[1]) + ', '

    for options in filter_parameters_combine:
        options = options[1:]
        for element in options:
            configuration += str(element[0]) + ' ' + str(element[1]) + ', '

    for option in filter_parameters_boolean:
        configuration += str(option[0]) + ' ' + str(option[1]) + ', '

    for option in laser_parameters:
        configuration += str(option[0]) + ' ' + str(option[1]) + ', '

    for option in laser_parameters_combine:
        configuration += str(option[0]) + ' ' + str(option[1]) + ', '

    for option in laser_parameters_string:
        configuration += str(option[0]) + ' ' + str(option[1]) + ', '

    for option in odometry_parameters:
        configuration += str(option[0]) + ' ' + str(option[1]) + ', '

    for option in odometry_parameters_string:
        configuration += str(option[0]) + ' ' + str(option[1]) + ', '

    for option in odometry_parameters_boolean:
        configuration += str(option[0]) + ' ' + str(option[1]) + ', '

    configuration = configuration[:-2]
    existing_id = mdb.select_ids('from configurations where options = "{0}"'.format(configuration))

    if len(existing_id) > 0:
        id = existing_id[0]
    else:
        id = mdb.add_configuration(configuration)

    if workers is None:
        mdb.add_todo(id, iterations, priority=priority)
    else:
        for worker in workers:
            mdb.add_todo(id, iterations, worker=worker, priority=priority)

    mdb.shutdown()


def add_combine_configurations_to_explore(db, configurations, values, workers=None, priority=None, iterations=6):
    mdb.startup(db)

    i = 0
    while i < len(configurations):
        configuration = configurations[i]
        value = values[i]

        for option_value in value:
            option = configuration[1] + ' ' + str(option_value) + ', ' + configuration[2] + ' ' + str(option_value)
            add_configuration(db, option, workers=workers, priority=priority, iterations=iterations)

        i += 1

    mdb.shutdown()


def add_configurations_to_explore(db, configurations, values, workers=None, priority=None, iterations=6):
    i = 0
    while i < len(configurations):
        configuration = configurations[i]
        for value in values[i]:
            option = configuration + ' ' + str(value)
            add_configuration(db, option, workers=workers, priority=priority, iterations=iterations)

        i += 1


def add_configuration(db, configuration, workers=None, priority=None, iterations=6):
    mdb.startup(db)

    existing_id = mdb.select_ids('from configurations where options = "{0}"'.format(configuration))

    if len(existing_id) > 0:
        id = existing_id[0]
    else:
        id = mdb.add_configuration(configuration)

    if workers is None:
        mdb.add_todo(id, iterations, priority=priority)
    else:
        for worker in workers:
            mdb.add_todo(id, iterations, worker=worker, priority=priority)

    mdb.shutdown()


def add_pair_wise_configurations(db, pair_wise_configurations, pair_wise_values):
    mdb.startup(db)
    servers_1 = [server for server in servers[:4]]
    servers_2 = [server for server in servers[4:]]

    i = 0
    while i < len(pair_wise_configurations):
        configurations = pair_wise_configurations[i]
        configurations_values = pair_wise_values[i]

        for x in configurations_values[0]:
            option = configurations[0] + ' ' + str(x) + ', '

            for y in configurations_values[1]:
                if y < x:
                    continue

                hold = option + configurations[1] + ' ' + str(y)

                existing_id = mdb.select_ids('from configurations where options = "{0}"'.format(hold))

                if len(existing_id) > 0:
                    id = existing_id[0]
                else:
                    id = mdb.add_configuration(hold)

                mdb.add_todo(id, 6, worker=servers_1[0])
                mdb.add_todo(id, 6, worker=servers_1[1])

        i += 1

    mdb.shutdown()


def add_min_and_max_configurations(db, options, workers=None, priority=None, iterations=6):
    i = 0
    while i < len(options):
        configuration = str(options[i][0]) + ' ' + str(options[i][-2])
        add_configuration(db, configuration, workers=workers, priority=priority, iterations=iterations)

        configuration = str(options[i][0]) + ' ' + str(options[i][-1])
        add_configuration(db, configuration, workers=workers, priority=priority, iterations=iterations)

        i += 1


filter_parameters = [('kld_err', 0.01, 0.0, 1.0), ('kld_z', 0.99, 0.0, 1.0), ('update_min_d', 0.2, 0.0, 5.0),
                     ('update_min_a', 3.14159 / 6.0, 0.0, 6.28318), ('resample_interval', 2, 1, 20),
                     ('transform_tolerance', 0.1, 0.0, 2.0), ('recovery_alpha_slow', 0.0, 0.0, 0.5),
                     ('recovery_alpha_fast', 0.0, 0.0, 1.0), ('gui_publish_rate', -1.0, 10.0, 100.0),
                     ('save_pose_rate', 0.5, 0.0, 10.0)
                     ]
filter_parameters_combine = [
    ['particles', ('min_particles', 100, 5, 1000), ('max_particles', 5000, 5, 1000)]]  # max_particles -> max = 10000
filter_parameters_boolean = [('use_map_topic', False)]

filter_parameters_to_explore = ['recovery_alpha_slow', 'update_min_d', 'update_min_a', 'save_pose_rate',
                                'resample_interval', 'transform_tolerance'
                                ]
filter_parameters_to_explore_values = [[0.0, 0.1, 0.2, 0.3, 0.4, 0.5],
                                       [0.0, 0.2, 0.4, 0.6, 0.8, 1.0, 2.0, 3.0, 4.0, 5.0],
                                       [0.0, 0.5, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 6.28318],
                                       [0.0, 0.5, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0],
                                       [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 15, 20],
                                       [0.0, 0.1, 0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4, 1.6, 1.8, 2.0]
                                       ]
filter_parameters_combine_to_explore = [('particles', 'min_particles', 'max_particles')]
filter_parameters_combine_to_explore_values = [[5, 10, 20, 30, 40, 50, 70, 100, 125, 150, 200, 250, 300, 350, 400, 450,
                                                500, 750, 1000]
                                              ]


laser_parameters = [('laser_max_beams', 30, 1, 100), ('laser_z_hit', 0.95, 0.1, 10.0),
                    ('laser_z_short', 0.1, 0.01, 10.0),
                    ('laser_z_max', 0.05, 0.01, 10.0), ('laser_z_rand', 0.05, 0.01, 10.0),
                    ('laser_sigma_hit', 0.2, 0.1, 10.0), ('laser_lambda_short', 0.1, 0.01, 10.0),
                    ('laser_likelihood_max_dist', 2.0, 0.1, 20.0)
                    ]
laser_parameters_combine = [('laser_min_range', -1.0, 1.0, 1000.0), ('laser_max_range', -1.0, 1.0, 1000.0)]
laser_parameters_string = [('laser_model_type', "'likelihood_field'")]

odometry_parameters = [('odom_alpha1', 0.2, 0.1, 10.0), ('odom_alpha2', 0.2, 0.1, 10.0),
                       ('odom_alpha3', 0.2, 0.1, 10.0),
                       ('odom_alpha4', 0.2, 0.1, 10.0), ('odom_alpha5', 0.2, 0.1, 10.0)
                       ]
odometry_parameters_string = [('odom_model_type', "'diff'"), ('odom_frame_id', "'odom'"),
                              ('base_frame_id', "'base_link'"),
                              ('global_frame_id', "'map'")]
odometry_parameters_boolean = [('tf_broadcast', True)]

amcl_parameters_not_in_website = ['beam_skip_distance', 'beam_skip_threshold', 'first_map_only', 'restore_defaults',
                                  'do_beamskip']

environment_parameters = [('kinect_miscalibration', 0, -0.7, 0.7), ('kinect_noise', 0, 0.1, 1.0),
                          ('odometry_miscalibration', 0, -0.7, 0.7), ('odometry_noise', 0, 0.1, 1.0),
                          ('kinect_array', 640, 0, 640)]
environment_parameters_to_explore = ['kinect_miscalibration', 'kinect_noise', 'odometry_miscalibration',
                                     'odometry_noise', 'kinect_array']
environment_parameters_to_explore_values = [[-0.7, -0.6, -0.5, -0.4, -0.3, -0.2, -0.1, 0.0, 0.1, 0.2, 0.3, 0.4, 0.5,
                                             0.6, 0.7],
                                            [0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0],
                                            [-0.7, -0.6, -0.5, -0.4, -0.3, -0.2, -0.1, 0.0, 0.1, 0.2, 0.3, 0.4, 0.5,
                                             0.6, 0.7],
                                            [0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0],
                                            [0, 50, 75, 100, 150, 200, 250, 300, 350, 400, 450, 500, 550, 600, 640]
                                            ]

custom_configurations = ['min_particles 5, max_particles 5, resample_interval 20',
                         'min_particles 1000, max_particles 1000, resample_interval 1']

pair_wise_configurations = [('min_particles', 'max_particles')]
pair_wise_configurations_values = [[(5, 10, 20, 30, 40, 50, 70, 100, 125, 150, 200, 250, 300, 350, 400, 450, 500, 750,
                                     1000), (5, 10, 20, 30, 40, 50, 70, 100, 125, 150, 200, 250, 300, 350, 400, 450,
                                             500, 750, 1000, 1500, 2000, 3500, 5000, 7500, 10000)]]
