import jetway.mdb as mdb

turtlebot_explore_db = 'turtlebot-explore'

servers = ['feature3.andrew.cmu.edu', 'feature6.andrew.cmu.edu', 'feature8.andrew.cmu.edu',
             'feature10.andrew.cmu.edu', 'feature4.andrew.cmu.edu', 'feature7.andrew.cmu.edu', 'feature9.andrew.cmu.edu',
             'feature11.andrew.cmu.edu']

# Note that whichever mixture weights are in use should sum to 1.
# The beam model uses all 4: z_hit, z_short, z_max, and z_rand.
# The likelihood_field model uses only 2: z_hit and z_rand.


def add_default_configuration(db):
    mdb.startup(db)

    configuration = ''

    for option in filter_parameters:
        configuration += str(option[0]) + ' ' + str(option[1]) + ', '

    for option in filter_parameters_combine:
        configuration += str(option[0]) + ' ' + str(option[1]) + ', '

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

    # 9 servers * 40 times
    mdb.add_todo(id, 360)

    mdb.shutdown()


def add_combine_configurations_to_explore(db, configurations, values):
    mdb.startup(db)

    i = 0
    while i < len(configurations):
        configuration = configurations[i]
        value = values[i]

        for option_value in value:
            option = configuration[1] + ' ' + str(option_value) + ', ' + configuration[2] + ' ' + str(option_value)
            existing_id = mdb.select_ids('from configurations where options = "{0}"'.format(option))

            if len(existing_id) > 0:
                id = existing_id[0]
            else:
                id = mdb.add_configuration(option)

            # for server in servers:
            #     # 9 servers * 5 times each
            #     mdb.add_todo(id, 5, worker=server)
            mdb.add_todo(id, 10, worker=servers[0])
            mdb.add_todo(id, 10, worker=servers[1])

        i += 1

    mdb.shutdown()


def add_configurations_to_explore(db, configurations, values):
    mdb.startup(db)
    servers_1 = [server for server in servers[:4]]
    servers_2 = [server for server in servers[4:]]

    i = 0
    while i < len(configurations):
        configuration = configurations[i]
        for value in values[i]:
            option = configuration + ' ' + str(value)
            existing_id = mdb.select_ids('from configurations where options = "{0}"'.format(option))

            if len(existing_id) > 0:
                id = existing_id[0]
            else:
                id = mdb.add_configuration(option)

            mdb.add_todo(id, 10, worker=servers_1[0])
            mdb.add_todo(id, 10, worker=servers_1[1])
            # for server in servers:
            #     mdb.add_todo(id, 5, worker=server)

        i += 1

    mdb.shutdown()


def add_configuration(db, configuration, workers=None, priority=None, iterations=5):
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


def add_min_and_max_configurations(db, options, iterations=5):
    mdb.startup(db)
    servers_1 = [server for server in servers[:4]]
    servers_2 = [server for server in servers[4:]]

    i = 0
    while i < len(options):
        configuration = str(options[i][0]) + ' ' + str(options[i][-2])
        existing_id = mdb.select_ids('from configurations where options = "{0}"'.format(configuration))

        if len(existing_id) > 0:
            id = existing_id[0]
        else:
            id = mdb.add_configuration(configuration)

        # mdb.add_todo(id, iterations, worker=servers_1[i % len(servers_1)])
        # mdb.add_todo(id, iterations, worker=servers_2[i % len(servers_2)])
        mdb.add_todo(id, iterations, worker=servers_1[0])
        mdb.add_todo(id, iterations, worker=servers_1[1])

        configuration = str(options[i][0]) + ' ' + str(options[i][-1])
        existing_id = mdb.select_ids('from configurations where options = "{0}"'.format(configuration))

        if len(existing_id) > 0:
            id = existing_id[0]
        else:
            id = mdb.add_configuration(configuration)

        # mdb.add_todo(id, iterations, worker=servers_1[i % len(servers_1)])
        # mdb.add_todo(id, iterations, worker=servers_2[i % len(servers_2)])
        mdb.add_todo(id, iterations, worker=servers_1[0])
        mdb.add_todo(id, iterations, worker=servers_1[1])

        i += 1

    mdb.shutdown()


filter_parameters = [('kld_err', 0.01, 0.0, 1.0), ('kld_z', 0.99, 0.0, 1.0), ('update_min_d', 0.2, 0.0, 5.0),
                     ('update_min_a', 3.14159/6.0, 0.0, 6.28318), ('resample_interval', 2, 1, 20),
                     ('transform_tolerance', 0.1, 0.0, 2.0), ('recovery_alpha_slow', 0.0, 0.0, 0.5),
                     ('recovery_alpha_fast', 0.0, 0.0, 1.0), ('gui_publish_rate', -1.0, 10.0, 100.0),
                     ('save_pose_rate', 0.5, 0.0, 10.0)
                     ]
filter_parameters_combine = [['particles', ('min_particles', 100, 5, 1000), ('max_particles', 5000, 5, 1000)]] # max_particles -> max = 10000
filter_parameters_boolean = [('use_map_topic', False)]


filter_parameters_to_explore = ['recovery_alpha_slow', 'update_min_d', 'update_min_a', 'save_pose_rate',
                             'resample_interval',  'transform_tolerance'
                                ]
filter_parameters_to_explore_values = [[0.0, 0.1, 0.2, 0.3, 0.4, 0.5],
                                       [0.0, 0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4, 1.6, 1.8, 2.0, 2.2, 2.4, 2.6, 2.8, 3.0,
                                        3.2, 3.4, 3.6, 3.8, 4.0, 4.2, 4.4, 4.6, 4.8, 5.0],
                                       [0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0, 6.28318],
                                       [0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0, 6.5, 7.0, 7.5,
                                        8.0, 8.5, 9.0, 9.5, 10.0],
                                       [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20],
                                       [0.0, 0.1, 0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4, 1.6, 1.8, 2.0]
                                       ]
filter_parameters_combine_to_explore = [('particles', 'min_particles', 'max_particles')]
filter_parameters_combine_to_explore_values = [[5, 10, 20, 30, 40, 50, 75, 100, 150, 200, 250, 300, 400, 500, 600, 700,
                                                800, 900, 1000]]


laser_parameters = [('laser_max_beams', 30, 1, 100), ('laser_z_hit', 0.95, 0.1, 10.0), ('laser_z_short', 0.1, 0.01, 10.0),
                    ('laser_z_max', 0.05, 0.01, 10.0), ('laser_z_rand', 0.05, 0.01, 10.0),
                    ('laser_sigma_hit', 0.2, 0.1, 10.0), ('laser_lambda_short', 0.1, 0.01, 10.0),
                    ('laser_likelihood_max_dist', 2.0, 0.1, 20.0)
                    ]
laser_parameters_combine = [('laser_min_range', -1.0, 1.0, 1000.0), ('laser_max_range', -1.0, 1.0, 1000.0)]
laser_parameters_string = [('laser_model_type', "'likelihood_field'")]

laser_parameters_to_explore = ['laser_max_beams', 'laser_z_hit', 'laser_z_short', 'laser_z_max', 'laser_z_rand',
                               'laser_sigma_hit', 'laser_likelihood_max_dist'
                               ]
laser_parameters_to_explore_values = [[1, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100],
                                      [0.1, 0.95, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0],
                                      [0.01, 0.1, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0],
                                      [0.01, 0.05, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0],
                                      [0.01, 0.05, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0],
                                      [0.1, 0.2, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0],
                                      [0.1, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 15.0, 20.0]
                                      ]


odometry_parameters = [('odom_alpha1', 0.2, 0.1, 10.0), ('odom_alpha2', 0.2, 0.1, 10.0), ('odom_alpha3', 0.2, 0.1, 10.0),
                       ('odom_alpha4', 0.2, 0.1, 10.0), ('odom_alpha5', 0.2, 0.1, 10.0)
                       ]
odometry_parameters_string = [('odom_model_type', "'diff'"), ('odom_frame_id', "'odom'"), ('base_frame_id', "'base_link'"),
                              ('global_frame_id', "'map'")]
odometry_parameters_boolean = [('tf_broadcast', True)]


amcl_parameters_not_in_website = ['beam_skip_distance', 'beam_skip_threshold', 'first_map_only', 'restore_defaults',
                                  'do_beamskip']


environment_parameters = [('kinect_miscalibration', 0, -0.7, 0.7), ('kinect_noise', 0, 0.1, 1.0),
                          ('odometry_miscalibration', 0, -0.7, 0.7), ('odometry_noise', 0, 0.1, 1.0)]
environment_parameters_to_explore = ['kinect_miscalibration', 'kinect_noise', 'odometry_miscalibration',
                                     'odometry_noise']
environment_parameters_to_explore_values = [[-0.7, -0.6, -0.5, -0.4, -0.3, -0.2, -0.1, 0.0, 0.1, 0.2, 0.3, 0.4, 0.5,
                                             0.6, 0.7],
                                            [0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0],
                                            [-0.7, -0.6, -0.5, -0.4, -0.3, -0.2, -0.1, 0.0, 0.1, 0.2, 0.3, 0.4, 0.5,
                                             0.6, 0.7],
                                            [0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]
                                            ]
