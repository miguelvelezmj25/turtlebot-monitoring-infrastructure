# Infrastructure

This folder contains the scripts used to measure the performance and behavior 
of the TurtleBot. It uses a combination of Python, bash, and R scripts to execute 
and monitor TurtleBot, and graph the results to compare and analyze the execution.

The following picture presents a view of the infrastructure

![Infrastructure](visuals/Infrastructure.png)

The master machine can run any OS. The TurtleBot and Simulator machines have to run
Linux.

## Technical Requirements

* [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) (Required. Install in TurtleBot 
and Simulator machines)

  This is the version of ROS used for the experiments.

* [TurtleBot Indigo](http://wiki.ros.org/turtlebot/Tutorials/indigo/Turtlebot%20Installation) (Required. 
Install in TurtleBot and Simulator machines)

  This is the TurtleBot simulator for the Indigo version of ROS.

* [Jetway](https://github.com/miguelvelezmj25/Jetway)
  
  This is a forked repo from Christian Kästner and adapted for this project. It is mainly used to communicate 
with a database. 

* [Fabric](http://www.fabfile.org/) (Optional. Install in master machine)
  
  This command-line tool is installed and used in the master machine for streamlining 
the use of SSH for application deployment or systems administration tasks. It is
used to send jobs and manage the machines that execute the experiments

* [Fabfiles](https://github.com/miguelvelezmj25/fabfiles) (Optional. Clone in master machine)

  This project contains a collection of fabfiles used along side Fabric to manage and
send jobs to multiple machines. This project contains files needed in the simulator machine
if you run your experiments headless. It also contains scripts for running the experiments.

* sshpass (Required. Install in TurtleBot machine)

  This Linux utility allows to run ssh using the mode referred to as "keyboard-interactive" 
password authentication, but in non-interactive mode. This is used in a bash scripts
to initialize and terminate processes for the gazebo simulator in order machines. You 
can install it by typing:
 
        sudo apt-get install sshpass

* mpstat (Required. Install in TurtleBot machine)

  Part of Sysstat. This Linux command writes to standard output activities for each available processor.
It is used to measure CPU utilization of an entire machine. You can install it by typing:
 
        sudo apt-get install sysstat

* Xorg dummy driver (Required to run the simulator headless. Install in  Simulator machine)

  This driver allows to run the simulator headless. You can install it by typing:
 
        sudo apt-get install xserver-xorg-video-dummy


## Additional Files

The following files contain sensitive data that is unique to each individual who uses this
repo. Therefore, they are not provided in this repo and must be copied in the machines running
the TurtleBot and Simulator. You can use the [Fabfiles](https://github.com/miguelvelezmj25/fabfiles) 
project to copy them. 

* .dbconfig
  
  This file is used to connect to the database that stores the jobs and saves the data from
the executions. It must have the following structure:

        [server]
        hostname = hostname
        user = user
        password = password
        database = database

* .serverconfig

  This file is used to configure the communication between the TurtleBot and Simulator machines.
It mus have the following structure:

        [serverX]
        simulator = serverY
        username = username
        password = password
  
  This means that ```serverX``` will act as the TurtleBot machine and will communicate with ```serverY```
  to run the simulator in that machine. **WARNING: Machines cannot run TurtleBot code and the simulator
  at the same time**.
  
There are additional files needed to run the simulator headless. More information can be found in 
the [Fabfiles](https://github.com/miguelvelezmj25/fabfiles) repo.
  
## Database

This project requires a database to store the configurations of TurtleBot, the time
series and summary measurements, the non-functional properties to be measured, and
jobs to do. The following is the DDL from the database currently used.

    CREATE TABLE nfps
    (
        id INT(11) unsigned PRIMARY KEY NOT NULL AUTO_INCREMENT,
        name VARCHAR(300) DEFAULT '' NOT NULL,
        created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP NOT NULL
    );
    CREATE TABLE todos
    (
        configuration_id VARCHAR(36) DEFAULT '' NOT NULL,
        iterations INT(11) NOT NULL,
        worker VARCHAR(100) DEFAULT '' NOT NULL,
        priority INT(11),
        created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP NOT NULL,
        CONSTRAINT `PRIMARY` PRIMARY KEY (configuration_id, worker),
        CONSTRAINT todos_configurations_id_fk FOREIGN KEY (configuration_id) REFERENCES configurations (id)
    );
    CREATE TABLE configurations
    (
        id VARCHAR(36) PRIMARY KEY NOT NULL,
        options VARCHAR(760) DEFAULT '' NOT NULL,
        created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP NOT NULL
    );
    CREATE UNIQUE INDEX configurations_options_uindex ON configurations (options);
    CREATE TABLE measurements_verbose
    (
        configuration_id VARCHAR(36) DEFAULT '' NOT NULL,
        simulator VARCHAR(30) DEFAULT '' NOT NULL,
        host VARCHAR(30) DEFAULT '' NOT NULL,
        nfp_id INT(11) unsigned NOT NULL,
        value VARCHAR(20) DEFAULT '' NOT NULL,
        time VARCHAR(20) DEFAULT '' NOT NULL,
        created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP NOT NULL,
        CONSTRAINT measurements_verbose_configurations_id_fk FOREIGN KEY (configuration_id) REFERENCES configurations (id),
        CONSTRAINT measurements_verbose_nfps_id_fk FOREIGN KEY (nfp_id) REFERENCES nfps (id)
    );
    CREATE INDEX measurements_verbose_configurations_id_fk ON measurements_verbose (configuration_id);
    CREATE INDEX measurements_verbose_nfps_id_fk ON measurements_verbose (nfp_id);
    CREATE TABLE measurements
    (
        configuration_id VARCHAR(36) DEFAULT '' NOT NULL,
        simulator VARCHAR(30) DEFAULT '' NOT NULL,
        host VARCHAR(30) DEFAULT '' NOT NULL,
        nfp_id INT(11) unsigned NOT NULL,
        value VARCHAR(20),
        created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP NOT NULL,
        CONSTRAINT measurements_configurations_id_fk FOREIGN KEY (configuration_id) REFERENCES configurations (id),
        CONSTRAINT measurements_nfps_id_fk FOREIGN KEY (nfp_id) REFERENCES nfps (id)
    );
    CREATE INDEX measurements_configurations_id_fk ON measurements (configuration_id);
    CREATE INDEX measurements_nfps_id_fk ON measurements (nfp_id);
       
## Running Experiments

The following instructions are the ones we followed to get the infrastructure cloned and ready 
to run experiments. You can use a different approach besides Fabric to set up and update the infrastructure.
However, you must execute and follow the same logic that the scripts that use the fabfiles execute. You have
to check the [Fabfiles](https://github.com/miguelvelezmj25/fabfiles) repo for instructions on how to
complete some of the following steps.

1. Install and clone the require software, files, and projects in your master machine and your servers. The
[Fabfiles](https://github.com/miguelvelezmj25/fabfiles) project must be cloned in your master machine. 
This project must be cloned and setup in your servers. You can do so by running

        clone_infrastructure.sh {hosts} {password}
        set_bash_profile.sh {hosts} {password}

  from the master machine. They will setup this project, the necessary files used by it, and the ```.bash_profile``` in your servers. 
  Remember that you must provide the ```.dbconfig``` and ```.serverconfig``` files in your 
  [Fabfiles](https://github.com/miguelvelezmj25/fabfiles) project to be copied to your servers.

2. Run the headless server in your server machines. 

3. Add configurations and jobs to the database.

4. Execute the experiment you stored in the database. You can do so by running

        run_experiments.sh {hosts} {password} {iterations}
        
  from the master machine.

## Metrics

This is the list of the current metrics we are measuring are and the associated subscribers 
getting that data

* mean_cpu_utilization --> cpu_monitor.py
* mean_localization_error --> ground_truth_pose.py and estimate_pose.py
* time (This metric is not measure with a monitor file. Rather it is calculated from the simulator.)
* mean_amcl_cpu_utilization --> amcl_cpu_monitor.py

## Measuring Other Metrics 

The ```.monitors``` file contains the files that subscribe to a ROS topic and get data. If you want to measure 
more metrics, you need to add the files with the subscribers in this file. It is recommended to use only one 
subscriber per files. It is also adviced to not have multiple subscribers subcribed to one topic since the 
messages will be distributed to all subscribers and each subscriber will only get a subset of all the messages. 
If your subscriber is written in Python, you need to make that file executable and include the extension in the 
```.monitos``` files. If your subscriber is written in C++, you need add the file name with the extension as an
executable in the ```CMakelLists``` like
    
    add_executable({node_name} {path_to_file_with_extension})
    target_link_libraries({node_name} ${catkin_LIBRARIES})