# infrastructure

This folder contains the scripts used to measure the performance and behavior 
of the turtlebot. It uses a combination of Python, bash, and R scripts to execute 
and monitor Turtlebot, and graph the results to compare and analyze the execution.

## Required software

* [Jetway](https://github.com/miguelvelezmj25/Jetway)
This is a forked repo from Christian Kästner. It is meanly used to communicate 
with a database. 

* [Fabric](http://www.fabfile.org/)
This command-line tool is installed and used in the master machine or streamlining 
the use of SSH for application deployment or systems administration tasks. It is
used to send jobs and manage the machines that execute the experiments

* [Fabfiles](https://github.com/miguelvelezmj25/fabfiles)
This project contains collection of fabfiles used along side fabric to manage and
send jobs to multiple machines

## Database

This project requires a database to store the configurations of turtlebot, the time
series and summary measurements, the non-functional properties to be measured, and
jobs to do. The following is the DDL from the database we currently use

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
    
